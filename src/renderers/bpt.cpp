#include "parallel.h"
#include "paramset.h"
#include "camera.h"
#include "film.h"
#include "intersection.h"
#include "scene.h"
#include "memory.h"
#include "bpt.h"



struct BPTVertex
{
	/* data */
	Spectrum throughput;
	Vector in;
	BSDF *bsdf;
	float dvc;
	float dvcm;
};


class BPTTask : public Task {

};

void BPTRenderer::Render(const Scene *scene){

	vector<BPTVertex> lightPath;
	vector<BPTVertex> cameraPath;
	for(int i=0; i<camera->film->xResolution; i++){
		for (int j = 0; j < camera->film->yResolution; j++){
			for(int s=0; s<samplesPerPixel; s++){
				float time = rng.RandomFloat();
				lightPath.clear();
				cameraPath.clear();
				TraceLightPath(scene, lightPath, time, i,j);
				TraceCameraPath(scene, lightPath, cameraPath, time, i, j);
			}
		}
	}
}

void BPTRenderer::TraceLightPath(const Scene *scene, vector<BPTVertex> &lightPath, float time, int px, int py){
	BPTVertex vertex;
	RayDifferential ray;
	
	//sample light
	int lightIndex = (int(rng.RandomFloat() * scene->lights.size())) % scene->lights.size();
	float lightPdf = 1.f / float(scene->lights.size());
	float lightPointDirPdf;
	Normal lightNormal;
	vertex.throughput=scene->lights[lightIndex]->Sample_L(scene, LightSample(rng), rng.RandomFloat(), rng.RandomFloat(), time, &ray, &lightNormal, &lightPointDirPdf);
	
	//exit if samplig failed
	if(vertex.throughput.IsBlack() || lightPointDirPdf == 0.f)
		return;
	
	//select direction and init ray and weights
	float cosAtLight = AbsDot(Normalize(lightNormal), ray.d);
	vertex.throughput *=  cosAtLight / (lightPdf * lightPointDirPdf);
	vertex.in = ray.d;
	if(scene->lights[lightIndex]->IsDeltaLight()){
		vertex.dvcm = 1.f / lightPointDirPdf;
		vertex.dvc = 0.f;
	}
	else {
		vertex.dvcm = 1.f / lightPointDirPdf;
		vertex.dvc = cosAtLight / lightPointDirPdf;
	}

	//random walk
	int end=maxDepth;
	do{
		//intesect ray and exit if miss
		Intersection isect;
		if(!scene->Intersect(ray, &isect))
			return;
		Point position = vertex.bsdf->dgShading.p;
		Normal normal = vertex.bsdf->dgShading.nn;
		
		//update weights post intersection
		vertex.bsdf=isect.GetBSDF(ray, arena);
		if(end==maxDepth && !scene->lights[lightIndex]->IsDeltaLight()){
			float lightSamplingPdf=scene->lights[lightIndex]->Pdf(vertex.bsdf->dgShading.p, -ray.d);
			vertex.dvcm*=lightSamplingPdf;
		}
		float tSquare = (position-ray.o).LengthSquared();
		float cosTheta = AbsDot(normal, ray.d);
		vertex.dvcm *= tSquare / cosTheta;
		vertex.dvc /= cosTheta;
		
		//store intersection
		lightPath.push_back(vertex);
		
		//connect to camera
		
		//TODO CONNECT TO CAMERA
		
		//sample next direction
		float bsdfPdf;
		BxDFType sampledType;
		Vector out;
		Spectrum f = vertex.bsdf->Sample_f(-ray.d, &out, BSDFSample(rng), &bsdfPdf, BSDF_ALL, &sampledType);
		
		//exit if sample fails
		if(f.IsBlack() || bsdfPdf == 0.f)
			return;
		
		//calc attenuation
		float cosOut = AbsDot(out, normal);
		f *= cosOut / bsdfPdf;
		
		//russian roulete
		float surviveProb = min(f.y(), 1.f);
		if(rng.RandomFloat() > surviveProb)
			return;
		f /= surviveProb;
		
		//update weights pre intersection
		bsdfPdf *= surviveProb;
		float reversePdf = vertex.bsdf->Pdf(out, -ray.d) * surviveProb;
		if(sampledType & BSDF_SPECULAR) {
			vertex.dvcm=0.f;
			vertex.dvc *= cosOut;
		}
		else {
			
			vertex.dvc = (cosOut / bsdfPdf) * (vertex.dvcm + reversePdf * vertex.dvc);
			vertex.dvcm = 1.f / bsdfPdf;
		}
		
		//update ray
		ray.o = position;
		ray.d = out;
		vertex.throughput *= f;
		vertex.in = out;
	}while(--end);
}

void BPTRenderer::TraceCameraPath(const Scene *scene, vector<BPTVertex> &lightPath, vector<BPTVertex> &cameraPath, float time, int px, int py){
	BPTVertex vertex;
	RayDifferential ray;
	
	//sample camera direction and calc weights
	CameraSample camSample;
	camSample.imageX = rng.RandomFloat() + float(px);
	camSample.imageY = rng.RandomFloat() + float(py);
	camSample.lensU = rng.RandomFloat();
	camSample.lensV = rng.RandomFloat();
	camSample.time = time;

	camera->GenerateRayDifferential(camSample, &ray);

	int end=maxDepth;
	do{
		//intersect ray and exit if miss
		Intersection isect;
		if(!scene->Intersect(ray, &isect))
			return;
		Point position = vertex.bsdf->dgShading.p;
		Normal normal = vertex.bsdf->dgShading.nn;

		//update weights post untersection
		vertex.bsdf=isect.GetBSDF(ray, arena);
		float tSquare = (position-ray.o).LengthSquared();
		float cosTheta = AbsDot(normal, ray.d);
		vertex.dvcm *= tSquare / cosTheta;
		vertex.dvc /= cosTheta;

		//TODO connect to light

		//TODO connect to light path

		//sample next direction
		float bsdfPdf;
		BxDFType sampledType;
		Vector out;
		Spectrum f = vertex.bsdf->Sample_f(-ray.d, &out, BSDFSample(rng), &bsdfPdf, BSDF_ALL, &sampledType);
		
		//exit if sample fails
		if(f.IsBlack() || bsdfPdf == 0.f)
			return;
		
		//calc attenuation
		float cosOut = AbsDot(out, normal);
		f *= cosOut / bsdfPdf;
		
		//russian roulete
		float surviveProb = min(f.y(), 1.f);
		if(rng.RandomFloat() > surviveProb)
			return;
		f /= surviveProb;
		
		//update weights pre intersection
		bsdfPdf *= surviveProb;
		float reversePdf = vertex.bsdf->Pdf(out, -ray.d) * surviveProb;
		if(sampledType & BSDF_SPECULAR) {
			vertex.dvcm=0.f;
			vertex.dvc *= cosOut;
		}
		else {
			
			vertex.dvc = (cosOut / bsdfPdf) * (vertex.dvcm + reversePdf * vertex.dvc);
			vertex.dvcm = 1.f / bsdfPdf;
		}
		
		//update ray
		ray.o = position;
		ray.d = out;
		vertex.throughput *= f;
		vertex.in = out;
	}while(--end);
}


Spectrum BPTRenderer::Li(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena,
        Intersection *isect, Spectrum *T) const{
	return 0.f;
}

Spectrum BPTRenderer::Transmittance(const Scene *scene,
        const RayDifferential &ray, const Sample *sample,
        RNG &rng, MemoryArena &arena) const{
	return 1.f;
}

BPTRenderer *CreateBPTRenderer(const ParamSet &params, Camera *camera){
	int samplesPerPixel = params.FindOneInt("samplesPerPixel", 256);
	int maxDepth = params.FindOneInt("maxDepth", 10);
	return new BPTRenderer(samplesPerPixel, maxDepth, camera);
}