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
	camera->film->WriteImage();
}

void BPTRenderer::TraceLightPath(const Scene *scene, vector<BPTVertex> &lightPath, float time, int px, int py){
	BPTVertex vertex;
	RayDifferential ray;
	float nLightPath = float(camera->film->xResolution * camera->film->yResolution);
	
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
		float tSquare = (position-ray.o).LengthSquared();
		vertex.bsdf=isect.GetBSDF(ray, arena);
		if(end==maxDepth && !scene->lights[lightIndex]->IsDeltaLight()){
			float lightSamplingPdf=scene->lights[lightIndex]->Pdf(vertex.bsdf->dgShading.p, -vertex.in);
			vertex.dvcm *= lightSamplingPdf * (cosAtLight / tSquare);
		}
		float cosTheta = AbsDot(normal, vertex.in);
		vertex.dvcm *= tSquare / cosTheta;
		vertex.dvc /= cosTheta;
		
		//store intersection
		lightPath.push_back(vertex);
		
		//connect to camera
		CameraSample sample;
		sample.lensU=rng.RandomFloat();
		sample.lensV=rng.RandomFloat();
		sample.time = time;
		VisibilityTester vis;
		Vector camDir;
		float camPdf;
		Spectrum camResponse = camera->Sample_Wi(position, &sample, &camDir, &camPdf, &vis);
		if(!camResponse.IsBlack() && camPdf != 0.f && vis.Unoccluded(scene)){
			Spectrum res = camResponse * vertex.bsdf->f(-vertex.in, camDir);
			res /= camPdf;
			float camDirPdfW;
			camera->Pdf_We(Ray(vis.r.o, camDir, 1e-3, INFINITY, vis.r.time), NULL, &camDirPdfW);
			float camDirPdfA = camDirPdfW * AbsDot(camDir, normal) / (position-vis.r.o).LengthSquared();
			float reverseBsdfPdf = vertex.bsdf->Pdf(camDir, -vertex.in);

			//calc MIS weights
			float wLight = camDirPdfA / nLightPath * (vertex.dvcm + reverseBsdfPdf * vertex.dvc);
			float weight = 1.f / (wLight + 1.f);

			//add contribution
			camera->film->Splat(sample, weight * res / float(samplesPerPixel));
			
		}
		
		//sample next direction
		float bsdfPdf;
		BxDFType sampledType;
		Vector out;
		Spectrum f = vertex.bsdf->Sample_f(-vertex.in, &out, BSDFSample(rng), &bsdfPdf, BSDF_ALL, &sampledType);
		
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
		float reversePdf = vertex.bsdf->Pdf(out, -vertex.in) * surviveProb;
		if(sampledType & BSDF_SPECULAR) {
			vertex.dvcm=0.f;
			vertex.dvc *= cosOut;
		}
		else {
			
			vertex.dvc = (cosOut / bsdfPdf) * (vertex.dvcm + reversePdf * vertex.dvc);
			vertex.dvcm = 1.f / bsdfPdf;
		}
		
		//update ray
		ray = RayDifferential(position, out, ray, isect.rayEpsilon);
		vertex.throughput *= f;
		vertex.in = out;
	}while(--end);
}

void BPTRenderer::TraceCameraPath(const Scene *scene, vector<BPTVertex> &lightPath, vector<BPTVertex> &cameraPath, float time, int px, int py){
	BPTVertex vertex;
	RayDifferential ray;
	Spectrum result(0.f);
	float nLightPath = float(camera->film->xResolution * camera->film->yResolution);

	//sample camera direction 
	CameraSample camSample;
	camSample.imageX = rng.RandomFloat() + float(px);
	camSample.imageY = rng.RandomFloat() + float(py);
	camSample.lensU = rng.RandomFloat();
	camSample.lensV = rng.RandomFloat();
	camSample.time = time;
	camera->GenerateRayDifferential(camSample, &ray);
	vertex.throughput = Spectrum(1.f);
	vertex.in = ray.d;
	//calc weights
	float camDirPdfW;
	camera->Pdf_We(ray, NULL, &camDirPdfW);
	vertex.dvcm = nLightPath / camDirPdfW;
	vertex.dvc = 0.f;

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
		float cosTheta = AbsDot(normal, vertex.in);
		vertex.dvcm *= tSquare / cosTheta;
		vertex.dvc /= cosTheta;

		//Check if hit light
		const AreaLight *l=isect.primitive->GetAreaLight();
		if(l){
			Spectrum Lemmit = l->L(position, normal, -vertex.in);
			float weight=1.f;
			if(end!=maxDepth){
				float pdfIllum = l->Pdf(position, vertex.in) * cosTheta / (tSquare * scene->lights.size());
				float wCam = pdfIllum * vertex.dvcm + INV_TWOPI * vertex.dvc;
				weight = 1.f / (wCam + 1.f);
			}
			result += Lemmit * vertex.throughput * weight;
		}

		//TODO connect to light
		int lightIndex = (int(rng.RandomFloat() * scene->lights.size())) % scene->lights.size();
		float lightPdf = 1.f / float(scene->lights.size());
		Vector lightDir;
		float lightPointPdfW;
		VisibilityTester vis;
		LightSample ls(rng);
		Spectrum L = scene->lights[lightIndex]->Sample_L(position, isect.rayEpsilon, ls, time, &lightDir, &lightPointPdfW, &vis);
		Spectrum bsdfFactor = vertex.bsdf->f(-vertex.in, lightDir);
		if(!L.IsBlack() && lightPointPdfW!=0.f && !bsdfFactor.IsBlack() && vis.Unoccluded(scene)){
			Spectrum contrib = (L / (lightPdf*lightPointPdfW)) * bsdfFactor* vertex.throughput;
			Point lightPoint;
			Normal lightNormal;
			float lEmitPointPdf, lEmitDirPdf;
			scene->lights[lightIndex]->Pdf_Le(ls, -lightDir, &lEmitPointPdf, &lEmitDirPdf, &lightPoint, &lightNormal);
			//calc weights
			float lDistSquare = (position - lightPoint).LengthSquared();
			float weight=1.f;
			if(scene->lights[lightIndex]->IsDeltaLight()){
				float wCam = lEmitDirPdf * AbsDot(lightDir, normal) / lDistSquare;
				wCam *= vertex.dvcm + vertex.bsdf->Pdf(lightDir, -vertex.in) * vertex.dvc;
				weight = 1.f / (wCam * 1.f);
			}
			else{
				float bsdfToLightPdfW = vertex.bsdf->Pdf(-vertex.in, lightDir);
				float cosAtLight = AbsDot(lightDir, lightNormal);
				float bsdfToLightPdfA = bsdfToLightPdfW * cosAtLight / lDistSquare;
				float lightPointPdfA = lightPointPdfW * AbsDot(lightDir, normal) / lDistSquare;
				float wLight = bsdfToLightPdfA / lightPointPdfA;

				float wCam = (lEmitPointPdf/lightPointPdfA) * 
							lEmitDirPdf * (AbsDot(lightDir, normal) / lDistSquare) * 
							(vertex.dvcm + vertex.bsdf->Pdf(lightDir, -vertex.in) * vertex.dvc);

				weight = 1.f / (wLight + 1.f + wCam);
			}
			result += contrib * weight;
		}

		//TODO connect to light path
		for(int i = 0; i<lightPath.size(); i++){

		}

		//sample next direction
		float bsdfPdf;
		BxDFType sampledType;
		Vector out;
		Spectrum f = vertex.bsdf->Sample_f(-vertex.in, &out, BSDFSample(rng), &bsdfPdf, BSDF_ALL, &sampledType);
		
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
		float reversePdf = vertex.bsdf->Pdf(out, -vertex.in) * surviveProb;
		if(sampledType & BSDF_SPECULAR) {
			vertex.dvcm=0.f;
			vertex.dvc *= cosOut;
		}
		else {
			
			vertex.dvc = (cosOut / bsdfPdf) * (vertex.dvcm + reversePdf * vertex.dvc);
			vertex.dvcm = 1.f / bsdfPdf;
		}
		
		//update ray
		ray = RayDifferential(position, out, ray, isect.rayEpsilon);
		vertex.throughput *= f;
		vertex.in = out;
	}while(--end);

	camera->film->Splat(camSample, result/float(samplesPerPixel));

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