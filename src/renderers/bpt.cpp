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
	Intersection isect;
	float dvc;
	float dvcm;
};

class BPTTask : public Task {

};

void BPTRenderer::Render(const Scene *scene){

	vector<BPTVertex> lightPath;

	int x0, x1, y0, y1;
    camera->film->GetPixelExtent(&x0, &x1, &y0, &y1);
	
	for(int i=x0; i<x1; i++){
		for (int j = y0; j < y1; j++){
			for(int s=0; s<samplesPerPixel; s++){
				float time = rng.RandomFloat();
				lightPath.clear();
				TraceLightPath(scene, lightPath, time, i,j);
				TraceCameraPath(scene, lightPath, time, i, j);
				arena.FreeAll();
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
	vertex.throughput /= (lightPdf * lightPointDirPdf);
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
		if(!scene->Intersect(ray, &vertex.isect))
			break;
		vertex.bsdf=vertex.isect.GetBSDF(ray, arena);
		Point position = vertex.bsdf->dgShading.p;
		Normal normal = vertex.bsdf->dgShading.nn;
		
		//update weights post intersection
		float tSquare = (position-ray.o).LengthSquared();
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
			Spectrum bsdfFactor = vertex.bsdf->f(-vertex.in, camDir);
			Spectrum res = camResponse * bsdfFactor * vertex.throughput / camPdf;
			
			float camDirPdfW, camPointPdf;
			camera->Pdf_We(Ray(vis.r.o, camDir, 0, INFINITY, vis.r.time), &camPointPdf, &camDirPdfW);
			float camDirPdfA = camDirPdfW * AbsDot(camDir, normal) / (position-vis.r.o).LengthSquared();
			float reverseBsdfPdf = vertex.bsdf->Pdf(camDir, -vertex.in);
			float reverseSurviveProb = min(1.f,(bsdfFactor.y() * AbsDot(vertex.in, normal) / reverseBsdfPdf));
			reverseBsdfPdf *= reverseSurviveProb;
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
			break;
		
		//calc attenuation
		float cosOut = AbsDot(out, normal);
		f *= cosOut / bsdfPdf;
		
		//russian roulete
		float surviveProb = min(f.y(), 1.f);
		if(rng.RandomFloat() > surviveProb)
			break;
		f /= surviveProb;
		
		//update weights pre intersection
		bsdfPdf *= surviveProb;
		float reversePdf = vertex.bsdf->Pdf(out, -vertex.in);
		float reverseSurviveProb = min(1.f, vertex.bsdf->f(out, -vertex.in).y()*AbsDot(vertex.in, normal) / reversePdf);
		reversePdf *= reverseSurviveProb;

		if(sampledType & BSDF_SPECULAR) {
			vertex.dvcm=0.f;
			vertex.dvc *= cosOut;
		}
		else {
			
			vertex.dvc = (cosOut / bsdfPdf) * (vertex.dvcm + reversePdf * vertex.dvc);
			vertex.dvcm = 1.f / bsdfPdf;
		}
		
		//update ray
		ray = RayDifferential(position, out, ray, vertex.isect.rayEpsilon);
		vertex.throughput *= f;
		vertex.in = out;
	}while(--end);
}

void BPTRenderer::TraceCameraPath(const Scene *scene, vector<BPTVertex> &lightPath, float time, int px, int py){
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
	float camDirPdfW, camPointPdf;
	camera->Pdf_We(ray, &camPointPdf, &camDirPdfW);
	vertex.dvcm = nLightPath / camDirPdfW;
	vertex.dvc = 0.f;

	int end=maxDepth;
	do{
		//intersect ray and exit if miss
		
		if(!scene->Intersect(ray, &vertex.isect))
			break;
		vertex.bsdf=vertex.isect.GetBSDF(ray, arena);
		Point position = vertex.bsdf->dgShading.p;
		Normal normal = vertex.bsdf->dgShading.nn;

		//update weights post untersection
		float tSquare = (position-ray.o).LengthSquared();
		float cosTheta = AbsDot(normal, vertex.in);
		vertex.dvcm *= tSquare / cosTheta;
		vertex.dvc /= cosTheta;

		//Check if hit light
		const AreaLight *l=vertex.isect.primitive->GetAreaLight();
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
		Spectrum L = scene->lights[lightIndex]->Sample_L(position, vertex.isect.rayEpsilon, ls, time, &lightDir, &lightPointPdfW, &vis);
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
				float revBsdfPdf = vertex.bsdf->Pdf(lightDir, -vertex.in);
				float revSurviveProb = min(1.f, bsdfFactor.y() * AbsDot(normal, vertex.in) / revBsdfPdf);
				revBsdfPdf *= revSurviveProb;
				wCam *= vertex.dvcm + revBsdfPdf * vertex.dvc;
				weight = 1.f / (wCam * 1.f);
			}
			else{
				float bsdfToLightPdfW = vertex.bsdf->Pdf(-vertex.in, lightDir);
				float cosAtLight = AbsDot(lightDir, lightNormal);
				float bsdfToLightPdfA = bsdfToLightPdfW * cosAtLight / lDistSquare;
				float lightPointPdfA = lightPointPdfW * AbsDot(lightDir, normal) / lDistSquare;
				
				float revBsdfPdf = vertex.bsdf->Pdf(lightDir, -vertex.in);
				float revSurviveProb = min(1.f, bsdfFactor.y() * AbsDot(normal, vertex.in) / revBsdfPdf);
				revBsdfPdf *= revSurviveProb;

				float wLight = bsdfToLightPdfA / lightPointPdfA;
				float wCam = (lEmitPointPdf/lightPointPdfA) * 
							lEmitDirPdf * (AbsDot(lightDir, normal) / lDistSquare) * 
							(vertex.dvcm + revBsdfPdf * vertex.dvc);

				weight = 1.f / (wLight + 1.f + wCam);
			}
			result += contrib * weight;
		}

		//TODO connect to light path
		for(int i = 0; i<end-1 && i<lightPath.size(); i++){
			result += ConnectVertices(vertex, lightPath[i], time, scene);
		}

		//sample next direction
		float bsdfPdf;
		BxDFType sampledType;
		Vector out;
		Spectrum f = vertex.bsdf->Sample_f(-vertex.in, &out, BSDFSample(rng), &bsdfPdf, BSDF_ALL, &sampledType);
		
		//exit if sample fails
		if(f.IsBlack() || bsdfPdf == 0.f)
			break;
		
		//calc attenuation
		float cosOut = AbsDot(out, normal);
		f *= cosOut / bsdfPdf;
		
		//russian roulete
		float surviveProb = min(f.y(), 1.f);
		if(rng.RandomFloat() > surviveProb)
			break;
		f /= surviveProb;
		
		//update weights pre intersection
		bsdfPdf *= surviveProb;
		float reversePdf = vertex.bsdf->Pdf(out, -vertex.in);
		float reverseSurviveProb = min(1.f, vertex.bsdf->f(out, -vertex.in).y()*AbsDot(vertex.in, normal) / reversePdf);
		reversePdf *= reverseSurviveProb;

		if(sampledType & BSDF_SPECULAR) {
			vertex.dvcm=0.f;
			vertex.dvc *= cosOut;
		}
		else {
			
			vertex.dvc = (cosOut / bsdfPdf) * (vertex.dvcm + reversePdf * vertex.dvc);
			vertex.dvcm = 1.f / bsdfPdf;
		}
		
		//update ray
		ray = RayDifferential(position, out, ray, vertex.isect.rayEpsilon);
		vertex.throughput *= f;
		vertex.in = out;
	}while(--end);

	camera->film->Splat(camSample, result/float(samplesPerPixel));

}

Spectrum BPTRenderer::ConnectVertices(const BPTVertex &camV, const BPTVertex &lightV, float time, const Scene *scene){
	Vector cam2lightDir = (lightV.bsdf->dgShading.p - camV.bsdf->dgShading.p);
	float tSquare = cam2lightDir.LengthSquared();
	cam2lightDir = Normalize(cam2lightDir);

	float cosAtCamV = AbsDot(cam2lightDir, camV.bsdf->dgShading.nn);
	float cosAtLightV = AbsDot(cam2lightDir, lightV.bsdf->dgShading.nn);

	Spectrum camBsdfFactor = camV.bsdf->f(-camV.in, cam2lightDir);
	Spectrum lBsdfFactor = lightV.bsdf->f(-lightV.in, -cam2lightDir);
	//calc result
	Spectrum result = camV.throughput * 
					  camBsdfFactor * 
					  lBsdfFactor * 
					  lightV.throughput *
					  cosAtCamV *
					  cosAtLightV /
					  tSquare;

	//test visibility
	VisibilityTester vis;
	vis.SetSegment(camV.bsdf->dgShading.p, camV.isect.rayEpsilon, lightV.bsdf->dgShading.p, lightV.isect.rayEpsilon, time);
	if(!result.IsBlack() && vis.Unoccluded(scene)){
		//calc weights
		float camPdf = camV.bsdf->Pdf(-camV.in, cam2lightDir);
		float camSurviveProb = min(1.f, camBsdfFactor.y() * AbsDot(cam2lightDir, camV.bsdf->dgShading.nn) / camPdf);
		camPdf *= camSurviveProb;

		float camRevPdf = camV.bsdf->Pdf(cam2lightDir, -camV.in);
		float camRevSurviveProb = min(1.f, camBsdfFactor.y() * AbsDot(camV.in, camV.bsdf->dgShading.nn) / camRevPdf);
		camRevPdf *= camRevSurviveProb;

		float lPdf = lightV.bsdf->Pdf(-lightV.in, -cam2lightDir);
		float lSurviveProb = min(1.f, lBsdfFactor.y() * AbsDot(cam2lightDir, lightV.bsdf->dgShading.nn) / lPdf);
		lPdf *= lSurviveProb;

		float lRevPdf = lightV.bsdf->Pdf(-cam2lightDir, -lightV.in);
		float lRevSurviveProb = min(1.f, lBsdfFactor.y() * AbsDot(-lightV.in, lightV.bsdf->dgShading.nn) / lPdf);
		lRevPdf *= lRevSurviveProb;

		float wCam = lPdf * (cosAtCamV / tSquare) * (camV.dvcm + camRevPdf * camV.dvc);
		float wLight = camPdf * (cosAtLightV / tSquare) * (lightV.dvcm + lRevPdf * lightV.dvc);
		
		float weight = 1.f / (wCam + wLight + 1.f);
		
		return result * weight;
	}
	else return 0.f;
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
	int samplesPerPixel = params.FindOneInt("samplesperpixel", 256);
	int maxDepth = params.FindOneInt("maxdepth", 10);
	return new BPTRenderer(samplesPerPixel, maxDepth, camera);
}