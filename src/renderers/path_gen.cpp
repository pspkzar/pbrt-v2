#include "stdafx.h"
#include "parallel.h"
#include "paramset.h"
#include "camera.h"
#include "film.h"
#include "intersection.h"
#include "scene.h"
#include "memory.h"
#include "path_gen.h"
#include "progressreporter.h"
#include <iostream>
#include <fstream>
#include <sstream>

void PathGenRenderer::Render(const Scene *scene){

	vector<Task *> renderTasks;

	int x0, x1, y0, y1;
	camera->film->GetPixelExtent(&x0, &x1, &y0, &y1);
	long long npoints = discretizationLevel*discretizationLevel;
	npoints = pow(npoints, maxDepth-1);
	npoints *= (x1-x0)*(y1-y0);
	std::cout << npoints << std::endl;
	
	float *pathBuffer = new float[npoints];
	
	int nTasks = 32 * NumSystemCores();
	nTasks = RoundUpPow2(nTasks);

	ProgressReporter reporter(nTasks, "Rendering");
	for (int i = 0; i < nTasks; ++i)
		renderTasks.push_back(new PathGenTask(scene, this, camera, maxDepth, discretizationLevel, reporter, pathBuffer, nTasks-1-i, nTasks));

	EnqueueTasks(renderTasks);
	WaitForAllTasks();
	for (uint32_t i = 0; i < renderTasks.size(); ++i)
		delete renderTasks[i];
	reporter.Done();
	camera->film->WriteImage();

	std::ofstream outFile;
	std::stringstream sstr;
	sstr << "pathGen" << maxDepth << ".txt";
	std::string filename = sstr.str();

	outFile.open(filename);
	for(int i=0; i<npoints; i++){
		outFile << pathBuffer[i] << "\n";
	}
	outFile.close();
	delete[] pathBuffer;
}


void PathGenTask::Run () {
	RNG rng(taskNum);
	MemoryArena arena;

	
	// Compute number of BPTTasks to create for rendering
	int x0, x1, y0, y1;
	camera->film->GetPixelExtent(&x0, &x1, &y0, &y1);
	float nPixPerTask = float((x1-x0)*(y1-y0)) / float(taskCount);
	int TaskFirstPix = int(nPixPerTask * float(taskNum));
	int TaskLastPix = int(nPixPerTask * float(taskNum+1));
	TaskLastPix = (TaskLastPix > ((x1-x0)*(y1-y0)) ? ((x1-x0)*(y1-y0)) : TaskLastPix);
	for(int i=TaskFirstPix; i<TaskLastPix; i++){
		int py = i / (x1-x0);
		int px = i % (x1-x0);
		py += y0;
		px += x0;
		
		float time = 0.f;
		TraceCameraPath(scene, rng, arena, time, px,py);
		arena.FreeAll();
		
		
	}
	reporter.Update();
}

void PathGenTask::TraceCameraPath(const Scene *scene, RNG &rng, MemoryArena &arena, float time, int px, int py){
	
	int x0, x1, y0, y1;
	camera->film->GetPixelExtent(&x0, &x1, &y0, &y1);
	long long npoints = discretizationLevel*discretizationLevel;
	npoints = pow(npoints, maxDepth-1);

	BSDFSample sample[maxDepth-1];

	for(long long i=0; i<npoints; i++){
		
		long long left=i;
		for(int d=0; d<maxDepth-1; d++){
			sample[d].uDir[0] = (float(left%discretizationLevel)+rng.RandomFloat())/float(discretizationLevel);
			left /= discretizationLevel;
			sample[d].uDir[1] = (float(left%discretizationLevel)+rng.RandomFloat())/float(discretizationLevel);
			left /= discretizationLevel;
			sample[d].uComponent = 0.f;
			//sample[d].uComponent = (float(left%discretizationLevel)+rng.RandomFloat())/float(discretizationLevel);
			//left /= discretizationLevel;
		}

		//sample = BSDFSample(rng);

		RayDifferential ray;
		Spectrum result(0.f);

		//sample camera direction
		CameraSample camSample;
		camSample.imageX = float(px) + rng.RandomFloat();
		camSample.imageY = float(py) + rng.RandomFloat();
		camSample.lensU = 0;//rng.RandomFloat();
		camSample.lensV = 0;//rng.RandomFloat();
		camSample.time = time;
		camera->GenerateRayDifferential(camSample, &ray);
		Spectrum throughput = Spectrum(1.f);
		Vector in = ray.d;

		int end=maxDepth;
		do{
						//intersect ray and exit if miss
			Intersection isect;
			if(!scene->Intersect(ray, &isect))
				break;
			BSDF *bsdf = isect.GetBSDF(ray, arena);
			Point position = bsdf->dgShading.p;
			Normal normal = bsdf->dgShading.nn;

						//Check if hit light
			const AreaLight *l=isect.primitive->GetAreaLight();
			if(l){
				Spectrum Lemmit = isect.Le(-in);
				if(!Lemmit.IsBlack()){
					result += Lemmit * throughput;
				}
			}



			//sample next direction
			if(end>1){
				float bsdfPdf;
				BxDFType sampledType;
				Vector out;
				Spectrum f = bsdf->Sample_f(-in, &out, sample[maxDepth-end], &bsdfPdf, BSDF_ALL, &sampledType);

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

						//update ray
				ray = RayDifferential(position, out, ray, isect.rayEpsilon);
				throughput *= f;
				in = out;
			}
		}while(--end);

		camera->film->Splat(camSample, result/float(discretizationLevel*discretizationLevel*discretizationLevel));
		//if(!result.IsBlack())
		//	std::cout << result.y() << std::endl;
		long long pathIndex = (py*(x1-x0)+px)*npoints+i;
		pathBuffer[pathIndex]=result.y()/float(discretizationLevel*discretizationLevel*discretizationLevel);
	}
}




Spectrum PathGenRenderer::Li(const Scene *scene, const RayDifferential &ray,
	const Sample *sample, RNG &rng, MemoryArena &arena,
	Intersection *isect, Spectrum *T) const{
	return 0.f;
}

Spectrum PathGenRenderer::Transmittance(const Scene *scene,
	const RayDifferential &ray, const Sample *sample,
	RNG &rng, MemoryArena &arena) const{
	return 1.f;
}

PathGenRenderer *CreatePathGenRenderer(const ParamSet &params, Camera *camera){
	int maxDepth = params.FindOneInt("maxdepth", 2);
	int discretizationLevel = params.FindOneInt("discretizationlevel", 32);
	return new PathGenRenderer(maxDepth, discretizationLevel, camera);
}