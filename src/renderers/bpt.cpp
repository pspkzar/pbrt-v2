#include "parallel.h"
#include "paramset.h"
#include "camera.h"
#include "film.h"
#include "intersection.h"
#include "bpt.h"


struct BPTVertex
{
	/* data */
	Intersection isect;
	float dvc;
	float dvcm;
};


class BPTTask : public Task {

};

void BPTRenderer::Render(const Scene *scene){

	for(int i=0; i<camera->film->xResolution; i++){
		for (int j = 0; j < camera->film->yResolution; j++){
			for(int s=0; s<samplesPerPixel; s++){
				vector<BPTVertex> lightPath;
				vector<BPTVertex> cameraPath;
				TraceLightPath(scene, lightPath, i,j);
				TraceCameraPath(scene, lightPath, cameraPath, i, j);
			}
		}
	}
}

void BPTRenderer::TraceLightPath(const Scene *scene, vector<BPTVertex> &lightPath, int px, int py){
	lightPath.clear();
	//sample light
	//select direction and init ray and weights
	int end=maxDepth;
	do{
		//intesect ray
			//exit if miss
		//store intersection
		//connect to camera
		//russian roulete
		//sample next direction
			//exit if sample fails
	}while(--end);
}

void BPTRenderer::TraceCameraPath(const Scene *scene, vector<BPTVertex> &lightPath, vector<BPTVertex> &cameraPath, int px, int py){
	cameraPath.clear();
	//sample camera direction and calc weights
	int end=maxDepth;
	do{
		//intersect ray
			//exit if miss
		//store intersection
		//connect to light
		//connect to light path
		//russian roulete
		//sample next direction
			//exti if sample fails
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