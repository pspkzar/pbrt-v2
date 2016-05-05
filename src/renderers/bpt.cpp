#include "parallel.h"
#include "paramset.h"
#include "camera.h"
#include "film.h"
#include "bpt.h"


struct BPTVertex
{
	/* data */
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