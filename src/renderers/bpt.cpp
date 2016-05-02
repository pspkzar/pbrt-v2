#include "parallel.h"
#include "paramset.h"
#include "bpt.h"

struct PathVertex
{
	/* data */
};


class BPTTask : public Task {

};

void BPTRenderer::Render(const Scene *scene){

	
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