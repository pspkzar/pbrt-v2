#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_BPT_RENDERERS_H
#define PBRT_BPT_RENDERERS_H

#include "pbrt.h"
#include "renderer.h"

class BPTRenderer : public Renderer{
	BPTRenderer(int samplesPerPixel, int maxDepth, Camera *camera);
	~BPTRenderer();
	virtual void Render(const Scene *scene);
	Spectrum Li(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena,
        Intersection *isect = NULL, Spectrum *T = NULL) const;
    Spectrum Transmittance(const Scene *scene,
        const RayDifferential &ray, const Sample *sample,
        RNG &rng, MemoryArena &arena) const;

protected:
	int maxDepth, samplesPerPixel;
	Camera *camera;
};

BPTRenderer *CreateBPTRenderer(const ParamSet &params, Camera *camera);

#endif