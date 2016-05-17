#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_BPT_RENDERERS_H
#define PBRT_BPT_RENDERERS_H

#include "pbrt.h"
#include "renderer.h"
#include "rng.h"

struct BPTVertex;

class BPTRenderer : public Renderer{
public:
	BPTRenderer(int samplesPerPixel, int maxDepth, Camera *camera, bool lightTraceOnly){
		this->camera = camera;
		this->samplesPerPixel = samplesPerPixel;
		this->maxDepth = maxDepth;
		this->lightTraceOnly = lightTraceOnly;
	}

	~BPTRenderer(){

	};

	void Render(const Scene *scene);

	Spectrum Li(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena,
        Intersection *isect = NULL, Spectrum *T = NULL) const;

    Spectrum Transmittance(const Scene *scene,
        const RayDifferential &ray, const Sample *sample,
        RNG &rng, MemoryArena &arena) const;

	int maxDepth, samplesPerPixel;
	bool lightTraceOnly;
	RNG rng;
	Camera *camera;
	MemoryArena arena;

private:
	void TraceLightPath(const Scene *scene, vector<BPTVertex> &lightPath, float time, int px, int py);
	void TraceCameraPath(const Scene *scene, vector<BPTVertex> &lightPath, float time, int px, int py);
	Spectrum ConnectVertices(const BPTVertex &camV, const BPTVertex &lightV, float time, const Scene *scene);
};

BPTRenderer *CreateBPTRenderer(const ParamSet &params, Camera *camera);

#endif