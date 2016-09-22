#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_PATH_GEN_RENDERERS_H
#define PBRT_PATH_GEN_RENDERERS_H

#include "pbrt.h"
#include "renderer.h"
#include "rng.h"

class PathGenTask : public Task {
public:
	// PathGenTask Public Methods
    PathGenTask(const Scene *sc, Renderer *ren, Camera *c, int mD, int dL,
                        ProgressReporter &pr, float* pB, int tn, int tc)
      : maxDepth(mD), discretizationLevel(dL),reporter(pr), pathBuffer(pB)
    {
        scene = sc; renderer = ren; camera = c; taskNum = tn; taskCount = tc;
    }
    void Run();
private:
	const Scene *scene;
	Renderer *renderer;
	Camera *camera;
	ProgressReporter &reporter;
	int taskNum, taskCount;
	int maxDepth, discretizationLevel;
	float* pathBuffer;

	void TraceCameraPath(const Scene *scene, RNG &rng, MemoryArena &arena, float time, int px, int py);
};


class PathGenRenderer : public Renderer{
public:
	PathGenRenderer(int maxDepth, int discretizationLevel, Camera *camera){
		this->camera = camera;
		this->maxDepth = maxDepth;
		this->discretizationLevel = discretizationLevel;
	}

	~PathGenRenderer(){

	};

	void Render(const Scene *scene);

	Spectrum Li(const Scene *scene, const RayDifferential &ray,
        const Sample *sample, RNG &rng, MemoryArena &arena,
        Intersection *isect = NULL, Spectrum *T = NULL) const;

    Spectrum Transmittance(const Scene *scene,
        const RayDifferential &ray, const Sample *sample,
        RNG &rng, MemoryArena &arena) const;

	int maxDepth;
	int discretizationLevel;
	
	Camera *camera;

private:
};

PathGenRenderer *CreatePathGenRenderer(const ParamSet &params, Camera *camera);

#endif