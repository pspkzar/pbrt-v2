#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_BPT_RENDERERS_H
#define PBRT_BPT_RENDERERS_H

#include "pbrt.h"
#include "renderer.h"
#include "rng.h"

struct BPTVertex;


class BPTTask : public Task {
public:
	// BPTTask Public Methods
    BPTTask(const Scene *sc, Renderer *ren, Camera *c, int mD, int samplesPixel, bool lt,
                        ProgressReporter &pr, int tn, int tc)
      : maxDepth(mD), samplesPerPixel(samplesPixel), reporter(pr), lightTraceOnly(lt)
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
	int maxDepth, samplesPerPixel;
	bool lightTraceOnly;


	void TraceLightPath(const Scene *scene, RNG &rng, MemoryArena &arena, vector<BPTVertex> &lightPath, float time, int px, int py);
	void TraceCameraPath(const Scene *scene, RNG &rng, MemoryArena &arena, vector<BPTVertex> &lightPath, float time, int px, int py);
	Spectrum ConnectVertices(const BPTVertex &camV, const BPTVertex &lightV, float time, const Scene *scene);

};


class BPTRenderer : public Renderer{
public:
	BPTRenderer(int samplesPerPixel, int maxDepth, 	bool lightTraceOnly, Camera *camera){
		this->camera=camera;
		this->samplesPerPixel=samplesPerPixel;
		this->maxDepth=maxDepth;
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
	Camera *camera;

private:
};

BPTRenderer *CreateBPTRenderer(const ParamSet &params, Camera *camera);

#endif