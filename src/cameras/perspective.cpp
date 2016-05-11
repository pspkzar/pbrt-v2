
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// cameras/perspective.cpp*
#include "stdafx.h"
#include "cameras/perspective.h"
#include "paramset.h"
#include "sampler.h"
#include "montecarlo.h"
#include "light.h"

// PerspectiveCamera Method Definitions
PerspectiveCamera:: PerspectiveCamera(const AnimatedTransform &cam2world,
        const float screenWindow[4], float sopen, float sclose,
        float lensr, float focald, float fov, Film *f)
    : ProjectiveCamera(cam2world, Perspective(fov, 1e-2f, 1000.f),
                       screenWindow, sopen, sclose, lensr, focald, f) {
    // Compute differential changes in origin for perspective camera rays
    dxCamera = RasterToCamera(Point(1,0,0)) - RasterToCamera(Point(0,0,0));
    dyCamera = RasterToCamera(Point(0,1,0)) - RasterToCamera(Point(0,0,0));
}


float PerspectiveCamera::GenerateRay(const CameraSample &sample,
                                     Ray *ray) const {
    // Generate raster and camera samples
    Point Pras(sample.imageX, sample.imageY, 0);
    Point Pcamera;
    RasterToCamera(Pras, &Pcamera);
    *ray = Ray(Point(0,0,0), Normalize(Vector(Pcamera)), 0.f, INFINITY);
    // Modify ray for depth of field
    if (lensRadius > 0.) {
        // Sample point on lens
        float lensU, lensV;
        ConcentricSampleDisk(sample.lensU, sample.lensV, &lensU, &lensV);
        lensU *= lensRadius;
        lensV *= lensRadius;

        // Compute point on plane of focus
        float ft = focalDistance / ray->d.z;
        Point Pfocus = (*ray)(ft);

        // Update ray for effect of lens
        ray->o = Point(lensU, lensV, 0.f);
        ray->d = Normalize(Pfocus - ray->o);
    }
    ray->time = sample.time;
    CameraToWorld(*ray, ray);
    return 1.f;
}


float PerspectiveCamera::GenerateRayDifferential(const CameraSample &sample,
                                                 RayDifferential *ray) const {
    // Generate raster and camera samples
    Point Pras(sample.imageX, sample.imageY, 0);
    Point Pcamera;
    RasterToCamera(Pras, &Pcamera);
    Vector dir = Normalize(Vector(Pcamera.x, Pcamera.y, Pcamera.z));
    *ray = RayDifferential(Point(0,0,0), dir, 0.f, INFINITY);
    // Modify ray for depth of field
    if (lensRadius > 0.) {
        // Sample point on lens
        float lensU, lensV;
        ConcentricSampleDisk(sample.lensU, sample.lensV, &lensU, &lensV);
        lensU *= lensRadius;
        lensV *= lensRadius;

        // Compute point on plane of focus
        float ft = focalDistance / ray->d.z;
        Point Pfocus = (*ray)(ft);

        // Update ray for effect of lens
        ray->o = Point(lensU, lensV, 0.f);
        ray->d = Normalize(Pfocus - ray->o);
    }

    // Compute offset rays for _PerspectiveCamera_ ray differentials
    if (lensRadius > 0.) {
        // Compute _PerspectiveCamera_ ray differentials with defocus blur

        // Sample point on lens
        float lensU, lensV;
        ConcentricSampleDisk(sample.lensU, sample.lensV, &lensU, &lensV);
        lensU *= lensRadius;
        lensV *= lensRadius;

        Vector dx = Normalize(Vector(Pcamera + dxCamera));
        float ft = focalDistance / dx.z;
        Point pFocus = Point(0,0,0) + (ft * dx);
        ray->rxOrigin = Point(lensU, lensV, 0.f);
        ray->rxDirection = Normalize(pFocus - ray->rxOrigin);

        Vector dy = Normalize(Vector(Pcamera + dyCamera));
        ft = focalDistance / dy.z;
        pFocus = Point(0,0,0) + (ft * dy);
        ray->ryOrigin = Point(lensU, lensV, 0.f);
        ray->ryDirection = Normalize(pFocus - ray->ryOrigin);
    }
    else {
        ray->rxOrigin = ray->ryOrigin = ray->o;
        ray->rxDirection = Normalize(Vector(Pcamera) + dxCamera);
        ray->ryDirection = Normalize(Vector(Pcamera) + dyCamera);
    }

    ray->time = sample.time;
    CameraToWorld(*ray, ray);
    ray->hasDifferentials = true;
    return 1.f;
}

Spectrum PerspectiveCamera::Sample_Wi(const Point &p, CameraSample *sample, Vector *wi, float *pdf, VisibilityTester *vis){
    Point pLens(0,0,0);
    if(lensRadius > 0){
        float lensU, lensV;
        ConcentricSampleDisk(sample->lensU, sample->lensV, &lensU, &lensV);
        lensU *= lensRadius;
        lensV *= lensRadius;

        pLens.x = lensU;
        pLens.y = lensV;
    }
    Point pLensWorld = CameraToWorld(sample->time, pLens);

    vis->SetSegment(pLensWorld, 0, p, 1e-3, sample->time);

    *wi = pLensWorld - p;
    float dist = wi->Length();
    *wi /= dist;

    float lensArea = lensRadius != 0 ? (M_PI * lensRadius * lensRadius) : 1;
    float cosTheta = AbsDot(*wi, CameraToWorld(sample->time, Vector(0,0,1)));
    *pdf = (dist * dist) / (cosTheta * lensArea);

    return We(Ray(pLensWorld, -(*wi), 0, INFINITY, sample->time), &(sample->imageX), &(sample->imageY));
}

Spectrum PerspectiveCamera::We(const Ray &ray, float *rasterX, float *rasterY){
    
    Point pMin = RasterToCamera(Point(0, 0, 0));
    Point pMax = RasterToCamera(Point(film->xResolution, film->yResolution, 0));
    pMin /= pMin.z;
    pMax /= pMax.z;
    float A = fabsf((pMax.x - pMin.x) * (pMax.y - pMin.y));

    Transform c2w;
    CameraToWorld.Interpolate(ray.time, &c2w);
    float cosTheta = Dot(ray.d, c2w(Vector(0,0,1)));
    if(cosTheta <= 0.f) return 0.f;

    Point pFocus = ray((lensRadius > 0 ? focalDistance : 1) / cosTheta);
    Point pRaster = Inverse(RasterToCamera)(Inverse(c2w)(pFocus));

    if(rasterX) *rasterX = pRaster.x;
    if(rasterY) *rasterY = pRaster.y;

    if(pRaster.x < 0.f || pRaster.x > film->xResolution || pRaster.y < 0.f || pRaster.y > film->yResolution)
        return 0.f;

    float lensArea = lensRadius != 0 ? (M_PI * lensRadius * lensRadius) : 1;

    float cos2Theta = cosTheta * cosTheta;
    return Spectrum(1 / (A * lensArea * cos2Theta * cos2Theta));

}

void PerspectiveCamera::Pdf_We(const Ray &ray, float *pdfPos, float *pdfDir){
    Point pMin = RasterToCamera(Point(0, 0, 0));
    Point pMax = RasterToCamera(Point(film->xResolution, film->yResolution, 0));
    pMin /= pMin.z;
    pMax /= pMax.z;
    float A = fabsf((pMax.x - pMin.x) * (pMax.y - pMin.y));

    Transform c2w;
    CameraToWorld.Interpolate(ray.time, &c2w);
    float cosTheta = Dot(ray.d, c2w(Vector(0, 0, 1)));
    if (cosTheta <= 0) {
        *pdfPos = *pdfDir = 0;
        return;
    }

    // Map ray $(\p{}, \w{})$ onto the raster grid
    Point pFocus = ray((lensRadius > 0 ? focalDistance : 1) / cosTheta);
    Point pRaster = Inverse(RasterToCamera)(Inverse(c2w)(pFocus));

    // Return zero probability for out of bounds points
    if(pRaster.x < 0.f || pRaster.x > film->xResolution || pRaster.y < 0.f || pRaster.y > film->yResolution){
        *pdfPos = *pdfDir = 0;
        return;
    }

    // Compute lens area of perspective camera
    float lensArea = lensRadius != 0 ? (M_PI * lensRadius * lensRadius) : 1;
    *pdfPos = 1 / lensArea;
    *pdfDir = 1 / (A * cosTheta * cosTheta * cosTheta);
}

PerspectiveCamera *CreatePerspectiveCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film) {
    // Extract common camera parameters from _ParamSet_
    float shutteropen = params.FindOneFloat("shutteropen", 0.f);
    float shutterclose = params.FindOneFloat("shutterclose", 1.f);
    if (shutterclose < shutteropen) {
        Warning("Shutter close time [%f] < shutter open [%f].  Swapping them.",
                shutterclose, shutteropen);
        swap(shutterclose, shutteropen);
    }
    float lensradius = params.FindOneFloat("lensradius", 0.f);
    float focaldistance = params.FindOneFloat("focaldistance", 1e30f);
    float frame = params.FindOneFloat("frameaspectratio",
        float(film->xResolution)/float(film->yResolution));
    float screen[4];
    if (frame > 1.f) {
        screen[0] = -frame;
        screen[1] =  frame;
        screen[2] = -1.f;
        screen[3] =  1.f;
    }
    else {
        screen[0] = -1.f;
        screen[1] =  1.f;
        screen[2] = -1.f / frame;
        screen[3] =  1.f / frame;
    }
    int swi;
    const float *sw = params.FindFloat("screenwindow", &swi);
    if (sw && swi == 4)
        memcpy(screen, sw, 4*sizeof(float));
    float fov = params.FindOneFloat("fov", 90.);
    float halffov = params.FindOneFloat("halffov", -1.f);
    if (halffov > 0.f)
        // hack for structure synth, which exports half of the full fov
        fov = 2.f * halffov;
    return new PerspectiveCamera(cam2world, screen, shutteropen,
        shutterclose, lensradius, focaldistance, fov, film);
}
