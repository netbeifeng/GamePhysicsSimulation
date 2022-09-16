#pragma once
#include "Simulator.h"

class Poly6Kernel {
public:
    void setRadius(const float radius) {
        _radius = radius;
        _radiusSqr = _radius * _radius;
        _k = 315.0f / (64.0f * M_PI * pow(_radius, 9.0f));
        _W0 = W(0.0f);
    }

    float W(const float r) const {
        const float r2 = r * r;
        return r2 <= _radiusSqr ? pow(_radiusSqr - r2, 3.0f) * _k : 0.0f;
    }

    float W(const Vec3& r) const {
        const auto r2 = r.x * r.x + r.y * r.y + r.z * r.z;
        return r2 <= _radiusSqr ? pow(_radiusSqr - r2, 3.0f) * _k : 0.0f;
    }

    float W0() const { return _W0; }

private:
    float _radius;
    float _radiusSqr;
    float _k;
    float _W0;
};

class SpikyKernel {
public:
    void setRadius(const float radius) {
        _radius = radius;
        _radiusSqr = _radius * _radius;
        _l = -45.0f / (M_PI * pow(_radius, 6.0f));
    }

    Vec3 gradW(const Vec3& r) const {
        Vec3 res{ 0.0f };
        const float r2 = r.x * r.x + r.y * r.y + r.z * r.z;
        if (r2 <= _radiusSqr && r2 > 1.0e-12f) {
            const float rl = sqrt(r2);
            const float hr = _radius - rl;
            const float hr2 = hr * hr;
            res = _l * hr2 * (r / rl);
        }

        return res;
    }

protected:
    float _radius;
    float _radiusSqr;
    float _l;
};

class SPHKernels {
public:
    explicit SPHKernels(float kernelRadius) {
        _poly6.setRadius(kernelRadius);
        _spiky.setRadius(kernelRadius);
    }

    float W0() const { return _poly6.W0(); }
    float W(const Vec3& r) const { return _poly6.W(r); }
    Vec3 gradW(const Vec3& r) const { return _spiky.gradW(r); }

private:
    Poly6Kernel _poly6;
    SpikyKernel _spiky;
};