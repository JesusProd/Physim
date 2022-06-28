//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

namespace PhySim
{
    template <typename T>
    class ScalarField
    {
    public:
        using Scalar = T;
        using Vector3s = Vector<T, 3>;
        
    public:
        virtual Scalar Sample(const Vector3d& vPoint) const = 0;

        virtual Vector3s Gradient(const Vector3d& vPoint) const = 0;

        Scalar operator()(const Vector3d& vPoint) { return Sample(vPoint); }
    };
}
