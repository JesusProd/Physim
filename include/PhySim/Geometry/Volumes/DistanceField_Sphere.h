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
#include <PhySim/Geometry/Volumes/DistanceField.h>

namespace PhySim
{
    class DistanceField_Sphere : public DistanceField
    {
    public:
        DistanceField_Sphere();
        DistanceField_Sphere(const Vector3d& vOrigin, Real radius);

    public:
        void Setup(const Vector3d& vOrigin, Real radius);

    public:
        Vector3d Origin() const;
        Real Radius() const;

    public:
        Real Sample(const Vector3d& vPoint) const override;
        Vector3d Gradient(const Vector3d& vPoint) const override;

    public:
        Vector3d m_vOrigin;
        Real m_radius;
    };
}
