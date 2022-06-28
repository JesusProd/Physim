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
#include <PhySim/Geometry/Volumes/ScalarField_Grid.h>
#include <PhySim/Geometry/Volumes/DistanceField.h>

namespace PhySim
{
    class DistanceField_Mesh : public DistanceField
    {
    public:
        void Setup(const MatrixXd& mPoints, const MatrixXi& mTriangles, int LargestAxisSize, int Padding=1);

    public:
        Vector3i Shape() const;
        Vector3d Origin() const;
        Vector3d Delta() const;

    public:
        const MatrixXd& Points() const;
        const MatrixXi& Faces() const;
        const Tensor3d& DistanceData() const;
        const Tensor3i& FaceData() const;

    public:
        Real Sample(const Vector3d& vPoint) const override;
        Vector3d Gradient(const Vector3d& vPoint) const override;
        Vector3i IndexOf(const Vector3d& vPoint, bool clamp = false);
        Vector3d PointOf(const Vector3i& vIndex);

    private:
        bool Validate();
        void Rasterize();
        void Sweep(const Vector3i& vDeltaIndex);
        void Compare(const Vector3d& vPoint, const Vector3i& vIndex, const Vector3i& vDeltaIndex);
        void DetermineSigns();

    public:
        MatrixXd m_mPoints;
        MatrixXi m_mFaces;
        ScalarField_Grid<Real> m_sfDistances;
        ScalarField_Grid<int> m_sfFaces;
    };
}
