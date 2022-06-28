#include <PhySim/Geometry/Volumes/DistanceField_Sphere.h>
#include <PhySim/Geometry/Meshes/Mesh_Face.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <unordered_set>
using namespace PhySim;

DistanceField_Sphere::DistanceField_Sphere()
    : m_vOrigin(Vector3d::Zero())
    , m_radius(0.0)
{
}

DistanceField_Sphere::DistanceField_Sphere(const Vector3d& vOrigin, Real radius)
{
    Setup(vOrigin, radius);
}

void DistanceField_Sphere::Setup(const Vector3d& vOrigin, Real radius)
{
    m_vOrigin = vOrigin;
    m_radius = radius;
}

Vector3d DistanceField_Sphere::Origin() const
{
    return m_vOrigin;
}

Real DistanceField_Sphere::Radius() const
{
    return m_radius;
}

Real DistanceField_Sphere::Sample(const Vector3d& vPoint) const
{
    return (vPoint - m_vOrigin).norm() - m_radius;
}

Vector3d DistanceField_Sphere::Gradient(const Vector3d& vPoint) const
{
    return (vPoint - m_vOrigin).normalized();
}
