//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Colliders/Collider.h>
using namespace PhySim;

namespace PhySim
{
    Collider::Collider()
        : m_pModel(nullptr)
        , m_CollisionLayer(0ull)
    {
    }

    string Collider::Name() const
    {
        return "Collider";
    }

    ColliderType Collider::Type() const
    {
        return ColliderType_Unknown;
    }

    Simulable* Collider::Model() const
    {
        return m_pModel;
    }

    Layer& Collider::CollisionLayer()
    {
        return m_CollisionLayer;
    }
}