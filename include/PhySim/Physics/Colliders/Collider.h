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

#include <PhySim/Physics/Layer.h>

namespace PhySim
{
    class Simulable;

    enum ColliderType
    {
        ColliderType_Unknown,
        ColliderType_SphereCloud,
        ColliderType_DistanceField
    };

    class Collider
    {
    public:
        Collider();
        virtual ~Collider() = default;

    public:
        virtual string Name() const;
        virtual ColliderType Type() const;

    public:
        //virtual void Init(Simulable* pModel) = 0;
        Simulable* Model() const;
        Layer& CollisionLayer();

        bool IsSimulated() { return m_pModel != NULL; }

    public:
        virtual void Update() = 0;

    protected:
        Simulable* m_pModel;
        Layer m_CollisionLayer;
    };
}