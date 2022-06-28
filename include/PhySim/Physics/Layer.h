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

#include <array>

namespace PhySim
{
    /////////////////////////////////////////////////////////////////////////////////////////////

    using Layer = unsigned char;

    /////////////////////////////////////////////////////////////////////////////////////////////

    class LayerMask
    {
    public:
        LayerMask();
        LayerMask(size_t bitMask);

        LayerMask(const LayerMask& other) = default;
        LayerMask& operator=(const LayerMask& other) = default;

        LayerMask(LayerMask&& other) = default;
        LayerMask& operator=(LayerMask&& other) = default;

    public:
        void Clear();
        void Add(Layer layer);
        void Remove(Layer layer);
        void Invert();
        
    public:
        LayerMask Union(LayerMask other) const;
        LayerMask Intersection(LayerMask other) const;
        bool Overlaps(LayerMask other) const;
        bool Contains(Layer layer) const;
        bool Empty() const;

    public:
        friend LayerMask operator+(LayerMask mask, Layer layer);
        friend LayerMask operator+(Layer layer, LayerMask mask);
        friend LayerMask operator-(LayerMask mask, Layer layer);
        friend LayerMask operator-(Layer layer, LayerMask mask);
        LayerMask& operator+=(Layer layer);
        LayerMask& operator-=(Layer layer);
        friend LayerMask operator!(LayerMask mask);

        LayerMask operator|(LayerMask other) const;
        LayerMask operator&(LayerMask other) const;
        LayerMask& operator|=(const LayerMask& other);
        LayerMask& operator&=(const LayerMask& other);

        friend bool operator&&(LayerMask lhsMask, LayerMask rhsMask);
        friend bool operator&&(Layer lhsLayer, LayerMask rhsMask);
        friend bool operator&&(LayerMask lhsMask, Layer rhsLayer);

    protected:
        size_t m_BitMask;

    public:
        static LayerMask All;
        static LayerMask None;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////

    class LayerMaskMatrix
    {
    public:
        LayerMaskMatrix();
        LayerMaskMatrix(bool value);
        LayerMaskMatrix(const LayerMaskMatrix& other) = default;
        LayerMaskMatrix& operator=(const LayerMaskMatrix& other) = default;
        LayerMaskMatrix(LayerMaskMatrix&& other) = default;
        LayerMaskMatrix& operator=(LayerMaskMatrix&& other) = default;

    public:
        void Clear();
        void Add(Layer lhsLayer, Layer rhsLayer);
        void Remove(Layer lhsLayer, Layer rhsLayer);
        void Invert();

    public:
        bool operator()(Layer lhsLayer, Layer rhsLayer) const;

    private:
        size_t IndexOf(Layer lhsLayer, Layer rhsLayer) const;

    public:
        std::array<bool, 2080> m_Matrix;
    };

    /////////////////////////////////////////////////////////////////////////////////////////////
}