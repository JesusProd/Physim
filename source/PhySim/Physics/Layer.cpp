//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Layer.h>
#include <algorithm>
using namespace PhySim;

namespace PhySim
{
    /////////////////////////////////////////////////////////////////////////////////////////////

    LayerMask::LayerMask()
        : m_BitMask(~0ull)
    {
    }

    LayerMask::LayerMask(size_t bitMask)
        : m_BitMask(bitMask)
    {
    }

    void LayerMask::Clear()
    {
        m_BitMask = 0ull;
    }

    void LayerMask::Add(Layer layer)
    {
        assert(layer >= 0ull && layer <= 63ull && "Specified layer is outside of the 0..63 range");
        m_BitMask |= 1ull << layer;
    }

    void LayerMask::Remove(Layer layer)
    {
        assert(layer >= 0ull && layer <= 63ull && "Specified layer is outside of the 0..63 range");
        m_BitMask &= ~(1ull << layer);
    }

    void LayerMask::Invert()
    {
        m_BitMask = ~m_BitMask;
    }

    LayerMask LayerMask::Union(LayerMask other) const
    {
        return LayerMask(m_BitMask | other.m_BitMask);
    }

    LayerMask LayerMask::Intersection(LayerMask other) const
    {
        return LayerMask(m_BitMask & other.m_BitMask);
    }

    bool LayerMask::Overlaps(LayerMask other) const
    {
        return (m_BitMask & other.m_BitMask) != 0ull;
    }

    bool LayerMask::Contains(Layer layer) const
    {
        assert(layer >= 0ull && layer <= 63ull && "Specified layer is outside of the 0..63 range");
        return m_BitMask & (1ull << layer);
    }

    bool LayerMask::Empty() const
    {
        return !m_BitMask;
    }

    LayerMask operator+(LayerMask mask, Layer layer)
    {
        mask.Add(layer);
        return mask;
    }

    LayerMask operator+(Layer layer, LayerMask mask)
    {
        mask.Add(layer);
        return mask;
    }

    LayerMask operator-(LayerMask mask, Layer layer)
    {
        mask.Remove(layer);
        return mask;
    }

    LayerMask operator-(Layer layer, LayerMask mask)
    {
        mask.Remove(layer);
        return mask;
    }

    LayerMask& LayerMask::operator+=(Layer layer)
    {
        Add(layer);
        return *this;
    }

    LayerMask& LayerMask::operator-=(Layer layer)
    {
        Remove(layer);
        return *this;
    }

    LayerMask operator!(LayerMask mask)
    {
        return LayerMask(!mask.m_BitMask);
    }

    LayerMask LayerMask::operator|(LayerMask other) const
    {
        return Union(other);
    }

    LayerMask LayerMask::operator&(LayerMask other) const
    {
        return Intersection(other);
    }

    LayerMask& LayerMask::operator|=(const LayerMask& other)
    {
        *this = Union(other);
        return *this;
    }

    LayerMask& LayerMask::operator&=(const LayerMask& other)
    {
        *this = Intersection(other);
        return *this;
    }

    bool operator&&(LayerMask lhsMask, LayerMask rhsMask)
    {
        return lhsMask.Overlaps(rhsMask);
    }   

    bool operator&&(Layer lhsLayer, LayerMask rhsMask)
    {
        return rhsMask.Contains(lhsLayer);
    }

    bool operator&&(LayerMask lhsMask, Layer rhsLayer)
    {
        return lhsMask.Contains(rhsLayer);
    }

    LayerMask LayerMask::All = ~0ull;
    LayerMask LayerMask::None = 0ull;

    /////////////////////////////////////////////////////////////////////////////////////////////

    LayerMaskMatrix::LayerMaskMatrix()
        : LayerMaskMatrix(true)
    {
    }

    LayerMaskMatrix::LayerMaskMatrix(bool value)
    {
        fill(m_Matrix.begin(), m_Matrix.end(), true);
    }

    void LayerMaskMatrix::Clear()
    {
        fill(m_Matrix.begin(), m_Matrix.end(), false);
    }

    void LayerMaskMatrix::Add(Layer lhsLayer, Layer rhsLayer)
    {
        m_Matrix[IndexOf(lhsLayer, rhsLayer)] = true;
    }

    void LayerMaskMatrix::Remove(Layer lhsLayer, Layer rhsLayer)
    {
        m_Matrix[IndexOf(lhsLayer, rhsLayer)] = false;
    }

    void LayerMaskMatrix::Invert()
    {
        for_each(m_Matrix.begin(), m_Matrix.end(), negate<bool>{});
    }

    bool LayerMaskMatrix::operator()(Layer lhsLayer, Layer rhsLayer) const
    {
        return m_Matrix[IndexOf(lhsLayer, rhsLayer)];
    }

    size_t LayerMaskMatrix::IndexOf(Layer lhsLayer, Layer rhsLayer) const
    {
        assert(lhsLayer >= 0ull && lhsLayer <= 63ull && "Specified layer is outside of the 0..63 range");
        assert(rhsLayer >= 0ull && rhsLayer <= 63ull && "Specified layer is outside of the 0..63 range");

        return lhsLayer <= rhsLayer
            ? (128 - lhsLayer - 1) * lhsLayer / 2 + rhsLayer
            : (128 - rhsLayer - 1) * rhsLayer / 2 + lhsLayer;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////
}