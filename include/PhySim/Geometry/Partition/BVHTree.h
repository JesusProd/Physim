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
    class BVHTree
    {
    public:
        struct Object
        {
            virtual size_t NumElements() const = 0;
            virtual AlignedBox3d BBox(size_t index) const = 0;
        };

        enum struct PartitionHeuristics
        {
            LargestAxis
        };

        struct Options
        {
            Options();
            size_t MaxElementsPerLeaf;
            size_t MaxDepth;
            PartitionHeuristics Heuristics;
        };

    public:
        using QueryResults = vector<size_t>;

        using IntersectionResults = vector<pair<size_t, size_t>>;

    private:
        struct Node
        {
            bool IsLeafNode;
            ptrdiff_t ParentId;
            ptrdiff_t LeftChildId;
            ptrdiff_t RightChildId;
            vector<size_t> ElementIds;
            AlignedBox3d BBox;
        };

    public:
        BVHTree();
        BVHTree(Object* pObject, const Options& options);

        // Movable
        BVHTree(BVHTree&&) = default;
        BVHTree& operator=(BVHTree&&) = default;

        // Non-copyable
        BVHTree(const BVHTree&) = delete;
        BVHTree& operator=(const BVHTree&) = delete;

    public:
        bool IsValid() const;
        bool IsEmpty() const;
        
        void Clear();
        void Build(Object* pObject, const Options& options);
        void Rebuild();
        void Refit();

        void Query(const AlignedBox3d& bbox, QueryResults& entries, Real distance = 0.0) const;
        void Intersect(IntersectionResults& entries, Real distance = 0.0) const;
        void Intersect(const BVHTree& other, IntersectionResults& entries,
                       Real distance = 0.0) const;

    private:
        size_t Build(ptrdiff_t parentId, bool isLeftChild,
                     vector<size_t>::iterator first,
                     vector<size_t>::iterator last,
                     size_t depth);

        void Refit(ptrdiff_t nodeId);

        void Query(ptrdiff_t nodeId, const AlignedBox3d& boundingBox,
                   QueryResults& entries, Real sqrDistance) const;

        void Intersect(const BVHTree& other, ptrdiff_t nodeId, ptrdiff_t otherNodeId,
                       IntersectionResults& entries, Real sqrDistance) const;

        AlignedBox3d BBox(vector<size_t>::iterator first,
                          vector<size_t>::iterator last);

        vector<size_t>::iterator Partition(vector<size_t>::iterator first,
                                           vector<size_t>::iterator last);

    private:
        Object* m_pObject;
        vector<Node> m_vNodes;
        Options m_options;
        size_t m_numElements;
    };
}
