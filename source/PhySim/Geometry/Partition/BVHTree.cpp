#include <PhySim/Geometry/Partition/BVHTree.h>
#include <algorithm>
#include <numeric>
using namespace PhySim;

/////////////////////////////////////////////////////////////////////////////////////////////////

BVHTree::Options::Options()
    : MaxElementsPerLeaf(1)
    , MaxDepth(std::numeric_limits<size_t>::max())
    , Heuristics(PartitionHeuristics::LargestAxis)
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////

BVHTree::BVHTree()
    : m_pObject(nullptr)
    , m_numElements(0)
{
}

BVHTree::BVHTree(Object* pObject, const Options& options)
    : m_pObject(pObject)
    , m_options(options)
{
    Rebuild();
}


bool BVHTree::IsValid() const
{
    return m_pObject != nullptr && IsEmpty() == false;
}

bool BVHTree::IsEmpty() const
{
    return m_vNodes.size() == 0;
}

void BVHTree::Clear()
{
    m_vNodes.clear();
}

void BVHTree::Build(Object* pObject, const Options& options)
{
    m_pObject = pObject;
    m_options = options;
    Rebuild();
}

void BVHTree::Rebuild()
{
    assert(m_pObject != nullptr && "The given object is null.");

    // Store object's current no. of elements.
    m_numElements = m_pObject->NumElements();

    // Initialize element partition.
    vector<size_t> elements;
    elements.resize(m_pObject->NumElements());
    iota(elements.begin(), elements.end(), 0);

    // Create tree.
    m_vNodes.clear();
    Build(-1, false, elements.begin(), elements.end(), 0);
}

void BVHTree::Refit()
{
    assert(m_pObject != nullptr && "The given object is null.");
    assert(m_pObject->NumElements() == m_numElements && "The number of elements in the object mismatchs.");
    Refit(0);
}

void BVHTree::Query(const AlignedBox3d& boundingBox, QueryResults& entries, Real distance) const
{
    if (m_vNodes.size() == 0)
        return;

    Query(0, boundingBox, entries, distance * distance);
}

void BVHTree::Intersect(IntersectionResults& entries, Real distance) const
{
    if (m_vNodes.size() <= 1)
        return;

    const Node& root = m_vNodes[0];
    Intersect(*this, root.LeftChildId, root.RightChildId, entries, distance * distance);
}

void BVHTree::Intersect(const BVHTree& other, IntersectionResults& entries,
                                        Real distance) const
{
    if (IsEmpty() == true || other.IsEmpty() == true)
        return;

    Intersect(other, 0, 0, entries, distance * distance);
}

size_t BVHTree::Build(ptrdiff_t parentId, bool isLeftChild,
                      std::vector<size_t>::iterator first, 
                      std::vector<size_t>::iterator last,
                      size_t depth)
{
    size_t nodeId = m_vNodes.size();

    // Determine number of elements in node. If only a single element remains,
    // or the maximum recursion depth has been reached, build a leaf node and
    // stop recursion.
    size_t numElements = std::distance(first, last);

    if (numElements <= m_options.MaxElementsPerLeaf || depth >= m_options.MaxDepth)
    {
        Node leaf;
        leaf.IsLeafNode = true;
        leaf.ParentId = parentId;
        leaf.LeftChildId = -1;
        leaf.RightChildId = -1;
        leaf.BBox = BBox(first, last);
        leaf.ElementIds.reserve(numElements);
        leaf.ElementIds.insert(leaf.ElementIds.end(), first, last);
        m_vNodes.push_back(leaf);
    }

    // Otherwise, determine split and generate branch nodes.
    else
    {
        
        // Partition elements according to the selected heuristic.
        std::vector<size_t>::iterator mid = Partition(first, last);

        // Generate node and recursively build children.
        Node node;
        node.IsLeafNode = false;
        node.ParentId = parentId;
        node.LeftChildId = -1;
        node.RightChildId = -1;
        node.BBox = BBox(first, last);
        m_vNodes.push_back(node);

        Build(nodeId, true, first, mid, depth + 1);
        Build(nodeId, false, mid, last, depth + 1);
    }

    // Finally, assign node id to the parent.
    if (parentId != -1)
    {
        if (isLeftChild == true) m_vNodes[parentId].LeftChildId = nodeId;
        else                     m_vNodes[parentId].RightChildId = nodeId;
    }

    // Done!
    return nodeId;
}

void BVHTree::Refit(ptrdiff_t nodeId)
{
    Node& node = m_vNodes[nodeId];

    if (node.IsLeafNode == true)
    {
        node.BBox.setEmpty();

        for (auto elementId : node.ElementIds)
            node.BBox.extend(m_pObject->BBox(elementId));
    }
    else
    {
        Refit(node.LeftChildId);
        Refit(node.RightChildId);

        Node& leftChild = m_vNodes[node.LeftChildId];
        Node& rightChild = m_vNodes[node.RightChildId];

        node.BBox.setNull();
        node.BBox.extend(leftChild.BBox);
        node.BBox.extend(rightChild.BBox);
    }
}

void BVHTree::Query(ptrdiff_t nodeId, const AlignedBox3d& boundingBox,
                    QueryResults& entries, Real sqrDistance) const
{
    const Node& node = m_vNodes[nodeId];

    if (node.BBox.squaredExteriorDistance(boundingBox) <= sqrDistance)
    {
        if (node.IsLeafNode == true)
        {
            //entries.reserve(entries.size() + node.ElementIds.size());
            entries.insert(entries.end(), node.ElementIds.begin(), node.ElementIds.end());
        }
        else
        {
            Query(node.LeftChildId, boundingBox, entries, sqrDistance);
            Query(node.RightChildId, boundingBox, entries, sqrDistance);
        }
    }
}

void BVHTree::Intersect(const BVHTree& other, ptrdiff_t nodeId, ptrdiff_t otherNodeId,
                        IntersectionResults& entries, Real sqrDistance) const
{
    const Node& node = m_vNodes[nodeId];
    const Node& otherNode = other.m_vNodes[otherNodeId];

    if (node.BBox.squaredExteriorDistance(otherNode.BBox) <= sqrDistance)
    {
        if (node.IsLeafNode == true && otherNode.IsLeafNode == true)
        {
            size_t totalNumElements = node.ElementIds.size() * otherNode.ElementIds.size();
            //entries.reserve(entries.size() + totalNumElements);

            for (size_t elementId : node.ElementIds)
            for (size_t otherElementId : otherNode.ElementIds)
            {
                entries.emplace_back(elementId, otherElementId);
            }
        }
        else if (node.IsLeafNode == true)
        {
            Intersect(other, nodeId, otherNode.LeftChildId, entries, sqrDistance);
            Intersect(other, nodeId, otherNode.RightChildId, entries, sqrDistance);
        }
        else if (otherNode.IsLeafNode == true)
        {
            Intersect(other, node.LeftChildId, otherNodeId, entries, sqrDistance);
            Intersect(other, node.RightChildId, otherNodeId, entries, sqrDistance);
        }
        else
        {
            Intersect(other, node.LeftChildId, otherNode.LeftChildId, entries, sqrDistance);
            Intersect(other, node.LeftChildId, otherNode.RightChildId, entries, sqrDistance);
            Intersect(other, node.RightChildId, otherNode.LeftChildId, entries, sqrDistance);
            Intersect(other, node.RightChildId, otherNode.RightChildId, entries, sqrDistance);
        }
    }
}

AlignedBox3d BVHTree::BBox(vector<size_t>::iterator first, 
                           vector<size_t>::iterator last)
{
    AlignedBox3d bbox;
    bbox.setEmpty();

    for (auto it = first; it != last; ++it)
        bbox.extend(m_pObject->BBox(*it));

    return bbox;
}

vector<size_t>::iterator BVHTree::Partition(vector<size_t>::iterator first,
                                            vector<size_t>::iterator last)
{
    vector<size_t>::iterator mid = first;

    // Perform partition according to the desired heuristics.
    // TODO: Implement other heuristics if necessary.
    switch (m_options.Heuristics)
    {
        case BVHTree::PartitionHeuristics::LargestAxis:
        {
            // Compute aggregate bounding box.
            AlignedBox3d bbox = BBox(first, last);
            Vector3d size = bbox.sizes();
            //Index dim = Utils::argMax(size);
			Index dim;
            Real center = bbox.center()[dim];

            // Partition elements along the largest axis's mid-point.
            mid = partition(first, last, [this, dim, center](size_t elementId) {
                return m_pObject->BBox(elementId).center()[dim] < center;
            });

            break;
        }
    }

    // If the resulting split is invalid, the element set might be degenerated.
    // In such cases, opt to split the elements into two halves.

    const size_t leftNumElements = distance(first, mid);
    const size_t rightNumElements = distance(mid, last);

    if (leftNumElements == 0 || rightNumElements == 0)
        mid = next(first, (leftNumElements + rightNumElements) / 2);

    // Done!
    return mid;
}
