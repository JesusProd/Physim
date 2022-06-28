//#include <PhySim/Geometry/Partition/KDTree.h>
//using namespace PhySim;
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//
//KDTree::Options::Options()
//    : MaxDepth(20)
//{
//}
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//
//KDTree::DataSet::DataSet()
//    : pObject(nullptr)
//{
//}
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//
//KDTree::KDTree(Object* pObject, const Options& options)
//{
//    m_dataset.pObject = pObject;
//    m_options = options;
//    Rebuild();
//}
//
//bool KDTree::IsValid() const
//{
//    return m_dataset.pObject != nullptr && IsEmpty() == false;
//}
//
//bool KDTree::IsEmpty() const
//{
//    return m_pAdaptor != nullptr;
//}
//
//void KDTree::Clear()
//{
//    m_pAdaptor.release();
//}
//
//void KDTree::Build(Object* pObject, const Options& options)
//{
//    m_dataset.pObject = pObject;
//    m_options = options;
//    Rebuild();
//}
//
//void KDTree::Rebuild()
//{
//    assert(m_dataset.pObject != nullptr && "The given object is null.");
//
//    // Build new Nanoflann KD-Tree adaptor.
//    m_pAdaptor = NewU<Adaptor>(3, m_dataset, nanoflann::KDTreeSingleIndexAdaptorParams(m_options.MaxDepth));
//    m_pAdaptor->buildIndex();
//}
//
//void KDTree::FindKNN(const Vector3d& point, QueryResults& entries, size_t maxNeighbors) const
//{
//    assert(m_pAdaptor != nullptr && "The KD-Tree has not been built!");
//
//    // Find the K-NN storing the results into an intermediate structure.
//    std::vector<double> distances(maxNeighbors);
//    std::vector<size_t> indices(maxNeighbors);
//
//    nanoflann::KNNResultSet<double> results(maxNeighbors);
//    results.init(indices.data(), distances.data());
//    m_pAdaptor->findNeighbors(results, point.data(), {});
//
//    // Output entries.
//    entries.clear();
//    entries.reserve(results.size());
//
//    for (size_t i = 0; i < results.size(); ++i)
//        entries.emplace_back(std::make_pair(indices[i], distances[i]));
//}
//
//void KDTree::SearchRadius(const Vector3d& point, QueryResults& entries, Real radius) const
//{
//    assert(m_pAdaptor != nullptr && "The KD-Tree has not been built!");
//
//    // Find all points within the given radius.
//    entries.clear();
//    m_pAdaptor->radiusSearch(point.data(), radius * radius, entries, {});
//}
//
//size_t KDTree::DataSet::kdtree_get_point_count() const
//{
//    return pObject->NumPoints();
//}
//
//double KDTree::DataSet::kdtree_distance(const double* p1, const size_t p2, size_t) const
//{
//    const Map<const Vector3d> P1(p1);
//    const Vector3d P2 = pObject->Point(p2);
//    return (P1 - P2).norm();
//}
//
//double KDTree::DataSet::kdtree_get_pt(const size_t idx, int dim) const
//{
//    return pObject->Point(idx)[dim];
//}
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//
//KDPointCloudObject::KDPointCloudObject(const MatrixXd& mPoints)
//    : m_mPoints(mPoints)
//{
//    assert(m_mPoints.rows() != 0 && "Empty point cloud matrix provided.");
//    assert(m_mPoints.cols() == 3 && "Only 3D point clouds are supported.");
//}
//
//size_t KDPointCloudObject::NumPoints() const
//{
//    return m_mPoints.rows();
//}
//
//Vector3d KDPointCloudObject::Point(size_t index) const
//{
//    return m_mPoints.row(index);
//}
