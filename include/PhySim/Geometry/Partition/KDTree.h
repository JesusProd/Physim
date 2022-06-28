////==========================================================
////
////	PhySim library. Generic library for physical simulation.
////
////	Authors:
////			Jesus Perez Rodriguez, jesusprod @ GitHub
////
////==========================================================
//
//#pragma once
//#include <PhySim/CommonIncludes.h>
//#include <nanoflann.hpp>
//
//namespace PhySim
//{
//    class KDTree
//    {
//    public:
//        struct Object
//        {
//            virtual size_t NumPoints() const = 0;
//            virtual Vector3d Point(size_t index) const = 0;
//        };
//
//        struct Options
//        {
//            Options();
//            size_t MaxDepth;
//        };
//
//        using QueryResults = std::vector<std::pair<size_t, double>>;
//
//    private:
//        struct DataSet
//        {
//            DataSet();
//            size_t kdtree_get_point_count() const;
//            double kdtree_distance(const double* p1, const size_t p2, size_t) const;
//            template <class BBox> inline bool kdtree_get_bbox(BBox&) const { return false; }
//            double kdtree_get_pt(const size_t idx, int dim) const;
//            Object* pObject;
//        };
//
//        using Adaptor = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, DataSet>, DataSet, 3>;
//
//    public:
//        KDTree() = default;
//        KDTree(Object* pObject, const Options& options);
//
//        // Movable
//        KDTree(KDTree&&) = default;
//        KDTree& operator=(KDTree&&) = default;
//
//        // Non-copyable
//        KDTree(const KDTree&) = delete;
//        KDTree& operator=(const KDTree&) = delete;
//
//    public:
//        bool IsValid() const;
//        bool IsEmpty() const;
//        
//        void Clear();
//        void Build(Object* pObject, const Options& options);
//        void Rebuild();
//
//        void FindKNN(const Vector3d& point, QueryResults& entries, size_t maxNeighbors) const;
//        void SearchRadius(const Vector3d& point, QueryResults& entries, Real radius) const;
//
//    private:
//        Options m_options;
//        DataSet m_dataset;
//        PtrU<Adaptor> m_pAdaptor;
//    };
//
//    class KDPointCloudObject : public KDTree::Object
//    {
//    public:
//        KDPointCloudObject(const MatrixXd& mPoints);
//
//    public:
//        size_t NumPoints() const override;
//        Vector3d Point(size_t index) const override;
//
//    protected:
//        MatrixXd m_mPoints;
//    };
//}
