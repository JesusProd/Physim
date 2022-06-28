#include <PhySim/Geometry/Samplers/SurfaceSampler_Poisson.h>
#include <unordered_map>
#include <unordered_set>
#include <set>
using namespace PhySim;

/////////////////////////////////////////////////////////////////////////////////////////////////

//struct SamplesObject : public KDTree::Object
//{
//public:
//    SamplesObject(const vector<SurfaceSample>& vSamples)
//        : m_vSamples(vSamples)
//    {
//    }
//
//public:
//    size_t NumPoints() const override 
//    {
//        return m_vSamples.size();
//    }
//
//    Vector3d Point(size_t index) const override
//    {
//        return m_vSamples[index].vPosition;
//    }
//
//private:
//    const vector<SurfaceSample>& m_vSamples;
//};

/////////////////////////////////////////////////////////////////////////////////////////////////

struct WeightComparer
{
public:
    WeightComparer(const vector<double>& vWeights)
        : m_vWeights(vWeights)
    {
    }

public:
    bool operator()(size_t lhs, size_t rhs) const
    {
        return m_vWeights[lhs] > m_vWeights[rhs];
    }

private:
    const vector<double>& m_vWeights;
};

/////////////////////////////////////////////////////////////////////////////////////////////////

SurfaceSampler_Poisson::SurfaceSampler_Poisson()
    : m_ratio(5)
    , m_alpha(8.0)
    , m_beta(0.65)
    , m_gamma(1.5)
{
}

void SurfaceSampler_Poisson::Setup(const MatrixXd& mX, const MatrixXi& mF)
{
    SurfaceSampler::Setup(mX, mF);
    m_USurfaceSampler.Setup(mX, mF);
}

void SurfaceSampler_Poisson::Parameters(size_t ratio, double alpha, double beta, double gamma)
{
    m_ratio = ratio;
    m_alpha = alpha;
    m_beta = beta;
    m_gamma = gamma;
}

void SurfaceSampler_Poisson::Sample(size_t M, vector<SurfaceSample>& vOutput) const
{
    // Generate an input set of uniformly-distributed points.
    vector<SurfaceSample> vInput;
    m_USurfaceSampler.Sample(m_ratio * M, vInput);

    // Then, perform sample elimination to obtain the Poisson disk sample set.
    Elimination(M, vInput, vOutput);
}

void SurfaceSampler_Poisson::Elimination(size_t M, vector<SurfaceSample>& vInput, vector<SurfaceSample>& vOutput) const
{
    //const size_t N = vInput.size();
    //const double RMax = pow(m_totA / (4.0 * sqrt(2.0) * double(N)), 1.0 / 3.0);
    //const double RMin = RMax * m_beta * pow(1.0 - double(N) / M, m_gamma);
    //const double TwoRMax = 2.0 * RMax;
    //const double TwoRMin = 2.0 * RMin;

    //// For improving the efficiency of the elimination process, let's first find
    //// the neighbors and pair-wise weights for each input point. Use a KD-Tree spatial
    //// partition structure to accelerate this process.
    //vector<double> vWeights(N, 0.0);
    //vector<unordered_set<size_t>> vNeigh(N);
    //unordered_map<tuple<size_t, size_t>, double, Utils::XorHasher> mNeighWeights;
    //
    //SamplesObject kdTreeObject(vInput);
    //KDTree kdTree(&kdTreeObject, {});
    //KDTree::QueryResults query;

    //for (size_t i = 0; i < N; ++i)
    //{
    //    kdTree.SearchRadius(vInput[i].vPosition, query, TwoRMax);
    //    vNeigh[i].reserve(query.size() - 1);
    //    
    //    for (auto& neighbor : query)
    //    {
    //        const size_t j = neighbor.first;
    //        const double d = sqrt(neighbor.second);
    //        const double w = pow(1.0 - d / TwoRMax, m_alpha);
    //        if (j == i) continue;

    //        vWeights[i] += w;
    //        vNeigh[i].insert(j);
    //        mNeighWeights[{ i, j }] = w;
    //    }
    //}

    //// Build a priority queue for input samples using the computed weights. Unfortunately,
    //// using STL's priority_queue is inefficient when updating the weights. Instead, the set
    //// data structure allows faster insertion and removal of objects, so use this structure
    //// to mimick the priority queue behavior.
    //// https://stackoverflow.com/questions/649640/how-to-do-an-efficient-priority-update-in-stl-priority-queue
    //WeightComparer sHeapPred(vWeights);
    //set<size_t, WeightComparer> sHeap(vWeights);
    //for (size_t i = 0; i < N; ++i) sHeap.emplace(i);

    //// While the number of samples is not yet the desired...
    //while (sHeap.size() > M)
    //{
    //    // Pull top sample from heap.
    //    size_t i = *sHeap.begin();
    //    sHeap.erase(sHeap.begin());

    //    // Remove weight component from neighbors in the heap.
    //    for (size_t j : vNeigh[i])
    //    {
    //        // Pull out from heap, update weight and re-insert.
    //        sHeap.erase(j);
    //        vWeights[j] -= mNeighWeights[{ i, j }];
    //        sHeap.insert(j);

    //        // Erase i from the neighbors of j.
    //        vNeigh[j].erase(i);
    //    }
    //}

    //// After the algorithm is complete, the remaining samples in the heap approximates
    //// a Poisson-disk distribution. Copy the data to the output vector.
    //vOutput.reserve(vOutput.size() + M);
    //for (size_t i : sHeap) vOutput.push_back(vInput[i]);

    //// Done!
}

void SurfaceSampler_Poisson::Seed(default_random_engine::result_type seed)
{
    // Seed uniform sampler accordingly as well.
    m_USurfaceSampler.Seed(seed);
    SurfaceSampler::Seed(seed);
}