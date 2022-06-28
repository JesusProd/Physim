#include <PhySim/Geometry/Samplers/SurfaceSampler_Poisson.h>
#include <PhySim/Geometry/SpatialPartition/KDTree.h>
#include <PhySim/Utils/Serializer_Mesh.h>
#include <fstream>
using namespace PhySim;
using namespace std;

int main(int argc, char** argv)
{
    const size_t N = 2000;

    // Load input mesh.
    MatrixXd mX;
    MatrixXi mF;
    Serializer_Mesh::ReadSurface_OBJ("D:/StanfordBunny.obj", mX, mF);
    cout << "Loaded input model with " << mX.rows() << " vertices and " << mF.rows() << " faces." << endl;

    // Perform poisson sampling over the surface.
    cout << "Generating " << N << " samples over the surface... ";
    vector<SurfaceSample> samples;
    SurfaceSampler_Poisson sampler;
    sampler.Setup(mX, mF);
    sampler.Sample(N, samples);
    cout << "Done!" << endl;

    // Compute point statistics.
    cout << "Computing some sampling statistics... ";
    {
        MatrixXd mP(N, 3);
        VectorXd vD(N);
        vD.setConstant(numeric_limits<double>::infinity());

        for (int i = 0; i < N; ++i)
            mP.row(i) = samples[i].vPosition.transpose();

        KDTree tree;
        KDPointCloudObject obj(mP);
        KDTree::QueryResults res;
        tree.Build(&obj, {});

        for (int i = 0; i < N; ++i)
        {
            tree.FindKNN(mP.row(i).transpose(), res, 2); // 2 neighbors, as the first one will be the point itself.
            vD[i] = sqrt(res[1].second);
        }
        
        cout << "Done!" << endl;
        cout << "- Min. point distance: " << vD.minCoeff() << endl;
        cout << "- Max. point distance: " << vD.maxCoeff() << endl;
        cout << "- Avge. point distance: " << vD.mean() << endl;
    }

    // Save to file.
    size_t counter = 0;
    ofstream file("D:/Points.txt");
    file << "[";

    for (auto& sample : samples)
    {
        if (++counter % 3 == 0)
            file << std::endl;

        file << "[" 
             << sample.vPosition[0] << "," 
             << sample.vPosition[1] << "," 
             << sample.vPosition[2] << "],";
    }

    file << "]";

    return 0;
}