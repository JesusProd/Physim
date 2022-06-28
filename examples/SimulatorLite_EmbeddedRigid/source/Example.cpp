#include <PhySim/Extensions/SimulatorLite.h>
#include <PhySim/Utils/Serializer_Mesh.h>
using namespace PhySim;
using namespace std;

int run(int subCase)
{
    SimulatorLite sim;
    SimulatorLite::SimulatorInput input;
    SimulatorLite::SimulatorOutput output;

    cout << "Running sub-case " << subCase << endl;

    // Initialize example. Please go to the method for a full explanation of
    // the initialization procedure.
    if (!sim.InitializeEmbeddedRigid(input, subCase))
    {
        cerr << "Unable to initialize example." << endl;
        return 1;
    }

    // Take snapshot of the initial state.
    Serializer_Mesh::WriteVolume_TET("examples/SimulatorLite_EmbeddedRigid/assets/InitialState",
        input.vSoftObjects[0].mVolVert,
        input.vSoftObjects[0].mVolTetras,
        {});

    // Perform simulation. Keep going until the objective function is minimized
    // (or an error is found).

    OSResult res = OR_ONGOING;
    int frame = 0;

    while (res != OR_SUCCESS && res != OR_FAILURE && res != OR_NONDESC)
    {
        cout << "Simulating 10 steps... ";
        res = sim.Simulate(output, 10);

        switch (res)
        {
        case OR_SUCCESS: cout << "Success!" << endl; break;
        case OR_FAILURE: cout << "Failed!" << endl; break;
        case OR_MAXITER: cout << "Maximum iterations reached!" << endl; break;
        case OR_NONDESC: cout << "Non-descendent step found!" << endl; break;
        default: cout << "Invalid error code obtained!" << endl;
        }

        if (res == OR_MAXITER)
        {
            stringstream ss;
            ss << "examples/SimulatorLite_EmbeddedRigid/assets/State" << subCase << "_" << frame;
            ++frame;

            Serializer_Mesh::WriteVolume_TET(ss.str(), output.vmSoftVertices[0],
                input.vSoftObjects[0].mVolTetras, {});
        }
    }
    
    // If successful, store final state of the objects.
    {
        stringstream ss;
        ss << "examples/SimulatorLite_EmbeddedRigid/assets/FinalState" << subCase;
        ++frame;

        Serializer_Mesh::WriteVolume_TET(ss.str(), output.vmSoftVertices[0],
            input.vSoftObjects[0].mVolTetras,
            {});
    }

    cout << "Finished!" << endl;
    cout << endl;

    return 0;
}

int main(int argc, char** argv)
{
    int retCode;

    if ((retCode = run(0)) != 0) return retCode;
    if ((retCode = run(1)) != 0) return retCode;
    if ((retCode = run(2)) != 0) return retCode;

    return 0;
}