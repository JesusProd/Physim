#include <PhySim/Extensions/SimulatorLite.h>
#include <PhySim/Utils/Serializer_Mesh.h>
using namespace PhySim;
using namespace std;

int main(int argc, char** argv)
{
    SimulatorLite sim;
    SimulatorLite::SimulatorInput input;
    SimulatorLite::SimulatorOutput output;

    // Initialize cylinders example. Please go to the method for a full explanation of
    // the initialization procedure.
    if (!sim.InitializeExampleCylinders(input))
    {
        cerr << "Unable to initialize example." << endl;
        return 1;
    }

    // Take snapshot of the initial state.
    Serializer_Mesh::WriteVolume_TET("examples/SimulatorLite_Cylinders/assets/InitialState_0",
        input.vSoftObjects[0].mVolVert,
        input.vSoftObjects[0].mVolTetras,
        {});

    Serializer_Mesh::WriteVolume_TET("examples/SimulatorLite_Cylinders/assets/InitialState_1", 
        input.vSoftObjects[1].mVolVert,
        input.vSoftObjects[1].mVolTetras,
        {});

    // Perform simulation. Keep going until the objective function is minimized
    // (or an error is found).

    OSResult res = OR_ONGOING;

    while (res != OR_SUCCESS && res != OR_FAILURE && res != OR_NONDESC)
    {
        cout << "Simulating 25 steps... ";
        res = sim.Simulate(output, 25);

        switch (res)
        {
        case OR_SUCCESS: cout << "Success!" << endl; break;
        case OR_FAILURE: cout << "Failed!" << endl; break;
        case OR_MAXITER: cout << "Maximum iterations reached!" << endl; break;
        case OR_NONDESC: cout << "Non-descendent step found!" << endl; break;
        default: cout << "Invalid error code obtained!" << endl;
        }
    }
    
    // If successful, store final state of the objects.

    Serializer_Mesh::WriteVolume_TET("examples/SimulatorLite_Cylinders/assets/FinalState_0",
        output.vmSoftVertices[0],
        input.vSoftObjects[0].mVolTetras,
        {});

    Serializer_Mesh::WriteVolume_TET("examples/SimulatorLite_Cylinders/assets/FinalState_1",
        output.vmSoftVertices[1],
        input.vSoftObjects[1].mVolTetras,
        {});

    return 0;
}