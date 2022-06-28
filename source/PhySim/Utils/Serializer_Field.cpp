//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/Serializer_Field.h>

#include <PhySim/Utils/IOUtils.h>

using namespace std;
using namespace PhySim;

/////////////////////////////////////////////////////////////////////////////////////////////////

// Mapping relating data types to their corresponding format id.

static char* SVLFormatNames[] = { "char", "short", "int", "float", "double" };

template <typename T> struct SVLFormatMapping { static constexpr int Value = -1; };
template <> struct SVLFormatMapping<char>     { static constexpr int Value = 0; };
template <> struct SVLFormatMapping<short>    { static constexpr int Value = 1; };
template <> struct SVLFormatMapping<int>      { static constexpr int Value = 2; };
template <> struct SVLFormatMapping<float>    { static constexpr int Value = 3; };
template <> struct SVLFormatMapping<double>   { static constexpr int Value = 4; };

// Helper function for reading arbitrary grid scalar fields in SVL format.

template <typename T>
static bool readGridSVL(const string& filePath, ScalarField_Grid<T>& field)
{
    static constexpr int ScalarFormat = SVLFormatMapping<T>::Value;
    static_assert(ScalarFormat != -1, "SVL format does not support the given scalar type.");

    // Open stream to SVL file.
    ifstream f(filePath, ios::binary);
    if (!f) return false;

    // Read SVL header.
    Vector3i vShape; IOUtils::readBinaryArray(f, vShape.data(), 3);
    Vector3d vOrigin; IOUtils::readBinaryArray(f, vOrigin.data(), 3);
    Vector3d vDelta; IOUtils::readBinaryArray(f, vDelta.data(), 3);
    int format; IOUtils::readBinaryValue(f, format);
    assert(format >= 0 && format <= 4 && "Invalid SVL format. File may be corrupted?");

    // Allocate grid data.
    field.Setup(vShape, vOrigin, vDelta);
    int count = field.Grid().size();

    // Read grid data. Check if the formats match between the file and the supplied grid. If they don't
    // perform conversion anyway but display warning.
    if (format != ScalarFormat)
    {
        IOUtils::logTrace(Verbosity::V1_Default, "[WARNING] The supplied grid's scalar type differs from SVL's. Conversion will happen from "
                 "%s to %s, which may incur in data loss.\n", SVLFormatNames[format], SVLFormatNames[ScalarFormat]);

        switch (format)
        {
        case 0: IOUtils::readBinaryArrayWithCast<char, T>(f, field.Grid().data(), count); break;
        case 1: IOUtils::readBinaryArrayWithCast<short, T>(f, field.Grid().data(), count); break;
        case 2: IOUtils::readBinaryArrayWithCast<int, T>(f, field.Grid().data(), count); break;
        case 3: IOUtils::readBinaryArrayWithCast<float, T>(f, field.Grid().data(), count); break;
        case 4: IOUtils::readBinaryArrayWithCast<double, T>(f, field.Grid().data(), count); break;
        }
    }
    else
    {
        IOUtils::readBinaryArray(f, field.Grid().data(), count);
    }
    
    // Done!
    f.close();
    return f.good();
}

template <typename T>
static bool writeGridSVL(const string& filePath, const ScalarField_Grid<T>& field)
{
    static constexpr int ScalarFormat = SVLFormatMapping<T>::Value;
    static_assert(ScalarFormat != -1, "SVL format does not support the given scalar type.");

    // Open stream to SVL file.
    std::ofstream f(filePath, ios::binary);
    if (!f) return false;

    // Write SVL header
    const Vector3i vShape = field.Shape(); IOUtils::writeBinaryArray(f, vShape.data(), 3);
    const Vector3d vOrigin = field.Origin(); IOUtils::writeBinaryArray(f, vOrigin.data(), 3);
    const Vector3d vDelta = field.Delta(); IOUtils::writeBinaryArray(f, vDelta.data(), 3);
    IOUtils::writeBinaryValue(f, ScalarFormat);
    int count = vShape.prod();

    // Write grid data.
    IOUtils::writeBinaryArray(f, field.Grid().data(), count);
    
    // Done!
    f.close();
    return f.good();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

bool PhySim::Serializer_Field::ReadGrid_SVL(const string& filePath, ScalarField_Grid<char>& field) { return readGridSVL(filePath, field); }
bool PhySim::Serializer_Field::ReadGrid_SVL(const string& filePath, ScalarField_Grid<short>& field) { return readGridSVL(filePath, field); }
bool PhySim::Serializer_Field::ReadGrid_SVL(const string& filePath, ScalarField_Grid<int>& field) { return readGridSVL(filePath, field); }
bool PhySim::Serializer_Field::ReadGrid_SVL(const string& filePath, ScalarField_Grid<float>& field) { return readGridSVL(filePath, field); }
bool PhySim::Serializer_Field::ReadGrid_SVL(const string& filePath, ScalarField_Grid<double>& field) { return readGridSVL(filePath, field); }

bool PhySim::Serializer_Field::WriteGrid_SVL(const string& filePath, const ScalarField_Grid<char>& field) { return writeGridSVL(filePath, field); }
bool PhySim::Serializer_Field::WriteGrid_SVL(const string& filePath, const ScalarField_Grid<short>& field) { return writeGridSVL(filePath, field); }
bool PhySim::Serializer_Field::WriteGrid_SVL(const string& filePath, const ScalarField_Grid<int>& field) { return writeGridSVL(filePath, field); }
bool PhySim::Serializer_Field::WriteGrid_SVL(const string& filePath, const ScalarField_Grid<float>& field) { return writeGridSVL(filePath, field); }
bool PhySim::Serializer_Field::WriteGrid_SVL(const string& filePath, const ScalarField_Grid<double>& field) { return writeGridSVL(filePath, field); }

/////////////////////////////////////////////////////////////////////////////////////////////////
