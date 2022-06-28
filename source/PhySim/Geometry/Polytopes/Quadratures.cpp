//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Quadratures.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	PtrS<Quadrature_Tri1>		Quadrature_Tri1::PINSTANCE = PtrS<Quadrature_Tri1>(NULL);
	PtrS<Quadrature_Quad1>		Quadrature_Quad1::PINSTANCE = PtrS<Quadrature_Quad1>(NULL);
	PtrS<Quadrature_Tet1>		Quadrature_Tet1::PINSTANCE = PtrS<Quadrature_Tet1>(NULL);
	PtrS<Quadrature_Tet4>		Quadrature_Tet4::PINSTANCE = PtrS<Quadrature_Tet4>(NULL);
	PtrS<Quadrature_Tet5>		Quadrature_Tet5::PINSTANCE = PtrS<Quadrature_Tet5>(NULL);
	PtrS<Quadrature_Tet8>		Quadrature_Tet8::PINSTANCE = PtrS<Quadrature_Tet8>(NULL);
	PtrS<Quadrature_Tet11>		Quadrature_Tet11::PINSTANCE = PtrS<Quadrature_Tet11>(NULL);
	PtrS<Quadrature_Hex1>		Quadrature_Hex1::PINSTANCE = PtrS<Quadrature_Hex1>(NULL);

}