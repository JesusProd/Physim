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
	using namespace std;
	using namespace Eigen;

	PtrS<Logger> Logger::INSTANCE = PtrS<Logger>(NULL);
}