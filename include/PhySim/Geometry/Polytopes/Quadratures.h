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

#include <PhySim/Utils/MathUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	/**
	* IQuadrature (abstract).
	*
	* Common interface for all FEM quadrature rules of the form U = sum_i w_i U(p_i), for some parametric
	* point p. The class provides information on the dimensionality of the basis, i.e. dim(p), as well as
	* the number and value of points and weights.
	*/
	class IQuadrature
	{
	public:
		IQuadrature() { };
		virtual ~IQuadrature() { };

		/**
		* Returns the dimensionality of the manifold basis, e.g.,
		* 2D for a triangle in R^3 and 3D for a hexahedron in R^3;
		*/
		virtual int DimBasis() const = 0;

		/**
		* The number of quadrature points being interpolated.
		*/
		virtual int NumPoints() const = 0;

		/**
		* The quadrature points in parametric coordinates.
		*/
		virtual const vector<VectorXd>& Points() const = 0;

		/**
		* The weights of the quadrature points.
		*/
		virtual const vector<Real>& Compute_Values() const = 0;
	};

	class Quadrature : public IQuadrature
	{
	protected:
		int					m_dimBasis;
		int					m_numPoints;
		vector<Real>		m_vweights;
		vector<VectorXd>	m_vpoints;

	public:
		inline virtual int DimBasis() const { return this->m_dimBasis; }
		inline virtual int NumPoints() const { return this->m_numPoints; }
		inline virtual const vector<Real>& Compute_Values() const { return this->m_vweights; }
		inline virtual const vector<VectorXd>& Points() const { return this->m_vpoints; }
	};

	class Quadrature_Tri1 : public Quadrature
	{
	public:
		static PtrS<Quadrature_Tri1> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Tri1());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Tri1> PINSTANCE;

	public:
		Quadrature_Tri1()
		{
			this->m_dimBasis = 2;
			this->m_numPoints = 1;
			this->m_vweights.resize(1);
			this->m_vpoints.resize(1);
			this->m_vweights[0] = 0.5;
			this->m_vpoints[0] = Vector3d(1/3, 1/3, 1/3);
		}
	};

	class Quadrature_Quad1 : public Quadrature
	{
	public:
		static  PtrS<Quadrature_Quad1> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Quad1());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Quad1> PINSTANCE;

	public:
		Quadrature_Quad1()
		{
			this->m_dimBasis = 2;
			this->m_numPoints = 1;
			this->m_vweights.resize(1);
			this->m_vpoints.resize(1);
			this->m_vweights[0] = 1.0;
			this->m_vpoints[0] = Vector3d(0, 0, 0);
		}
	};

	class Quadrature_Tet1 : public Quadrature
	{
	public:
		static PtrS<Quadrature_Tet1> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Tet1());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Tet1> PINSTANCE;

	public:
		Quadrature_Tet1()
		{
			this->m_dimBasis = 3;
			this->m_numPoints = 1;
			this->m_vweights.resize(1);
			this->m_vpoints.resize(1);
			this->m_vweights[0] = 1.0/6.0;
			this->m_vpoints[0] = Vector4d(1.0/2.0, 1.0/2.0, 1.0/2.0, 1.0/2.0);
		}
	};

	class Quadrature_Tet4 : public Quadrature
	{
	public:
		static PtrS<Quadrature_Tet4> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Tet4());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Tet4> PINSTANCE;

	public:
		Quadrature_Tet4()
		{
			this->m_dimBasis = 3;
			this->m_numPoints = 4;
			this->m_vpoints.resize(4);
			this->m_vweights.resize(4);

			this->m_vpoints[0] = Vector3d(0.5854101966249685, 0.1381966011250105, 0.1381966011250105);
			this->m_vpoints[1] = Vector3d(0.1381966011250105, 0.1381966011250105, 0.1381966011250105);
			this->m_vpoints[2] = Vector3d(0.1381966011250105, 0.1381966011250105, 0.5854101966249685);
			this->m_vpoints[3] = Vector3d(0.1381966011250105, 0.5854101966249685, 0.1381966011250105);
			this->m_vweights[0] = 0.25 / 6.0;
			this->m_vweights[1] = 0.25 / 6.0;
			this->m_vweights[2] = 0.25 / 6.0;
			this->m_vweights[3] = 0.25 / 6.0;
		}
	};

	class Quadrature_Tet5 : public Quadrature
	{
	public:
		static PtrS<Quadrature_Tet5> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Tet5());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Tet5> PINSTANCE;

	public:
		Quadrature_Tet5()
		{
			this->m_dimBasis = 3;
			this->m_numPoints = 5;
			this->m_vpoints.resize(4);
			this->m_vweights.resize(4);
			this->m_vpoints[0] = Vector3d(0.5854101966249685, 0.1381966011250105, 0.1381966011250105);
			this->m_vpoints[1] = Vector3d(0.1381966011250105, 0.1381966011250105, 0.1381966011250105);
			this->m_vpoints[2] = Vector3d(0.1381966011250105, 0.1381966011250105, 0.5854101966249685);
			this->m_vpoints[3] = Vector3d(0.1381966011250105, 0.5854101966249685, 0.1381966011250105);
			this->m_vweights[0] = 0.25 / 6.0;
			this->m_vweights[1] = 0.25 / 6.0;
			this->m_vweights[2] = 0.25 / 6.0;
			this->m_vweights[3] = 0.25 / 6.0;
		}
	};

	class Quadrature_Tet8 : public Quadrature
	{
	public:
		static PtrS<Quadrature_Tet8> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Tet8());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Tet8> PINSTANCE;

	public:
		Quadrature_Tet8()
		{
			this->m_dimBasis = 3;
			this->m_numPoints = 8;
			this->m_vpoints.resize(8);
			this->m_vweights.resize(8);
			this->m_vpoints[0] = Vector3d(0.009437387888358, 0.035220811090087, 0.166666666666667);
			this->m_vpoints[1] = Vector3d(0.035220811090087, 0.009437387888358, 0.166666666666667);
			this->m_vpoints[2] = Vector3d(0.035220811090087, 0.131445856471988, 0.044658198978444);
			this->m_vpoints[3] = Vector3d(0.131445856471988, 0.035220811090087, 0.044658198978444);
			this->m_vpoints[4] = Vector3d(0.035220810850163, 0.131445855576580, 0.622008467032738);
			this->m_vpoints[5] = Vector3d(0.131445855576580, 0.035220810850163, 0.622008467032738);
			this->m_vpoints[6] = Vector3d(0.131445855576580, 0.490562611456158, 0.166666666666667);
			this->m_vpoints[7] = Vector3d(0.490562611456158, 0.131445855576580, 0.166666666666667);
			this->m_vweights[0] = 0.001179673492382;
			this->m_vweights[1] = 0.001179673492382;
			this->m_vweights[2] = 0.004402601409914;
			this->m_vweights[3] = 0.004402601409914;
			this->m_vweights[4] = 0.016430731923420;
			this->m_vweights[5] = 0.016430731923420;
			this->m_vweights[6] = 0.061320326343747;
			this->m_vweights[7] = 0.061320326343747;
		}
	};

	class Quadrature_Tet11 : public Quadrature
	{
	public:
		static PtrS<Quadrature_Tet11> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Tet11());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Tet11> PINSTANCE;

	public:
		Quadrature_Tet11()
		{
			this->m_dimBasis = 3;
			this->m_numPoints = 11;
			this->m_vpoints.resize(11);
			this->m_vweights.resize(11);
			this->m_vpoints[0] = Vector3d(0.250000000000000, 0.250000000000000, 0.250000000000000);
			this->m_vpoints[1] = Vector3d(0.785714285714286, 0.071428571428571, 0.071428571428571);
			this->m_vpoints[2] = Vector3d(0.071428571428571, 0.071428571428571, 0.071428571428571);
			this->m_vpoints[3] = Vector3d(0.071428571428571, 0.071428571428571, 0.785714285714286);
			this->m_vpoints[4] = Vector3d(0.071428571428571, 0.785714285714286, 0.071428571428571);
			this->m_vpoints[5] = Vector3d(0.100596423833201, 0.399403576166799, 0.399403576166799);
			this->m_vpoints[6] = Vector3d(0.399403576166799, 0.100596423833201, 0.399403576166799);
			this->m_vpoints[7] = Vector3d(0.399403576166799, 0.399403576166799, 0.100596423833201);
			this->m_vpoints[8] = Vector3d(0.399403576166799, 0.100596423833201, 0.100596423833201);
			this->m_vpoints[9] = Vector3d(0.100596423833201, 0.399403576166799, 0.100596423833201);
			this->m_vpoints[10] = Vector3d(0.100596423833201, 0.100596423833201, 0.399403576166799);
			this->m_vweights[0] = -0.0789333333333333 / 6.0;
			this->m_vweights[1] = 0.0457333333333333 / 6.0;
			this->m_vweights[2] = 0.0457333333333333 / 6.0;
			this->m_vweights[3] = 0.0457333333333333 / 6.0;
			this->m_vweights[4] = 0.0457333333333333 / 6.0;
			this->m_vweights[5] = 0.1493333333333333 / 6.0;
			this->m_vweights[6] = 0.1493333333333333 / 6.0;
			this->m_vweights[7] = 0.1493333333333333 / 6.0;
			this->m_vweights[8] = 0.1493333333333333 / 6.0;
			this->m_vweights[9] = 0.1493333333333333 / 6.0;
			this->m_vweights[10] = 0.1493333333333333 / 6.0;
		}
	};

	class Quadrature_Hex1 : public Quadrature
	{
	public:
		static PtrS<Quadrature_Hex1> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Quadrature_Hex1());
			return PINSTANCE;
		}

	private:
		static PtrS<Quadrature_Hex1> PINSTANCE;

	public:
		Quadrature_Hex1()
		{
			this->m_dimBasis = 3;
			this->m_numPoints = 1;
			this->m_vweights.resize(1);
			this->m_vpoints.resize(1);
			this->m_vweights[0] = 8.0;
			this->m_vpoints[0] = Vector4d(0, 0, 0, 0);
		}
	};

	class Quadrature_Gauss : public Quadrature
	{
	public:
		Quadrature_Gauss(int dimBasis, int numPoints)
		{
			this->m_dimBasis = dimBasis;
			this->m_numPoints = (int) pow(numPoints, 3);
			this->m_vweights.resize(this->m_numPoints);
			this->m_vpoints.resize(this->m_numPoints);

			MathUtils::gaussQuadrature(numPoints, -1, 1, this->m_vpoints, this->m_vweights);
		}
	};

	class Quadrature_FineGauss : public Quadrature
	{
	public:
		Quadrature_FineGauss(int numPoints)
		{
			using namespace std;
			using namespace Eigen;
			this->m_numPoints = 8*(int)pow(numPoints, 3);

			double halflength = 1.0/numPoints;
			double weight  =1.0/ pow(numPoints, 3);

			vector<VectorXd> vq;
			vector<double> vw;

			MathUtils::gaussQuadrature(numPoints, -1, 1, vq, vw);

			for (size_t i = 0; i < numPoints; i++)
			for (size_t j = 0; j < numPoints; j++)
			for (size_t k = 0; k < numPoints; k++)
			{
				auto center= Vector3d(
					(2*i+1)*halflength-1,
					(2*j+1)*halflength-1,
					(2*k+1)*halflength-1 );
				for(auto q:vq)
				{
					this->m_vpoints.push_back(center+q/numPoints);
					this->m_vweights.push_back(weight);		
				}						
			}
			
		}
	};

	//Todo
	class Quadrature_Uniform : public Quadrature
	{
	public:
		Quadrature_Uniform(int numPoints)
		{
			
			this->m_numPoints = (int) pow(numPoints, 3);
			this->m_vweights.clear();
			this->m_vpoints.clear();

			double halflength = 1.0/numPoints;
			double weight  =8.0/pow(numPoints, 3);

			for (size_t i = 0; i < numPoints; i++)
			for (size_t j = 0; j < numPoints; j++)
			for (size_t k = 0; k < numPoints; k++)
			{
				this->m_vpoints.push_back(Vector3d(
					(2*i+1)*halflength-1,
					(2*j+1)*halflength-1,
					(2*k+1)*halflength-1 ));

				this->m_vweights.push_back(weight);				
			}
			

		}
	};
}