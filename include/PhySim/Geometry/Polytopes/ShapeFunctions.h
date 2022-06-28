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


#include <PhySim/Geometry/Polytopes/Interpolations.h>

#include <PhySim/Geometry/Polytopes/Poly.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <numeric>
#include <algorithm>

namespace PhySim
{

	using namespace std;
	using namespace Eigen;

	/**
	* IShapeFunction (abstract).
	*
	* Common interface for all polytope shape (interpolation) function of the form x = sum_i N_i(p), for
	* some parametetric point p. The class provides information on the dimensionality of the basis, i.e.
	* dim(p), and the space, i.e. dim(x). It must also provide information on the number of interpolated
	* control points, i.e., 4 in the case of a linear tetrahedron, together with the interpolation weight
	* N_i(p) and partial derivatives, dN_i(p)/dp.
	*/
	class IShapeFunction
	{
	public:
		IShapeFunction() { };
		virtual ~IShapeFunction() { };

		/**
		* Returns the dimensionality of the space, e.g. 3 for 3D.
		*/
		virtual int DimSpace() const = 0;

		/**
		* Returns the dimensionality of the manifold basis, e.g.,
		* 2D for a triangle in R^3 and 3D for a hexahedron in R^3;
		*/
		virtual int DimBasis() const = 0;

		/**
		* Returns the number of control points for the interpolation.
		* Should be equal to the number of nodes of the polytope that
		* is being interpolated.
		*/
		virtual int NumPoints() const = 0;

		virtual bool IsValidParametric(const VectorXd& vp) const = 0;

		virtual void InterpolateDeformation(const Poly* pPoly, Tag trait0, Tag traitX, const MatrixXd& mP, MatrixXd& mI) const = 0;
		virtual void InterpolateDeformation(const MatrixXd& mN0, const MatrixXd& mNx, const MatrixXd& mP, MatrixXd& mI) const = 0;
		virtual void InterpolateValue(const Poly* pPoly, Tag trait, const MatrixXd& mP, MatrixXd& mI) const = 0;
		virtual void InterpolateValue(const MatrixXd& mN, const MatrixXd& mP, MatrixXd& mI) const = 0;

		///////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////// DEFORMATION ///////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////

		struct IDeformationData
		{
		public:
			virtual void SetPoly(const Poly* pPoly) = 0;
			virtual void SetTag0(Tag t0) = 0;
			virtual void SetTagX(Tag tX) = 0;
			virtual void SetPoints(const vector<VectorXd>& vp) = 0;
			virtual const vector<MatrixXd>& Get_J() const = 0;
			virtual const vector<MatrixXd>& Get_F() const = 0;
			virtual const vector<MatrixXd>& Get_DFDx() const = 0;

			IDeformationData() { };
			virtual ~IDeformationData() { };
		};

		virtual PtrS<IDeformationData> CreateDeformationData() const = 0;
		virtual void InitDeformationAtSamples(PtrS<IDeformationData> pData) const = 0;
		virtual void UpdateKinematicsAtSamples(PtrS<IDeformationData> pData) const = 0;
		virtual void UpdateDeformationAtSamples(PtrS<IDeformationData> pData) const = 0;

	};

	class ShapeFunction : public IShapeFunction
	{
	public:
		struct DeformationData : public IDeformationData
		{
		public:
			const Poly* m_pElemPoly;
			Tag m_tag0;
			Tag m_tagX;
			vector<VectorXd> m_vpoints;
			vector<MatrixXd> m_vmJ;
			vector<MatrixXd> m_vmF;
			vector<MatrixXd> m_vmDFDx;
			bool m_isInit;

			virtual void SetPoly(const Poly* pPoly) override { this->m_pElemPoly = pPoly; };
			virtual void SetTag0(Tag t0) override { this->m_tag0 = t0; };
			virtual void SetTagX(Tag tX) override { this->m_tagX = tX; };
			virtual void SetPoints(const vector<VectorXd>& vp) override { this->m_vpoints = vp; }
			virtual const vector<MatrixXd>& Get_J() const override { return this->m_vmJ; };
			virtual const vector<MatrixXd>& Get_F() const override { return this->m_vmF; };
			virtual const vector<MatrixXd>& Get_DFDx() const override { return this->m_vmDFDx; };

			DeformationData() { this->m_isInit = false; };
			virtual ~DeformationData() { };
		};

		virtual PtrS<IShapeFunction::IDeformationData> CreateDeformationData() const
		{
			return PtrS<IShapeFunction::IDeformationData>(new DeformationData());
		}

		virtual void InitDeformationAtSamples(PtrS<IDeformationData> pData) const override { }
		virtual void UpdateKinematicsAtSamples(PtrS<IDeformationData> pData) const override { }
		virtual void UpdateDeformationAtSamples(PtrS<IDeformationData> pData) const override { }


	protected:

		virtual void Compute_MatrixX(MatrixXd& mNX, PtrS<IShapeFunction::IDeformationData> pDataParent) const;
		virtual void Compute_Matrix0(MatrixXd& mN0, PtrS<IShapeFunction::IDeformationData> pDataParent) const;

	};

	class ShapeFunction_LERP : public ShapeFunction
	{
	protected:
		PtrS<IInterpolation> m_pInterpolation;
	public:

		ShapeFunction_LERP() { };
		virtual ~ShapeFunction_LERP() { };

		virtual int DimSpace() const override { return 3; };

		virtual void InitDeformationAtSamples(PtrS<IShapeFunction::IDeformationData> pDataParent) const override;
		virtual void UpdateDeformationAtSamples(PtrS<IShapeFunction::IDeformationData> pDataParent) const override;

		virtual void InterpolateDeformation(const Poly* pPoly, Tag trait0, Tag traitX, const MatrixXd& mP, MatrixXd& mI) const override;
		virtual void InterpolateDeformation(const MatrixXd& mN0, const MatrixXd& mNx, const MatrixXd& mP, MatrixXd& mI) const override;
		virtual void InterpolateValue(const Poly* pPoly, Tag trait, const MatrixXd& mP, MatrixXd& mI) const override;
		virtual void InterpolateValue(const MatrixXd& mN, const MatrixXd& mP, MatrixXd& mI) const override;

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const = 0;
		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const = 0;

	};

	class ShapeFunction_Node : public ShapeFunction_LERP
	{
	public:
		static PtrS<ShapeFunction_Node> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Node());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Node>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 0; };

		virtual int NumPoints() const override { return 1; };

		virtual bool IsValidParametric(const VectorXd& vpoint) const override
		{
			return true;
		}

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			vN.resize(1);
			vN(0) = 1.0;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB = MatrixXd();
		}

	};

	class ShapeFunction_Tri3 : public ShapeFunction_LERP
	{
	public:
		static PtrS<ShapeFunction_Tri3> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Tri3());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Tri3>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 3; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != this->DimBasis())
				return false;

			Real sum = 0;

			for (int i = 0; i < this->DimBasis(); ++i)
			{
				if (vp(i) < 0)
					return false;

				if (vp(i) > 1)
					return false;

				sum += vp(i);
			}

			return sum >= 0 && sum <= 1;
		}

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			vN.resize(3);
			vN(1) = vp(0);
			vN(2) = vp(1);
			vN(0) = 1.0f - vp(0) - vp(1);
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(3, 2);
			mB.row(0) = Vector2d(-1, -1);
			mB.row(1) = Vector2d(1, 0);
			mB.row(2) = Vector2d(0, 1);
		}

	};

	class ShapeFunction_Tri6 : public ShapeFunction_Tri3
	{
	public:
		static PtrS<ShapeFunction_Tri6> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Tri6());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Tri6>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 6; };

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			throw PhySim::exception("Not implemented");
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			throw PhySim::exception("Not implemented");
		}

	};

	class ShapeFunction_Quad4 : public ShapeFunction_LERP
	{
	public:
		static PtrS<ShapeFunction_Quad4> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Quad4());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Quad4> PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 3; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != this->DimBasis())
				return false;

			for (int i = 0; i < this->DimBasis(); ++i)
			{
				if (vp(i) < -1)
					return false;

				if (vp(i) > 1)
					return false;
			}

			return true;
		}

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			vN.resize(4);
			Real factor = 1.0 / 4.0;
			vN(0) = (1 - vp[0])*(1 - vp[1]);
			vN(1) = (1 + vp[0])*(1 - vp[1]);
			vN(2) = (1 + vp[0])*(1 + vp[1]);
			vN(3) = (1 - vp[0])*(1 + vp[1]);
			vN = factor * vN;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(4, 2);
			Real factor = 1.0 / 4.0;
			// DNDp1
			mB(0, 0) = -(1 - vp[1]);
			mB(1, 0) = +(1 - vp[1]);
			mB(2, 0) = +(1 + vp[1]);
			mB(3, 0) = -(1 + vp[1]);
			// DNDp1
			mB(0, 1) = -(1 - vp[0]);
			mB(1, 1) = -(1 + vp[0]);
			mB(2, 1) = +(1 + vp[0]);
			mB(3, 1) = +(1 - vp[0]);
			mB = factor * mB;
		}

	};

	class ShapeFunction_Quad8 : public ShapeFunction_Quad4
	{
	public:
		static PtrS<ShapeFunction_Quad8> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Quad8());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Quad8> PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 3; };

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			throw PhySim::exception("Not implemented");
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			throw PhySim::exception("Not implemented");
		}

	};

	class ShapeFunction_Tet4 : public ShapeFunction_LERP
	{
	public:
		static PtrS<ShapeFunction_Tet4> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Tet4());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Tet4>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 4; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != this->DimBasis())
				return false;

			Real sum = 0;

			for (int i = 0; i < this->DimBasis(); ++i)
			{
				if (vp(i) < 0)
					return false;

				if (vp(i) > 1)
					return false;

				sum += vp(i);
			}

			return sum >= 0 && sum <= 1;
		}

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			vN.resize(4);
			vN(1) = vp(0);
			vN(2) = vp(1);
			vN(3) = vp(2);
			vN(0) = 1.0 - vp(0) - vp(1) - vp(2);
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(4, 3);
			mB.row(0) = Vector3d(-1, -1, -1);
			mB.row(1) = Vector3d(1, 0, 0);
			mB.row(2) = Vector3d(0, 1, 0);
			mB.row(3) = Vector3d(0, 0, 1);
		}

	};

	class ShapeFunction_Tet10 : public ShapeFunction_Tet4
	{

	public:
		static PtrS<ShapeFunction_Tet10> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Tet10());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Tet10> PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 10; };

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			vN.resize(10);
			Real t0 = 1.0 - vp(0) - vp(1) - vp(2);
			Real t1 = vp(0);
			Real t2 = vp(1);
			Real t3 = vp(2);
			vN(0) = t0 * (2 * t0 - 1);
			vN(1) = t1 * (2 * t1 - 1);
			vN(2) = t2 * (2 * t2 - 1);
			vN(3) = t3 * (2 * t3 - 1);
			vN(4) = 4 * t0*t1;
			vN(5) = 4 * t1*t2;
			vN(6) = 4 * t2*t0;
			vN(7) = 4 * t0*t3;
			vN(8) = 4 * t1*t3;
			vN(9) = 4 * t2*t3;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			Real t0 = 1.0 - vp(0) - vp(1) - vp(2);
			Real t1 = vp(0);
			Real t2 = vp(1);
			Real t3 = vp(2);

			MatrixXd DNDt(10, 4);
			DNDt.row(0) = Vector4d(4 * t0 - 1, 0, 0, 0);
			DNDt.row(1) = Vector4d(0, 4 * t1 - 1, 0, 0);
			DNDt.row(2) = Vector4d(0, 0, 4 * t2 - 1, 0);
			DNDt.row(3) = Vector4d(0, 0, 0, 4 * t3 - 1);
			DNDt.row(4) = Vector4d(4 * t1, 4 * t0, 0, 0);
			DNDt.row(5) = Vector4d(0, 4 * t2, 4 * t1, 0);
			DNDt.row(6) = Vector4d(4 * t2, 0, 4 * t0, 0);
			DNDt.row(7) = Vector4d(4 * t3, 0, 0, 4 * t0);
			DNDt.row(8) = Vector4d(0, 4 * t3, 0, 4 * t1);
			DNDt.row(9) = Vector4d(0, 0, 4 * t3, 4 * t2);

			MatrixXd DtDp(4, 3);
			DtDp.row(0) = Vector3d(-1, -1, -1);
			DtDp.row(1) = Vector3d(1, 0, 0);
			DtDp.row(2) = Vector3d(0, 1, 0);
			DtDp.row(3) = Vector3d(0, 0, 1);

			mB = DNDt * DtDp;
		}

	};

	class ShapeFunction_Hex8 : public ShapeFunction_LERP
	{

	public:
		static PtrS<ShapeFunction_Hex8> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Hex8());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Hex8>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 8; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != this->DimBasis())
				return false;

			Real sum = 0;

			for (int i = 0; i < this->DimBasis(); ++i)
			{
				if (vp(i) < -1 - 1e-6)
					return false;

				if (vp(i) > +1 + 1e-6)
					return false;
			}

			return true;
		}

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			vN.resize(8);
			Real factor = 1.0 / 8.0;
			vN(0) = (1 - vp[0])*(1 - vp[1])*(1 - vp[2]);
			vN(1) = (1 + vp[0])*(1 - vp[1])*(1 - vp[2]);
			vN(2) = (1 + vp[0])*(1 + vp[1])*(1 - vp[2]);
			vN(3) = (1 - vp[0])*(1 + vp[1])*(1 - vp[2]);
			vN(4) = (1 - vp[0])*(1 - vp[1])*(1 + vp[2]);
			vN(5) = (1 + vp[0])*(1 - vp[1])*(1 + vp[2]);
			vN(6) = (1 + vp[0])*(1 + vp[1])*(1 + vp[2]);
			vN(7) = (1 - vp[0])*(1 + vp[1])*(1 + vp[2]);
			vN = factor * vN;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(8, 3);
			Real factor = 1.0 / 8.0;
			// DNDp1
			mB(0, 0) = -(1 - vp[1])*(1 - vp[2]);
			mB(1, 0) = +(1 - vp[1])*(1 - vp[2]);
			mB(2, 0) = +(1 + vp[1])*(1 - vp[2]);
			mB(3, 0) = -(1 + vp[1])*(1 - vp[2]);
			mB(4, 0) = -(1 - vp[1])*(1 + vp[2]);
			mB(5, 0) = +(1 - vp[1])*(1 + vp[2]);
			mB(6, 0) = +(1 + vp[1])*(1 + vp[2]);
			mB(7, 0) = -(1 + vp[1])*(1 + vp[2]);
			// DNDp1
			mB(0, 1) = -(1 - vp[0])*(1 - vp[2]);
			mB(1, 1) = -(1 + vp[0])*(1 - vp[2]);
			mB(2, 1) = +(1 + vp[0])*(1 - vp[2]);
			mB(3, 1) = +(1 - vp[0])*(1 - vp[2]);
			mB(4, 1) = -(1 - vp[0])*(1 + vp[2]);
			mB(5, 1) = -(1 + vp[0])*(1 + vp[2]);
			mB(6, 1) = +(1 + vp[0])*(1 + vp[2]);
			mB(7, 1) = +(1 - vp[0])*(1 + vp[2]);
			// DNDp2
			mB(0, 2) = -(1 - vp[0])*(1 - vp[1]);
			mB(1, 2) = -(1 + vp[0])*(1 - vp[1]);
			mB(2, 2) = -(1 + vp[0])*(1 + vp[1]);
			mB(3, 2) = -(1 - vp[0])*(1 + vp[1]);
			mB(4, 2) = +(1 - vp[0])*(1 - vp[1]);
			mB(5, 2) = +(1 + vp[0])*(1 - vp[1]);
			mB(6, 2) = +(1 + vp[0])*(1 + vp[1]);
			mB(7, 2) = +(1 - vp[0])*(1 + vp[1]);
			mB = factor * mB;
		}

	};

	class ShapeFunction_Hex20 : public ShapeFunction_Hex8
	{
	public:
		static PtrS<ShapeFunction_Hex20> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new ShapeFunction_Hex20());
			return PINSTANCE;
		}

	private:
		static PtrS<ShapeFunction_Hex20>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 8; };

		virtual void Compute_Values(const VectorXd& vp, VectorXd& vN) const override
		{
			throw PhySim::exception("Not implemented");
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			throw PhySim::exception("Not implemented");
		}
	
	};

	class ShapeFunction_Chen : public ShapeFunction_Hex8
	{
	protected:
		vector<vector<Matrix3d>> m_vmNij;
		VectorXi m_vCoarsening;
		int m_fineNodesPerElem;
		int m_fineElemsPerElem;
		MatrixXi m_vHCoar;
		MatrixXi m_mHFine;
		MatrixXd m_mVFine;

	public:

		struct DeformationData : public ShapeFunction::DeformationData
		{
		public:
			PtrS<ShapeFunction::DeformationData> m_pRotationSamples;	// Central sample used to compute element rotation

			Matrix3d m_mR; // Matrix storing the computed rotation of the element 
			MatrixXd m_mU; // Matrix storing the unrotated coarse displacements
			vector<vector<Vector3d>> m_vDNDX; // Vector of vDNDN[k][j] vectors
			VectorXi m_vSubEle;
			Vector3d m_vSizes;

			DeformationData() { };
			virtual ~DeformationData() { };
		};

		ShapeFunction_Chen(const vector<vector<Matrix3d>>& vmnij, const VectorXi& vCoarsening);

		virtual ~ShapeFunction_Chen() { };

		virtual PtrS<IShapeFunction::IDeformationData> CreateDeformationData() const
		{
			PtrS<DeformationData> pDefData = PtrS<DeformationData>(new DeformationData());
			pDefData->m_pRotationSamples = dynamic_pointer_cast<ShapeFunction::DeformationData>(ShapeFunction_Hex8::CreateDeformationData());
			return dynamic_pointer_cast<IShapeFunction::IDeformationData>(pDefData);
		}

		virtual void InitDeformationAtSamples(PtrS<IShapeFunction::IDeformationData> pDataParent) const override;
		virtual void UpdateKinematicsAtSamples(PtrS<IShapeFunction::IDeformationData> pDataParent) const override;
		virtual void UpdateDeformationAtSamples(PtrS<IShapeFunction::IDeformationData> pDataParent) const override;

		virtual void InterpolateDeformation(const MatrixXd& mN0, const MatrixXd& mNx, const MatrixXd& mP, MatrixXd& mI) const override;
		virtual void InterpolateValue(const MatrixXd& mN, const MatrixXd& mP, MatrixXd& mI) const override;

	protected:

		virtual void Compute_F(MatrixXd& mF, PtrS<IShapeFunction::IDeformationData> pDataParent, int k) const;
		virtual void Compute_DFDx(MatrixXd& mDFDx, PtrS<IShapeFunction::IDeformationData> pDataParent, int k) const;

		int ChooseSubelement(const VectorXd& vp) const;

	};

}
