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


#include <PhySim/Utils/SpatialDeformer.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	template <class T>
	class ISpatialDistribution
	{
	public:
		ISpatialDistribution() {}
		virtual ~ISpatialDistribution() {}

		virtual T& GetValueAtSpatialPoint(const VectorXd& vx) = 0;
	};

	template <class T>
	class SpatialDistribution : public ISpatialDistribution<T>
	{
	protected:
		vector<PtrS<IDeformer>> m_vdeformer;

	public:
		SpatialDistribution(const vector<PtrS<IDeformer>>& vdeformer = vector<PtrS<IDeformer>>())
		{
			this->m_vdeformer = vdeformer;
		}

		virtual ~SpatialDistribution() { }

		virtual T& GetValueAtSpatialPoint(const VectorXd& vx) override
		{
			VectorXd vxT = vx;
			for (int i = 0; i < (int)this->m_vdeformer.size(); ++i)
				vxT = m_vdeformer[i]->Interpolate(vxT); // Interpolate space

			return this->GetValueAtSpatialPointInternal(vxT);
		}

	protected:
		virtual T& GetValueAtSpatialPointInternal(const VectorXd& vx) = 0;

	};

	template <class T>
	class SpatialDistribution_BisectionX : public SpatialDistribution<T>
	{
	protected:
		T m_value0;
		T m_value1;
		Real m_barrier;

	public:
		SpatialDistribution_BisectionX(Real barrier, T& value0, T& value1, const vector<PtrS<IDeformer>>& vdeformer = vector<PtrS<IDeformer>>()) : SpatialDistribution<T>(vdeformer)
		{
			this->m_value0 = value0;
			this->m_value1 = value1;
			this->m_barrier = barrier;
		}

		virtual ~SpatialDistribution_BisectionX() {}

	protected:
		virtual T& GetValueAtSpatialPointInternal(const VectorXd& vx) override
		{
			if (vx.x() > m_barrier)
				return this->m_value0;
			else return this->m_value1;
		}
	};

	template <class T>
	class SpatialDistribution_StripesX : public SpatialDistribution<T>
	{
	protected:
		T m_value0;
		T m_value1;
		Real m_zero;
		Real m_width;

	public:
		SpatialDistribution_StripesX(Real zero, Real width, T& value0, T& value1, const vector<PtrS<IDeformer>>& vdeformer = vector<PtrS<IDeformer>>()) : SpatialDistribution<T>(vdeformer)
		{
			this->m_value0 = value0;
			this->m_value1 = value1;
			this->m_zero = zero;
			this->m_width = width;
		}

		virtual ~SpatialDistribution_StripesX() {}

	protected:
		virtual T& GetValueAtSpatialPointInternal(const VectorXd& vx) override
		{
			int s = abs((int)(vx.x() / this->m_width));
			if (vx.x() > m_zero)
			{
				if (s % 2 == 0)
					return m_value0; // Even stripe
				else return m_value1; // Odd stripe
			}
			else
			{
				if (s % 2 == 0)
					return m_value1; // Even stripe
				else return m_value0; // Odd stripe
			}
		}
	};

	template <class T>
	class SpatialDistribution_Grid : public SpatialDistribution<T>
	{
	protected:
		T m_value0;
		T m_value1;
		VectorXd m_vzero;
		VectorXd m_vwidth;

	public:
		SpatialDistribution_Grid(const VectorXd& vzero, const VectorXd& vwidth, T& value0, T& value1, const vector<PtrS<IDeformer>>& vdeformer = vector<PtrS<IDeformer>>()) : SpatialDistribution<T>(vdeformer)
		{
			this->m_value0 = value0;
			this->m_value1 = value1;
			this->m_vzero = vzero;
			this->m_vwidth = vwidth;
		}

		virtual ~SpatialDistribution_Grid() {}

	protected:
		virtual T& GetValueAtSpatialPointInternal(const VectorXd& vx) override
		{
			VectorXd vs = (vx - this->m_vzero).cwiseQuotient(this->m_vwidth);
			bool result = true;
			for (int i = 0; i < vs.size(); ++i)
				if (vx(i) > m_vzero(i))
				{
					if (((int)vs(i)) % 2 == 0) result = !result; // Switch
				}
				else
				{
					if (((int)vs(i)) % 2 != 0) result = !result; // Switch
				}

			return (result) ? this->m_value0 : this->m_value1;
		}
	};

}