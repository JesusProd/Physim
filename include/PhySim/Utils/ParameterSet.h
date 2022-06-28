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

	class ParameterSet
	{
	public:
		static const string Param_Thickness;
		static const string Param_Radius0;
		static const string Param_Radius1;
		static const string Param_CollT;
		static const string Param_CollK;
		static const string Param_Pressure;
		static const string Param_Density;
		static const string Param_Poisson;
		static const string Param_StretchK;
		static const string Param_BendingK;
		static const string Param_TwistK;
		static const string Param_ShearK;
		static const string Param_Young;
		static const string Param_Bulk;
		static const string Param_Lame1;
		static const string Param_Lame2;
		static const string Param_SNHAlpha;
		static const string Param_Mooney10;
		static const string Param_Mooney01;
		static const string Param_ShearMod;

		ParameterSet()
		{
			// Nothing to do here...
		}

		virtual ~ParameterSet()
		{
			this->m_params.clear();
		}

		inline virtual const map<string,Real>& ModelParameters() const { return this->m_params; }

		inline virtual map<string, Real>& ModelParameters() { return this->m_params; }

		inline Real GetParameter(const string& name) const
		{
			if (HasParameter(name))
				return (*this)[name]; 
			else throw PhySim::exception("Parameter not found");
		}


		inline void SetParameter(const string& name, Real p)
		{
			if (HasParameter(name))
				(*this)[name] = p;
			else throw PhySim::exception("Parameter not found");
		}

		inline bool HasParameter(const string& name) const
		{
			return this->m_params.find(name) != this->m_params.end();
		}

		inline bool RemParameter(const string& name)
		{
			if (!this->HasParameter(name))
				return false;

			this->m_params.erase(name);

			return true;
		}

		inline void AddParameter(const string& name, const Real& value)
		{
			this->m_params.insert(pair<string, Real>(name, value));

			// Set actual value
			(*this)[name] = value;
		}

		inline void ClearParameters()
		{
			this->m_params.clear();
		}

		inline Real& operator[](const string& name)
		{
			if (!this->HasParameter(name))
				throw exception("Invalid property");
			return this->m_params.at(name);
		}

		inline const Real& operator[](const string& name) const
		{
			if (!this->HasParameter(name))
				throw exception("Invalid property");
			return this->m_params.at(name);
		}

		void InitFromDensity(Real density);
		void InitLinearFromYoungPoisson(Real young, Real poisson, Real density);
		void InitLinearFromLameParameters(Real lame1, Real lame2, Real density);
		void InitSNHFromYoungPoisson(Real young, Real poisson, Real density);

	protected:

		map<string, Real>		m_params;

	};

}
