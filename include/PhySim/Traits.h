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

#include <PhySim/Utils/ParameterSet.h>

//#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	// Data traits


	class IDoFSet;
	class ParameterSet;
	class KinematicsEle;

	class TraitSet
	{
	private:
		class TraitCase
		{
		public:
			void* pValue = NULL;

			TraitCase() { pValue = NULL; }
			virtual ~TraitCase() { }
			virtual TraitCase* Clone() const = 0;
		};

		template <class T>
		class TraitCaseT : public TraitCase
		{
		public:
			TraitCaseT(const T& v) { pValue = new T(v); }
			virtual ~TraitCaseT() { delete static_cast<T*>(pValue); }
			virtual TraitCase* Clone() const { return new TraitCaseT<T>(*static_cast<T*>(pValue)); }
		};

		map<Tag, TraitCase*> m_mData;

	public:
		TraitSet()
		{

		}

		TraitSet(const TraitSet& toCopy)
		{
			this->CloneTraits(toCopy);
		}

		virtual ~TraitSet()
		{
			this->ClearTraits();
		}

		TraitSet& operator=(const TraitSet& other)
		{
			if (this == &other)
				return *this;

			this->CloneTraits(other);

			return *this;
		}


		// Non-constant interface

		int& Int(Tag s)
		{
			return *static_cast<int*>(m_mData.at(s)->pValue);
		}

		double& Double(Tag s)
		{
			return *static_cast<double*>(m_mData.at(s)->pValue);
		}

		Eigen::Vector3d& Vector3d(Tag s)
		{
			return *static_cast<Eigen::Vector3d*>(m_mData.at(s)->pValue);
		}

		Eigen::Matrix3d& Matrix3d(Tag s)
		{
			return *static_cast<Eigen::Matrix3d*>(m_mData.at(s)->pValue);
		}

		Eigen::VectorXd& VectorXd(Tag s)
		{
			return *static_cast<Eigen::VectorXd*>(m_mData.at(s)->pValue);
		}

		Eigen::MatrixXd& MatrixXd(Tag s)
		{
			return *static_cast<Eigen::MatrixXd*>(m_mData.at(s)->pValue);
		}

		Eigen::VectorXi& VectorXi(Tag s) 		
		{
			return *static_cast<Eigen::VectorXi*>(m_mData.at(s)->pValue);
		}

		Eigen::MatrixXi& MatrixXi(Tag s)
		{
			return *static_cast<Eigen::MatrixXi*>(m_mData.at(s)->pValue);
		}

		PhySim::ParameterSet& ParameterSet(Tag s)
		{
			return *static_cast<PhySim::ParameterSet*>(m_mData.at(s)->pValue);
		}

		PhySim::Frame3d& Frame3d(Tag s)
		{
			return *static_cast<PhySim::Frame3d*>(m_mData.at(s)->pValue);
		}

		PhySim::IDoFSet* DoFSet(Tag s)
		{
			return *static_cast<PhySim::IDoFSet**>(m_mData.at(s)->pValue);
		}

	    PhySim::KinematicsEle* Kinematics(Tag s)
		{
			return *static_cast<PhySim::KinematicsEle**>(m_mData.at(s)->pValue);
		}

		template <class T> T& Trait(Tag s)
		{
			return *static_cast<T*>(m_mData.at(s)->pValue);
		}

		// Constant interface

		const int& Int(Tag s) const
		{
			return *static_cast<int*>(m_mData.at(s)->pValue);
		}

		const double& Double(Tag s) const
		{
			return *static_cast<double*>(m_mData.at(s)->pValue);
		}

		const Eigen::Vector3d& Vector3d(Tag s) const
		{
			return *static_cast<Eigen::Vector3d*>(m_mData.at(s)->pValue);
		}

		const Eigen::Matrix3d& Matrix3d(Tag s) const
		{
			return *static_cast<Eigen::Matrix3d*>(m_mData.at(s)->pValue);
		}

		const Eigen::VectorXd& VectorXd(Tag s) const
		{
			return *static_cast<Eigen::VectorXd*>(m_mData.at(s)->pValue);
		}

		const Eigen::MatrixXd& MatrixXd(Tag s) const
		{
			return *static_cast<Eigen::MatrixXd*>(m_mData.at(s)->pValue);
		}

		const Eigen::VectorXi& VectorXi(Tag s) const
		{
			return *static_cast<Eigen::VectorXi*>(m_mData.at(s)->pValue);
		}

		const Eigen::MatrixXi& MatrixXi(Tag s) const
		{
			return *static_cast<Eigen::MatrixXi*>(m_mData.at(s)->pValue);
		}

		const PhySim::ParameterSet& ParameterSet(Tag s) const
		{
			return *static_cast<PhySim::ParameterSet*>(m_mData.at(s)->pValue);
		}

		const PhySim::Frame3d& Frame3d(Tag s) const
		{
			return *static_cast<PhySim::Frame3d*>(m_mData.at(s)->pValue);
		}

		const PhySim::IDoFSet* DoFSet(Tag s) const
		{
			return *static_cast<PhySim::IDoFSet**>(m_mData.at(s)->pValue);
		}

		const PhySim::KinematicsEle* Kinematics(Tag s) const
		{
			return *static_cast<PhySim::KinematicsEle**>(m_mData.at(s)->pValue);
		}

		template <class T> const T& Trait(Tag s) const
		{
			return *static_cast<T*>(m_mData.at(s)->pValue);
		}

		// Add/remove data

		void ClearTraits()
		{
			map<Tag, TraitCase*>::iterator itCur = m_mData.begin();
			map<Tag, TraitCase*>::iterator itEnd = m_mData.end();
			for (; itCur != itEnd; ++itCur)
				delete itCur->second;
			this->m_mData.clear();
		}

		void CloneTraits(const TraitSet& toCopy)
		{
			this->ClearTraits();

			map<Tag, TraitCase*>::const_iterator itCur = toCopy.m_mData.begin();
			map<Tag, TraitCase*>::const_iterator itEnd = toCopy.m_mData.end();
			for (; itCur != itEnd; ++itCur)
			{
				this->m_mData.insert(pair<Tag, TraitCase*>(itCur->first, itCur->second->Clone()));
			}
		}

		template <class T>
		bool AddTrait(Tag s, const T& value)
		{
			bool has = HasTrait(s);
			if (has) this->RemTrait(s);

			this->m_mData.insert(pair<Tag, TraitCase*>(s, new TraitCaseT<T>(value)));

			return !has;
		}

		bool AddTrait(Tag s, const int& value)
		{
			return AddTrait<int>(s, value);
		}

		bool AddTrait(Tag s, const double& value)
		{
			return AddTrait<double>(s, value);
		}

		bool AddTrait(Tag s, const Eigen::Vector3d& value)
		{
			return AddTrait<Eigen::Vector3d>(s, value);
		}

		bool AddTrait(Tag s, const Eigen::Matrix3d& value)
		{
			return AddTrait<Eigen::Matrix3d>(s, value);
		}

		bool AddTrait(Tag s, const Eigen::VectorXd& value)
		{
			return AddTrait<Eigen::VectorXd>(s, value);
		}

		bool AddTrait(Tag s, const Eigen::MatrixXd& value)
		{
			return AddTrait<Eigen::MatrixXd>(s, value);
		}

		bool AddTrait(Tag s, const Eigen::VectorXi& value)
		{
			return AddTrait<Eigen::VectorXi>(s, value);
		}

		bool AddTrait(Tag s, const Eigen::MatrixXi& value)
		{
			return AddTrait<Eigen::MatrixXi>(s, value);
		}

		bool AddTrait(Tag s, const PhySim::ParameterSet& value)
		{
			return AddTrait<PhySim::ParameterSet>(s, value);
		}

		bool AddTrait(Tag s, const PhySim::Frame3d& value)
		{
			return AddTrait<PhySim::Frame3d>(s, value);
		}

		bool RemTrait(const Tag& s)
		{
			if (!HasTrait(s))
			{
				return false;
			}
			else
			{
				delete this->m_mData.find(s)->second;
				this->m_mData.erase(s);
				return true;
			}
		}

		bool HasTrait(const Tag& sema)
		{
			return this->m_mData.find(sema) != this->m_mData.end();
		}
	};

}