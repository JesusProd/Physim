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

#include <PhySim/Traits.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class IDoFSet
	{
	public:
		IDoFSet() { };
		virtual ~IDoFSet() { };

		virtual const int& NumDim() const = 0;

		virtual int& Layer() = 0;
		virtual int& Index() = 0;
		virtual int& Offset() = 0;
		virtual bool& Active() = 0;

		virtual const int& Layer() const = 0;
		virtual const int& Index() const = 0;
		virtual const int& Offset() const = 0;
		virtual const bool& Active() const = 0;

		virtual void SetFixed(bool) = 0;

		virtual const bVector& GetFixed() const = 0;
		virtual void SetFixed(const bVector& fix) = 0;

		virtual bool AnyFixed() const = 0;
		virtual bool AllFixed() const = 0;

		//virtual Geometry* Geometry() = 0;

		virtual VectorXd GetValue() const = 0;
		virtual void SetValue(const VectorXd&) = 0;

		virtual void GetTrait(Tag t, VectorXd&) const = 0;
		virtual void SetTrait(Tag t, const VectorXd&) = 0;
		virtual TraitSet& Traits() = 0;
		virtual const TraitSet& Traits() const = 0;

	};

	class Simulable;
	class Geometry;

	class DoFSet : public IDoFSet
	{
	protected:
		bool							m_active;
		//bool							m_fixed;
		bVector							m_vfixed;
		int								m_numDim;
		int								m_index;
		int								m_offset;
		int								m_layer;
		Simulable*						m_pModel;
		Geometry*						m_pGeometry;
		TraitSet						m_traitSet;

	public:

		DoFSet();

		DoFSet(Simulable* pModel);

		DoFSet(Simulable* pModel, Geometry* pGeometry, int numDim);

		inline virtual ~DoFSet() 
		{
			// Nothing to do here...
		}

		inline virtual Simulable* GetModel() const { return this->m_pModel; }
		inline virtual void SetModel(Simulable* pM) { this->m_pModel = pM; }

		inline virtual const int& NumDim(void) const override { return m_numDim; }
		//inline virtual Geometry* Geometry() override { return this->m_pGeometry; }

		inline virtual int& Layer() override { return m_layer; }
		inline virtual int& Index() override { return m_index; }
		inline virtual int& Offset() override { return m_offset; }
		inline virtual bool& Active() override { return m_active; }

		inline virtual const int& Layer() const override { return m_layer; }
		inline virtual const int& Index() const override { return m_index; }
		inline virtual const int& Offset() const override { return m_offset; }
		inline virtual const bool& Active() const override { return m_active; }

		virtual void SetFixed(bool fix) { for (auto b : this->m_vfixed) b = fix; }

		virtual const bVector& GetFixed() const { return this->m_vfixed; };
		virtual void SetFixed(const bVector& fix) { this->m_vfixed = fix; };

		virtual bool AnyFixed() const { bool any = false; for (auto b : this->m_vfixed) any |= b; return any; };
		virtual bool AllFixed() const { bool all = true; for (auto b : this->m_vfixed) all &= b; return all; }

		virtual void GetTrait(Tag t, VectorXd&) const;
		virtual void SetTrait(Tag t, const VectorXd&);
		virtual TraitSet& Traits();
		virtual const TraitSet& Traits() const;

	};

	//class DoFWorld : public DoFSet
	//{
	//protected:
	//	KinematicsEle* m_pKineDoF;
	//public:
	//	DoFWorld(KinematicsEle* pKineDoF) : DoFSet(pKineDoF->GetModel(), pKineDoF->NumDim(), pKineDoF->Geometry()) 
	//	{
	//		this->m_pKineDoF = pKineDoF;
	//		this->Index() = pKineDoF->Index();
	//		this->Offset() = pKineDoF->Offset();
	//	}
	//	inline virtual VectorXd GetValue() const override { return m_pKineDoF->GetPosition(Tag_Position_X); }
	//	inline virtual void SetValue(const VectorXd& vv) override { m_pKineDoF->SetPosition(vv, Tag_Position_X); }
	//};

	//class DoFRest : public DoFSet
	//{
	//protected:
	//	KinematicsEle* m_pKineDoF;
	//public:
	//	DoFRest(KinematicsEle* pKineDoF) : DoFSet(pKineDoF->GetModel(), pKineDoF->NumDim(), pKineDoF->Geometry())
	//	{
	//		this->m_pKineDoF = pKineDoF;
	//		this->Index() = pKineDoF->Index();
	//		this->Offset() = pKineDoF->Offset();
	//	}
	//	inline virtual VectorXd GetValue() const override { return m_pKineDoF->GetPosition(Tag_Position_0); }
	//	inline virtual void SetValue(const VectorXd& vv) override { m_pKineDoF->SetPosition(vv, Tag_Position_0); }
	//};


}
