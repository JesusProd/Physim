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

#include <PhySim/Physics/Boundary/BCondition.h>
#include <PhySim/Physics/Layer.h>
#include <PhySim/Physics/Simulables/Simulable_Composite.h>
#include <PhySim/Utils/Signal.h>
#include <unordered_set>

namespace PhySim {
class Collider;
class Collider_SphereCloud;
class Collider_DistanceField;

class BC_Collisions : public BCondition {
 public:
  struct Contact {
    size_t Index;
    VectorXd Point;
    VectorXd Normal;
    const void* pData;
  };

  struct ContactSet {
    PtrS<Collider> pColliderA;
    PtrS<Collider> pColliderB;
    vector<Contact> vContactsA;
    vector<Contact> vContactsB;
    vector<PtrS<IEnergyElement>> vpEnergyElements;
    vector<PtrS<IConstraintSet>> vpConstraints;
    const void* pUserDataA;
    const void* pUserDataB;
  };

  using ContactStorage =
      unordered_map<pair<Collider*, Collider*>, PtrS<ContactSet>, XorHasher>;

  using ColliderFilter = unordered_set<pair<Collider*, Collider*>, XorHasher>;

 public:
  BC_Collisions(Simulable* pSimulable,
                const vector<PtrS<Collider>>& vColliders);

 public:
  inline virtual string Name() const { return "Collisions"; };

 public:
  void Init() override;
  void Update() override;

 private:
  void DetectContacts(const PtrS<Collider_SphereCloud>& pColliderA,
                      const PtrS<Collider_SphereCloud>& pColliderB,
                      vector<Contact>& vContactsA,
                      vector<Contact>& vContactsB);

  void DetectContacts(const PtrS<Collider_DistanceField>& pColliderA,
                      const PtrS<Collider_SphereCloud>& pColliderB,
                      vector<Contact>& vContactsA,
                      vector<Contact>& vContactsB);

  void GenerateConstraints(ContactSet& rContactSet);

 public:
  vector<PtrS<Collider>> Colliders() const;
  LayerMaskMatrix& CollisionMatrix();
  void EnableCollision(const PtrS<Collider>& pColliderA,
                       const PtrS<Collider>& pColliderB);
  void DisableCollision(const PtrS<Collider>& pColliderA,
                        const PtrS<Collider>& pColliderB);
  bool AbleToCollide(const PtrS<Collider>& pColliderA,
                     const PtrS<Collider>& pColliderB);

 public:
  ContactSet* Contacts(const PtrS<Collider>& pColliderA,
                       const PtrS<Collider>& pColliderB) const;
  vector<ContactSet*> Contacts(const PtrS<Collider>& pCollider) const;

 public:
  Signal<BC_Collisions*> OnDetectCollisions;
  Signal<BC_Collisions*> OnCollisionsDetected;

 private:
  vector<PtrS<Collider>> m_vColliders;
  LayerMaskMatrix m_CollisionMatrix;
  ContactStorage m_ContactStorage;
  ColliderFilter m_sFilteredColliders;
};
}  // namespace PhySim