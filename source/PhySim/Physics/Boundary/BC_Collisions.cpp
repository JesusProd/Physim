//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_Collisions.h>
#include <PhySim/Physics/Colliders/Collider.h>
#include <PhySim/Physics/Colliders/Collider_SphereCloud.h>
#include <PhySim/Physics/Elements/ConstraintSet_NodeNodeColl.h>
#include <PhySim/Physics/Elements/EnergyElement_SoftConstraint.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
BC_Collisions::BC_Collisions(Simulable* pSimulable,
                             const vector<PtrS<Collider>>& vColliders)
    : BCondition(pSimulable), m_vColliders(vColliders) {}

void BC_Collisions::Init() {
  // Initialize contact storage.
  m_ContactStorage.clear();

  for (int i = 0; i < m_vColliders.size(); ++i)
    for (int j = i; j < m_vColliders.size(); ++j) {
      const PtrS<Collider>& pColliderA = m_vColliders[i];
      const PtrS<Collider>& pColliderB = m_vColliders[j];

      auto it = m_ContactStorage.find({pColliderA.get(), pColliderB.get()});
      if (it != m_ContactStorage.end())
        continue;

      PtrS<ContactSet> pContactSet = NewS<ContactSet>();
      pContactSet->pColliderA = pColliderA;
      pContactSet->pColliderB = pColliderB;

      m_ContactStorage[{pColliderA.get(), pColliderB.get()}] = pContactSet;
      m_ContactStorage[{pColliderB.get(), pColliderA.get()}] = pContactSet;
    }
}

void BC_Collisions::Update() {
  IOUtils::logTrace(Verbosity::V1_Default, "\n[TRACE] Detecting collisions...");

  // Signal that collision detection is about to begin.
  OnDetectCollisions.Emit(this);

  // Clear all contacts and associated constraints/energies.
  // TODO: It's probably a better idea performance-wise to persist the energies
  // and constraints for those contacts that happen across frames, rather than
  // creating them all over again.
  m_vpCon.clear();
  m_vpEne.clear();

  for (auto itContacts : m_ContactStorage) {
    itContacts.second->vContactsA.clear();
    itContacts.second->vContactsB.clear();
    itContacts.second->vpConstraints.clear();
    itContacts.second->vpEnergyElements.clear();
  }

  // Prepare all colliders for collision detection.
  for (auto& pCollider : m_vColliders)
    pCollider->Update();

  // For all pairs of colliders, as long as they are able to collide
  // (either by collider type or by mask), check potential intersections.
  for (int i = 0; i < m_vColliders.size(); ++i)
    for (int j = i; j < m_vColliders.size(); ++j) {
      if (AbleToCollide(m_vColliders[i], m_vColliders[j]) == false)
        continue;

      ColliderType TypeA = m_vColliders[i]->Type();
      ColliderType TypeB = m_vColliders[j]->Type();
      ContactSet& rContactSet = *Contacts(m_vColliders[i], m_vColliders[j]);

      if (TypeA == ColliderType_SphereCloud &&
          TypeB == ColliderType_SphereCloud) {
        DetectContacts(
            dynamic_pointer_cast<Collider_SphereCloud>(m_vColliders[i]),
            dynamic_pointer_cast<Collider_SphereCloud>(m_vColliders[j]),
            rContactSet.vContactsA, rContactSet.vContactsB);
      } else {
        IOUtils::logTrace(
            Verbosity::V3_HardDebug, "Unsupported contact detection between %s and %s",
            m_vColliders[i]->Name().c_str(), m_vColliders[j]->Name().c_str());
      }
    }

  IOUtils::logTrace(Verbosity::V1_Default, "\n[TRACE] Generating contacts...");

  // After generating all contacts, iterate through all the contacts
  // and generate the corresponding constraints and energy elements.
  for (int i = 0; i < m_vColliders.size(); ++i)
    for (int j = i; j < m_vColliders.size(); ++j) {
      if (AbleToCollide(m_vColliders[i], m_vColliders[j]) == false)
        continue;

      ContactSet& rContactSet = *Contacts(m_vColliders[i], m_vColliders[j]);
      GenerateConstraints(rContactSet);
      m_vpCon.insert(m_vpCon.end(), rContactSet.vpConstraints.begin(),
                     rContactSet.vpConstraints.end());
      m_vpEne.insert(m_vpEne.end(), rContactSet.vpEnergyElements.begin(),
                     rContactSet.vpEnergyElements.end());
    }

  IOUtils::logTrace(Verbosity::V1_Default, "\n[TRACE] Generated %i contact constraints",
                    m_vpCon.size());

  // Signal that collision detection just finished.
  OnCollisionsDetected.Emit(this);
}

void BC_Collisions::DetectContacts(const PtrS<Collider_SphereCloud>& pColliderA,
                                   const PtrS<Collider_SphereCloud>& pColliderB,
                                   vector<Contact>& vContactsA,
                                   vector<Contact>& vContactsB) {
  const BVHTree& rTreeA = pColliderA->Tree();
  const BVHTree& rTreeB = pColliderB->Tree();

  const vector<Collider_SphereCloud::Point>& vpPointsA = pColliderA->Points();
  const vector<Collider_SphereCloud::Point>& vpPointsB = pColliderB->Points();

  const bool IsSelfCollision = pColliderA == pColliderB;

  // Broad-Phase: Use acceleration structure to find all potential contact
  // candidates.
  BVHTree::IntersectionResults vCandidates;
  rTreeA.Intersect(rTreeB, vCandidates);

  vContactsA.reserve(vContactsA.size() + vCandidates.size());
  vContactsB.reserve(vContactsB.size() + vCandidates.size());

  for (auto& Candidate : vCandidates) {
    // Narrow-Phase: Check if they are really in contact.
    const Collider_SphereCloud::Point& pPointA = vpPointsA[Candidate.first];
    const Collider_SphereCloud::Point& pPointB = vpPointsB[Candidate.second];

    VectorXd PositionA = pPointA.Position->InterpolateValue(Tag::Tag_Position_X);
    VectorXd PositionB = pPointB.Position->InterpolateValue(Tag::Tag_Position_X);
    Real RadiusA = pPointA.Radius;
    Real RadiusB = pPointB.Radius;
    Real TotalRadius = RadiusA + RadiusB;

    Real Distance =
        (PositionA - PositionB).squaredNorm() - TotalRadius * TotalRadius;
    if (Distance >= 0.0)
      continue;

    // Special case: If self-colliding, make sure to prevent generating contacts
    // with adjacent neighbors.
    if (IsSelfCollision) {
      VectorXd RestPositionA =
          pPointA.Position->InterpolateValue(Tag::Tag_Position_0);
      VectorXd RestPositionB =
          pPointB.Position->InterpolateValue(Tag::Tag_Position_0);

      Real RestDistance = (RestPositionA - RestPositionB).squaredNorm() -
                          TotalRadius * TotalRadius;
      if (RestDistance < 0.0)
        continue;
    }

    // Store contacts.
    Contact ContactA;
    Contact ContactB;
    ContactA.Index = Candidate.first;
    ContactA.Point = 0.5 * (PositionA + PositionB);
    ContactA.Normal = (PositionB - PositionA).normalized();
    ContactA.pData = &pPointA;

    ContactB.Index = Candidate.second;
    ContactB.Point = ContactA.Point;
    ContactB.Normal = -ContactA.Normal;
    ContactB.pData = &pPointB;

    vContactsA.emplace_back(ContactA);
    vContactsB.emplace_back(ContactB);
  }
}

void BC_Collisions::DetectContacts(
    const PtrS<Collider_DistanceField>& pColliderA,
    const PtrS<Collider_SphereCloud>& pColliderB,
    vector<Contact>& vContactsA,
    vector<Contact>& vContactsB) {
  // TODO: Not implemented yet.
}

void BC_Collisions::GenerateConstraints(ContactSet& rContactSet) {
  assert(rContactSet.vContactsA.size() == rContactSet.vContactsB.size() &&
         "Number of contacts mismatch!");

  size_t N = rContactSet.vContactsA.size();
  ColliderType TypeA = rContactSet.pColliderA->Type();
  ColliderType TypeB = rContactSet.pColliderB->Type();

  rContactSet.vpConstraints.reserve(rContactSet.vpConstraints.size() + N);
  rContactSet.vpEnergyElements.reserve(rContactSet.vpEnergyElements.size() + N);

  if (TypeA == ColliderType_SphereCloud && TypeB == ColliderType_SphereCloud) {
    PtrS<Collider_SphereCloud> pSphereCloudA =
        dynamic_pointer_cast<Collider_SphereCloud>(rContactSet.pColliderA);
    PtrS<Collider_SphereCloud> pSphereCloudB =
        dynamic_pointer_cast<Collider_SphereCloud>(rContactSet.pColliderB);

    const vector<Collider_SphereCloud::Point>& vpPointsA =
        pSphereCloudA->Points();
    const vector<Collider_SphereCloud::Point>& vpPointsB =
        pSphereCloudB->Points();

    for (size_t i = 0; i < N; ++i) {
      const Collider_SphereCloud::Point& pPointA =
          vpPointsA[rContactSet.vContactsA[i].Index];
      const Collider_SphereCloud::Point& pPointB =
          vpPointsB[rContactSet.vContactsB[i].Index];

      Real RadiusA = pPointA.Radius;
      Real RadiusB = pPointB.Radius;
      Real TotalRadius = RadiusA + RadiusB;
      Real ApproxSurfaceArea = M_PI * TotalRadius * TotalRadius;

      PtrS<ConstraintSet_NodeNodeColl> pConstraint;
      if (rContactSet.pColliderA->IsSimulated() &&
          rContactSet.pColliderB
              ->IsSimulated())  // Two-way (both colliders simulated)
        pConstraint = NewS<ConstraintSet_NodeNodeColl>(
            m_pModel, true, pPointA.Position, pPointB.Position,
            RadiusA + RadiusB, true);
      else if (rContactSet.pColliderA
                   ->IsSimulated())  // One-way (collider A is simulated)
        pConstraint = NewS<ConstraintSet_NodeNodeColl>(
            m_pModel, true, pPointA.Position, pPointB.Position,
            RadiusA + RadiusB, false);
      else if (rContactSet.pColliderB
                   ->IsSimulated())  // One-way (collider B is simulated)
        pConstraint = NewS<ConstraintSet_NodeNodeColl>(
            m_pModel, true, pPointB.Position, pPointA.Position,
            RadiusA + RadiusB, false);

      if (pConstraint != NULL) {
        PtrS<EnergyElement_SoftConstraint> pEnergy =
            NewS<EnergyElement_SoftConstraint>(
                m_pModel, pConstraint, 1e9 * ApproxSurfaceArea,
                true);  // TODO: Expose stiffness.
        rContactSet.vpConstraints.push_back(pConstraint);
        rContactSet.vpEnergyElements.push_back(pEnergy);
      }
    }
  }
}

vector<PtrS<Collider>> BC_Collisions::Colliders() const {
  return m_vColliders;
}

LayerMaskMatrix& BC_Collisions::CollisionMatrix() {
  return m_CollisionMatrix;
}

bool BC_Collisions::AbleToCollide(const PtrS<Collider>& pColliderA,
                                  const PtrS<Collider>& pColliderB) {
  Layer CollisionLayerA = pColliderA->CollisionLayer();
  Layer CollisionLayerB = pColliderA->CollisionLayer();

  return m_CollisionMatrix(CollisionLayerA, CollisionLayerB) &&
         m_sFilteredColliders.find({pColliderA.get(), pColliderB.get()}) ==
             m_sFilteredColliders.end();
}

void BC_Collisions::EnableCollision(const PtrS<Collider>& pColliderA,
                                    const PtrS<Collider>& pColliderB) {
  auto it = m_sFilteredColliders.find({pColliderA.get(), pColliderB.get()});
  if (it != m_sFilteredColliders.end())
    m_sFilteredColliders.erase(it);
}

void BC_Collisions::DisableCollision(const PtrS<Collider>& pColliderA,
                                     const PtrS<Collider>& pColliderB) {
  auto it = m_sFilteredColliders.find({pColliderA.get(), pColliderB.get()});
  if (it == m_sFilteredColliders.end())
    m_sFilteredColliders.emplace(pColliderA.get(), pColliderB.get());
}

BC_Collisions::ContactSet* BC_Collisions::Contacts(
    const PtrS<Collider>& pColliderA,
    const PtrS<Collider>& pColliderB) const {
  auto it = m_ContactStorage.find({pColliderA.get(), pColliderB.get()});
  assert(it != m_ContactStorage.end() && "The given Collider pair is invalid.");
  return it->second.get();
}

vector<BC_Collisions::ContactSet*> BC_Collisions::Contacts(
    const PtrS<Collider>& pCollider) const {
  vector<ContactSet*> vContactSets;
  vContactSets.reserve(m_vColliders.size());

  for (int i = 0; i < m_vColliders.size(); ++i)
    vContactSets.push_back(Contacts(pCollider, m_vColliders[i]));

  return vContactSets;
}

}  // namespace PhySim