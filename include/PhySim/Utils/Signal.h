//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <cassert>
#include <functional>
#include <unordered_map>

namespace PhySim {
using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////

using Slot = size_t;

/////////////////////////////////////////////////////////////////////////////////////////////

template <typename... Args>
class Signal {
 public:
  Signal() : m_CurrentSlotId(0) {}

  Signal(const Signal& other) = default;
  Signal& operator=(const Signal& other) = default;

  Signal(Signal&& other) = default;
  Signal& operator=(Signal&& other) = default;

 public:
  Slot Connect(const function<void(Args...)>& fFunc) {
    size_t id = m_CurrentSlotId++;
    m_mpSlots[id] = func;
    return id;
  }

  template <typename T>
  Slot Connect(T* pInstance, void (T::*pFunc)(Args...)) {
    return Connect([=](Args...&& args) { (pInstance->*pFunc)(args...); });
  }

  template <typename T>
  Slot Connect(T* pInstance, void (T::*pFunc)(Args...) const) {
    return Connect([=](Args...&& args) { (pInstance->*pFunc)(args...); });
  }

  void Disconnect(const Slot& slot) {
    auto it = m_mpSlots.find(slot);
    assert(it != m_mpSlots.end() && "Slot not found.");
    m_mpSlots.erase(it);
  }

  void DisconnectAll() { m_mpSlots.clear(); }

  void Emit(Args... args) const {
    for (const auto& it : m_mpSlots)
      it.second(forward<Args>(args)...);
  }

 public:
  size_t NumConnections() const { return m_mpSlots.count(); }

 public:
  size_t m_CurrentSlotId;
  unordered_map<size_t, function<void(Args...)>> m_mpSlots;
};

/////////////////////////////////////////////////////////////////////////////////////////////
}  // namespace PhySim
