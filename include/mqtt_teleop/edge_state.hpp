#pragma once

namespace mqtt_teleop {

// edge_state.py + STOPPING (fsm.md の stopping に対応)
enum class EdgeState {
  INIT,
  STARTING,
  REGISTRATION,
  REGISTERED,
  ACQUIRED,
  RUNNING,
  STOPPING,
  EXITING,
};

inline const char * to_string(EdgeState s)
{
  switch (s) {
    case EdgeState::INIT: return "INIT";
    case EdgeState::STARTING: return "STARTING";
    case EdgeState::REGISTRATION: return "REGISTRATION";
    case EdgeState::REGISTERED: return "REGISTERED";
    case EdgeState::ACQUIRED: return "ACQUIRED";
    case EdgeState::RUNNING: return "RUNNING";
    case EdgeState::STOPPING: return "STOPPING";
    case EdgeState::EXITING: return "EXITING";
  }
  return "?";
}

}  // namespace mqtt_teleop
