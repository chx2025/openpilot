#include <cassert>
#include <stdexcept>

#include "sunnypilot/hardware/profile.h"


int main() {
#ifdef SUNNYPILOT_HARDWARE_PROFILE_C3XL
  assert(sunnypilot::hardware::resolve_internal_panda_type(0U) == 9U);
  assert(sunnypilot::hardware::resolve_internal_panda_type(9U) == 9U);

  bool rejected = false;
  try {
    sunnypilot::hardware::resolve_internal_panda_type(10U);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  assert(rejected);
#else
  assert(sunnypilot::hardware::resolve_internal_panda_type(0U) == 0U);
  assert(sunnypilot::hardware::resolve_internal_panda_type(9U) == 9U);
#endif

  return 0;
}
