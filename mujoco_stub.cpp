#include <typeinfo>

namespace mujoco {
    class PlatformUIAdapter {};
    class GlfwAdapter {};
}

// Satisfy both runtime RTTI symbol checks
auto dummy_stub1 = &typeid(mujoco::PlatformUIAdapter);
auto dummy_stub2 = &typeid(mujoco::GlfwAdapter);
