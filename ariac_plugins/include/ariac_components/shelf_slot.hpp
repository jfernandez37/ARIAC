#ifndef ARIAC_COMPONENTS_SHELF_SLOT_HPP_
#define ARIAC_COMPONENTS_SHELF_SLOT_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <gz/math/Pose3.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct ShelfSlot
  {
    // --- Kit Tray Shelf Configuration ---
    static constexpr double KT_X_START = -0.45;
    static constexpr double KT_X_SPACING = 0.30;
    static constexpr double KT_Z_TOP = 0.775;
    static constexpr double KT_Z_SPACING = 0.325;
    static constexpr double KT_Y = 0.0;

    inline static const std::vector<gz::math::Pose3d> KIT_TRAY_SHELF_SLOTS = {
      // Top row (z = 0.775)
      gz::math::Pose3d(KT_X_START + 0*KT_X_SPACING, KT_Y, KT_Z_TOP - 0*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 1*KT_X_SPACING, KT_Y, KT_Z_TOP - 0*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 2*KT_X_SPACING, KT_Y, KT_Z_TOP - 0*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 3*KT_X_SPACING, KT_Y, KT_Z_TOP - 0*KT_Z_SPACING, 0.0, 0.0, 0.0),
      // Middle row (z = 0.450)
      gz::math::Pose3d(KT_X_START + 0*KT_X_SPACING, KT_Y, KT_Z_TOP - 1*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 1*KT_X_SPACING, KT_Y, KT_Z_TOP - 1*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 2*KT_X_SPACING, KT_Y, KT_Z_TOP - 1*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 3*KT_X_SPACING, KT_Y, KT_Z_TOP - 1*KT_Z_SPACING, 0.0, 0.0, 0.0),
      // Bottom row (z = 0.125)
      gz::math::Pose3d(KT_X_START + 0*KT_X_SPACING, KT_Y, KT_Z_TOP - 2*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 1*KT_X_SPACING, KT_Y, KT_Z_TOP - 2*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 2*KT_X_SPACING, KT_Y, KT_Z_TOP - 2*KT_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(KT_X_START + 3*KT_X_SPACING, KT_Y, KT_Z_TOP - 2*KT_Z_SPACING, 0.0, 0.0, 0.0),
    };

    // --- High Priority Shelf Configuration ---
    static constexpr double HP_X_START = -0.15;
    static constexpr double HP_X_SPACING = 0.30;
    static constexpr double HP_Z_TOP = 0.775;
    static constexpr double HP_Z_SPACING = 0.325;
    static constexpr double HP_Y = 0.0;

    inline static const std::vector<gz::math::Pose3d> HIGH_PRIO_SHELF_SLOTS = {
      // Top row (z = 0.775)
      gz::math::Pose3d(HP_X_START + 0*HP_X_SPACING, HP_Y, HP_Z_TOP - 0*HP_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(HP_X_START + 1*HP_X_SPACING, HP_Y, HP_Z_TOP - 0*HP_Z_SPACING, 0.0, 0.0, 0.0),
      // Middle row (z = 0.450)
      gz::math::Pose3d(HP_X_START + 0*HP_X_SPACING, HP_Y, HP_Z_TOP - 1*HP_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(HP_X_START + 1*HP_X_SPACING, HP_Y, HP_Z_TOP - 1*HP_Z_SPACING, 0.0, 0.0, 0.0),
      // Bottom row (z = 0.125)
      gz::math::Pose3d(HP_X_START + 0*HP_X_SPACING, HP_Y, HP_Z_TOP - 2*HP_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(HP_X_START + 1*HP_X_SPACING, HP_Y, HP_Z_TOP - 2*HP_Z_SPACING, 0.0, 0.0, 0.0)
    };

    // --- Module Shelf Configuration ---
    static constexpr double MOD_X_START = -0.525;
    static constexpr double MOD_X_SPACING = 0.21;
    static constexpr double MOD_Z_TOP = 0.575;
    static constexpr double MOD_Z_SPACING = 0.225;
    static constexpr double MOD_Y = 0.0;

    inline static const std::vector<gz::math::Pose3d> MODULE_SHELF_SLOTS = {
      // Top row (z = 0.575)
      gz::math::Pose3d(MOD_X_START + 0*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 0*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 1*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 0*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 2*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 0*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 3*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 0*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 4*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 0*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 5*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 0*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      // Middle row (z = 0.350)
      gz::math::Pose3d(MOD_X_START + 0*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 1*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 1*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 1*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 2*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 1*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 3*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 1*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 4*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 1*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 5*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 1*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      // Bottom row (z = 0.125)
      gz::math::Pose3d(MOD_X_START + 0*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 2*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 1*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 2*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 2*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 2*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 3*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 2*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 4*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 2*MOD_Z_SPACING, 0.0, 0.0, 0.0),
      gz::math::Pose3d(MOD_X_START + 5*MOD_X_SPACING, MOD_Y, MOD_Z_TOP - 2*MOD_Z_SPACING, 0.0, 0.0, 0.0),
    };

    int index = 0;

    bool operator==(const ShelfSlot &_other) const;

    static bool equal(const ShelfSlot &_a, const ShelfSlot &_b);
  };

  namespace serializers
  {
    class ShelfSlotSerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const ShelfSlot &_slot);
        static std::istream &Deserialize(std::istream &_in, ShelfSlot &_slot);
    };
  }
}

namespace gz::sim::components
{
  using ShelfSlot = Component<ariac_components::ShelfSlot,
                              class ShelfSlotTag,
                              ariac_components::serializers::ShelfSlotSerializer>;
}

#endif // ARIAC_COMPONENTS_SHELF_SLOT_HPP_