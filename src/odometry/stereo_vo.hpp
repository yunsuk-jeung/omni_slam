#pragma once

#include "odometry/odometry.hpp"

namespace omni_slam {

class StereoVO : public Odometry {
public:
  StereoVO() = default;

  bool Initialize() override;
  void Run() override;
  void Shutdown() override;
};

}  // namespace omni_slam
