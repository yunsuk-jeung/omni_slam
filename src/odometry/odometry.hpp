#pragma once

#include <string>

namespace omni_slam {

class Odometry {
public:
  virtual ~Odometry() = default;

  virtual bool Initialize(const std::string& config_path) = 0;
  virtual void Run()                                      = 0;
  virtual void Shutdown()                                 = 0;
};

}  // namespace omni_slam
