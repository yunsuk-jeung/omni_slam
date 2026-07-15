#pragma once

#include <string>

namespace omni_slam {

class Odometry {
 public:
  virtual ~Odometry() = default;

  virtual bool setup(const std::string& config_path) = 0;
  virtual void run()                                 = 0;
  virtual void shutdown()                            = 0;
};

}  // namespace omni_slam
