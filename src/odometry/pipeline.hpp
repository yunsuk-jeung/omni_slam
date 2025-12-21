#pragma once

namespace omni_slam {

class OdometryPipeline {
public:
  virtual ~OdometryPipeline() = default;

  virtual bool initialize() = 0;
  virtual void run()        = 0;
  virtual void shutdown()   = 0;
};

}  //namespace omni_slam
