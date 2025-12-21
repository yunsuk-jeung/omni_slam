#pragma once

namespace omni_slam {

class OdometryPipeline {
public:
  virtual ~OdometryPipeline() = default;

  virtual bool Initialize() = 0;
  virtual void Run()        = 0;
  virtual void Shutdown()   = 0;
};

}  //namespace omni_slam
