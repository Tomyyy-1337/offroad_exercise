#ifndef __projects__PointcloudProject__mbbDriveToPoint_h__
#define __projects__PointcloudProject__mbbDriveToPoint_h__

#include "plugins/ib2c/tModule.h"

namespace finroc::PointcloudProject
{

class mbbDriveToPoint : public ib2c::tModule
{
public:
  tInput<float> in_goal_distance;
  tInput<float> in_goal_direction;
  tOutput<float> out_velocity;
  tOutput<float> out_steering;

  mbbDriveToPoint(core::tFrameworkElement *parent, const std::string &name = "DriveToPoint",
                  ib2c::tStimulationMode stimulation_mode = ib2c::tStimulationMode::AUTO);

protected:
  virtual ~mbbDriveToPoint();

private:
  virtual void OnStaticParameterChange() override;
  virtual void OnParameterChange() override;
  virtual bool ProcessTransferFunction() override;
  virtual ib2c::tTargetRating CalculateTargetRating() const override;
};

}  // namespace finroc::PointcloudProject

#endif