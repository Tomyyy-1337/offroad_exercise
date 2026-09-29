#ifndef __projects__PointcloudProject__mbbAvoidObstacles_h__
#define __projects__PointcloudProject__mbbAvoidObstacles_h__

#include "plugins/ib2c/tModule.h"

#include <array>

namespace finroc::PointcloudProject
{

class mbbAvoidObstacles : public ib2c::tModule
{
public:
  tInput<float> in_next_obstacle_distance_center;
  tInput<float> in_next_obstacle_distance_left_edge;
  tInput<float> in_next_obstacle_distance_right_edge;
  tOutput<float> out_steering;

  mbbAvoidObstacles(core::tFrameworkElement *parent, const std::string &name = "AvoidObstacles",
                    ib2c::tStimulationMode stimulation_mode = ib2c::tStimulationMode::AUTO);

protected:
  virtual ~mbbAvoidObstacles();

private:
  virtual void OnStaticParameterChange() override;
  virtual void OnParameterChange() override;
  virtual bool ProcessTransferFunction() override;
  virtual ib2c::tTargetRating CalculateTargetRating() const override;

  std::array<float, 5> steering_history{};
  std::size_t steering_history_index = 0;
  std::size_t steering_history_count = 0;
};

}  // namespace finroc::PointcloudProject

#endif