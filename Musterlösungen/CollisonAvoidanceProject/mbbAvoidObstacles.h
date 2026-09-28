//----------------------------------------------------------------------
/*!\file    projects/CollisonAvoidanceProject/mbbAvoidObstacles.h
 *
 * \brief Contains mbbAvoidObstacles
 *
 * Generates a steering command to avoid obstacles detected in front of the
 * triangle. The module does not process or change the velocity.
 */
//----------------------------------------------------------------------
#ifndef __projects__CollisonAvoidanceProject__mbbAvoidObstacles_h__
#define __projects__CollisonAvoidanceProject__mbbAvoidObstacles_h__

#include "plugins/ib2c/tModule.h"

namespace finroc
{
namespace CollisonAvoidanceProject
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
};

}
}

#endif
