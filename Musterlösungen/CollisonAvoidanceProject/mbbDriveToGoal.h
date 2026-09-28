//----------------------------------------------------------------------
/*!\file    projects/CollisonAvoidanceProject/mbbDriveToGoal.h
 *
 * \author  tom passberg
 *
 * \date    2026-09-28
 *
 * \brief Contains mbbDriveToGoal
 *
 * \b mbbDriveToGoal
 *
 * Drives towards the goal until. Ignores obstacles. Active while goal is not reached. 
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__CollisonAvoidanceProject__mbbDriveToGoal_h__
#define __projects__CollisonAvoidanceProject__mbbDriveToGoal_h__

#include "plugins/ib2c/tModule.h"

namespace finroc
{
namespace CollisonAvoidanceProject
{

class mbbDriveToGoal : public ib2c::tModule
{

public:
  tInput<float> in_goal_distance;
  tInput<float> in_goal_direction;

  tOutput<float> out_velocity;
  tOutput<float> out_steering;

public:

  mbbDriveToGoal(core::tFrameworkElement *parent, const std::string &name = "DriveToGoal",
                 ib2c::tStimulationMode stimulation_mode = ib2c::tStimulationMode::AUTO);


protected:
  virtual ~mbbDriveToGoal();

private:
  virtual void OnStaticParameterChange() override;   

  virtual void OnParameterChange() override;   

  virtual bool ProcessTransferFunction() override;

  virtual ib2c::tTargetRating CalculateTargetRating() const override;

};

}
}



#endif
