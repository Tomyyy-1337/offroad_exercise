//----------------------------------------------------------------------
/*!\file    projects/CollisonAvoidanceProject/mbbDriveToGoal.cpp
 *
 * \author  tom passberg
 *
 * \date    2026-09-28
 *
 */
//----------------------------------------------------------------------
#include "projects/CollisonAvoidanceProject/mbbDriveToGoal.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace
{
constexpr float kGoalTolerance = 0.05f;
constexpr float kMaximumVelocity = 1.0f;
constexpr float kMaximumSteeringAngle = 0.8f;
}

namespace finroc
{
namespace CollisonAvoidanceProject
{

runtime_construction::tStandardCreateModuleAction<mbbDriveToGoal> cCREATE_ACTION_FOR_MBB_DRIVETOGOAL("DriveToGoal");

mbbDriveToGoal::mbbDriveToGoal(core::tFrameworkElement *parent, const std::string &name,
                               ib2c::tStimulationMode stimulation_mode) :
  tModule(parent, name, stimulation_mode, false)
{}

mbbDriveToGoal::~mbbDriveToGoal()
{}


void mbbDriveToGoal::OnStaticParameterChange()
{
  tModule::OnStaticParameterChange();
}

void mbbDriveToGoal::OnParameterChange()
{
  tModule::OnParameterChange();
}

bool mbbDriveToGoal::ProcessTransferFunction()
{
  if (!in_goal_distance.IsConnected() || !in_goal_direction.IsConnected())
  {
    out_velocity.Publish(0.0f);
    out_steering.Publish(0.0f);
    return false;
  }

  const float goal_distance = std::max(0.0f, in_goal_distance.Get());
  const float goal_direction = in_goal_direction.Get();

  const float velocity = std::min(kMaximumVelocity, goal_distance);

  const float steering = -std::clamp(goal_direction,
                                     -kMaximumSteeringAngle,
                                     kMaximumSteeringAngle);

  out_velocity.Publish(velocity);
  out_steering.Publish(steering);
  return true;
}

ib2c::tTargetRating mbbDriveToGoal::CalculateTargetRating() const
{
  if (in_goal_distance.Get() <= kGoalTolerance)
  {
    return 0.0f;
  }

  return 1.0f;
}

}
}
