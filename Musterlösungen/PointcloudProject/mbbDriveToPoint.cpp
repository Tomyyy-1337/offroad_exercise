#include "projects/PointcloudProject/mbbDriveToPoint.h"

#include <algorithm>

namespace
{
constexpr float kGoalTolerance = 0.05f;
constexpr float kMaximumVelocity = 0.75f;
constexpr float kMaximumSteeringAngle = 0.4f;
}

namespace finroc::PointcloudProject
{

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mbbDriveToPoint> cCREATE_ACTION_FOR_MBB_DRIVE_TO_POINT("DriveToPoint");
#endif

mbbDriveToPoint::mbbDriveToPoint(core::tFrameworkElement *parent, const std::string &name,
                                 ib2c::tStimulationMode stimulation_mode) :
  tModule(parent, name, stimulation_mode, false)
{}

mbbDriveToPoint::~mbbDriveToPoint()
{}

void mbbDriveToPoint::OnStaticParameterChange()
{
  tModule::OnStaticParameterChange();
}

void mbbDriveToPoint::OnParameterChange()
{
  tModule::OnParameterChange();
}

bool mbbDriveToPoint::ProcessTransferFunction()
{
  const float goal_distance = std::max(0.0f, in_goal_distance.Get());

  if (goal_distance <= kGoalTolerance)
  {
    out_velocity.Publish(0.0f);
    out_steering.Publish(0.0f);
    return true;
  }

  out_velocity.Publish(std::min(kMaximumVelocity, goal_distance));
  out_steering.Publish(-std::clamp(in_goal_direction.Get(), -kMaximumSteeringAngle, kMaximumSteeringAngle));
  return true;
}

ib2c::tTargetRating mbbDriveToPoint::CalculateTargetRating() const
{
  return in_goal_distance.Get() > kGoalTolerance ? 1.0f : 0.0f;
}

}  // namespace finroc::PointcloudProject