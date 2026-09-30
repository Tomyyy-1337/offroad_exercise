//----------------------------------------------------------------------
/*!\file    projects/CollisonAvoidanceProject/mbbAvoidObstacles.cpp
 *
 * \brief Obstacle avoidance steering behavior.
 */
//----------------------------------------------------------------------
#include "projects/CollisonAvoidanceProject/mbbAvoidObstacles.h"

#include <algorithm>

namespace
{
constexpr float kSensorRange = 0.50f;
constexpr float kAvoidanceDistance = 0.2f;
constexpr float kMaximumSteeringAngle = 0.8f;
constexpr float kAvoidanceGain = 2.5f;
constexpr float kFrontAvoidanceGain = 2.0f;

float Threat(float distance)
{
  // The simulation publishes zero when a ray does not hit an obstacle.
  if (distance <= 0.0f)
  {
    return 0.0f;
  }

  return std::clamp((kAvoidanceDistance - distance) / kAvoidanceDistance, 0.0f, 1.0f);
}

float Clearance(float distance)
{
  return distance > 0.0f ? std::min(distance, kSensorRange) : kSensorRange;
}
}

namespace finroc
{
namespace CollisonAvoidanceProject
{

runtime_construction::tStandardCreateModuleAction<mbbAvoidObstacles> cCREATE_ACTION_FOR_MBB_AVOID_OBSTACLES("AvoidObstacles");

mbbAvoidObstacles::mbbAvoidObstacles(core::tFrameworkElement *parent, const std::string &name,
                                     ib2c::tStimulationMode stimulation_mode) :
  tModule(parent, name, stimulation_mode, false)
{}

mbbAvoidObstacles::~mbbAvoidObstacles()
{}

void mbbAvoidObstacles::OnStaticParameterChange()
{
  tModule::OnStaticParameterChange();
}

void mbbAvoidObstacles::OnParameterChange()
{
  tModule::OnParameterChange();
}

bool mbbAvoidObstacles::ProcessTransferFunction()
{
    if (!in_next_obstacle_distance_center.IsConnected() ||
      !in_next_obstacle_distance_left_edge.IsConnected() ||
      !in_next_obstacle_distance_right_edge.IsConnected())
  {
    out_steering.Publish(0.0f);
    return false;
  }

  const float front_threat = Threat(in_next_obstacle_distance_center.Get());
  const float left_threat = Threat(in_next_obstacle_distance_left_edge.Get());
  const float right_threat = Threat(in_next_obstacle_distance_right_edge.Get());

  float avoidance = left_threat - right_threat;

  if (front_threat > 0.0f)
  {
    const float left_clearance = Clearance(in_next_obstacle_distance_left_edge.Get());
    const float right_clearance = Clearance(in_next_obstacle_distance_right_edge.Get());

    const float preferred_side = right_clearance >= left_clearance ? 1.0f : -1.0f;
    avoidance += preferred_side * kFrontAvoidanceGain * front_threat;
  }

  const float steering = std::clamp(kMaximumSteeringAngle * kAvoidanceGain * avoidance,
                                    -kMaximumSteeringAngle,
                                    kMaximumSteeringAngle);
  out_steering.Publish(steering);
  return true;
}

ib2c::tTargetRating mbbAvoidObstacles::CalculateTargetRating() const
{
  const float front_threat = Threat(in_next_obstacle_distance_center.Get());
  const float left_threat = Threat(in_next_obstacle_distance_left_edge.Get());
  const float right_threat = Threat(in_next_obstacle_distance_right_edge.Get());

  return std::max(front_threat, std::max(left_threat, right_threat)) > 0.0f ? 1.0f : 0.0f;
}

}
}
