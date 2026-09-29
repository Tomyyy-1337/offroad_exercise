#include "projects/PointcloudProject/mbbAvoidObstacles.h"

#include <algorithm>

namespace
{
constexpr float kSensorRange = 0.20f;
constexpr float kAvoidanceDistance = 0.20f;
constexpr float kSharpTurnDistance = 0.10f;
constexpr float kMaximumSteeringAngle = 0.8f;
constexpr float kAvoidanceGain = 2.5f;
constexpr float kFrontAvoidanceGain = 2.0f;

float Threat(float distance)
{
  return std::clamp((kAvoidanceDistance - distance) / kAvoidanceDistance, 0.0f, 1.0f);
}

float Clearance(float distance)
{
  return std::min(distance, kSensorRange);
}
}

namespace finroc::PointcloudProject
{

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mbbAvoidObstacles> cCREATE_ACTION_FOR_MBB_AVOID_OBSTACLES("AvoidObstacles");
#endif

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
  const float left_distance = in_next_obstacle_distance_left_edge.Get();
  const float right_distance = in_next_obstacle_distance_right_edge.Get();
  const float front_threat = Threat(in_next_obstacle_distance_center.Get());
  const float left_threat = Threat(left_distance);
  const float right_threat = Threat(right_distance);
  float avoidance = left_threat - right_threat;

  if (left_threat > 0.0f && right_threat > 0.0f)
  {
    const float preferred_side = Clearance(right_distance) >= Clearance(left_distance) ? 1.0f : -1.0f;
    avoidance = preferred_side * std::min(left_threat, right_threat);

    if (left_distance < kSharpTurnDistance && right_distance < kSharpTurnDistance)
    {
      avoidance = preferred_side;
    }
  }

  if (front_threat > 0.0f)
  {
    const float preferred_side = Clearance(right_distance) >= Clearance(left_distance) ? 1.0f : -1.0f;
    avoidance += preferred_side * kFrontAvoidanceGain * front_threat;
  }

  const float steering = std::clamp(kMaximumSteeringAngle * kAvoidanceGain * avoidance,
                                    -kMaximumSteeringAngle, kMaximumSteeringAngle);
  steering_history[steering_history_index] = steering;
  steering_history_index = (steering_history_index + 1) % steering_history.size();
  steering_history_count = std::min(steering_history_count + 1, steering_history.size());

  float smoothed_steering = 0.0f;
  for (std::size_t index = 0; index < steering_history_count; index++)
  {
    smoothed_steering += steering_history[index];
  }
  out_steering.Publish(smoothed_steering / static_cast<float>(steering_history_count));
  return true;
}

ib2c::tTargetRating mbbAvoidObstacles::CalculateTargetRating() const
{
  return std::max({Threat(in_next_obstacle_distance_center.Get()),
                   Threat(in_next_obstacle_distance_left_edge.Get()),
                   Threat(in_next_obstacle_distance_right_edge.Get())}) > 0.0f ? 1.0f : 0.0f;
}

}  // namespace finroc::PointcloudProject