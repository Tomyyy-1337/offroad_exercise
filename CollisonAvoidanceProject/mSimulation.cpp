//----------------------------------------------------------------------
/*!\file    projects/CollisonAvoidanceProject/mTriangleSimulation.cpp
 *
 * \author  tom passberg
 *
 * \date    2026-09-17
 *
 */
//----------------------------------------------------------------------
#include "projects/CollisonAvoidanceProject/mSimulation.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kSpeedScale = 0.01f;
constexpr float kWheelBase = 0.12f;
constexpr float kMaxSteeringAngle = 0.8f;
constexpr float kCameraZoom = 3.5f;
constexpr float kObstacleRadius = 0.09f;
constexpr float kTargetX = 3.00f;
constexpr float kTargetY = 0.50f;
constexpr float kTargetRadius = 0.05f;
constexpr float kStartX = 0.42f;
constexpr float kStartY = 0.5f;
constexpr float kStartOrientation = 0.0f;
constexpr float kTenDegreeOffset = 10.0f * kPi / 180.0f;
constexpr float kTwentyDegreeOffset = 20.0f * kPi / 180.0f;
constexpr float kTriangleLength = 0.04f;
constexpr float kObstacleRayMaxDistance = 5.0f * kTriangleLength;

struct tObstacle
{
  float x;
  float y;
};

float Dot(float ax, float ay, float bx, float by)
{
  return ax * bx + ay * by;
}

float DistanceSquared(float ax, float ay, float bx, float by)
{
  const float dx = ax - bx;
  const float dy = ay - by;
  return dx * dx + dy * dy;
}

bool PointInTriangle(float px, float py,
                     float ax, float ay,
                     float bx, float by,
                     float cx, float cy)
{
  const float v0x = cx - ax;
  const float v0y = cy - ay;
  const float v1x = bx - ax;
  const float v1y = by - ay;
  const float v2x = px - ax;
  const float v2y = py - ay;

  const float dot00 = Dot(v0x, v0y, v0x, v0y);
  const float dot01 = Dot(v0x, v0y, v1x, v1y);
  const float dot02 = Dot(v0x, v0y, v2x, v2y);
  const float dot11 = Dot(v1x, v1y, v1x, v1y);
  const float dot12 = Dot(v1x, v1y, v2x, v2y);

  const float denominator = dot00 * dot11 - dot01 * dot01;
  if (std::abs(denominator) < 1e-8f)
  {
    return false;
  }

  const float inverse_denominator = 1.0f / denominator;
  const float u = (dot11 * dot02 - dot01 * dot12) * inverse_denominator;
  const float v = (dot00 * dot12 - dot01 * dot02) * inverse_denominator;
  return u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f;
}

float ClampToSegment(float value, float minimum, float maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

bool SegmentIntersectsCircle(float ax, float ay,
                            float bx, float by,
                            float cx, float cy,
                            float radius)
{
  const float abx = bx - ax;
  const float aby = by - ay;
  const float acx = cx - ax;
  const float acy = cy - ay;

  const float length_squared = abx * abx + aby * aby;
  if (length_squared < 1e-8f)
  {
    return DistanceSquared(ax, ay, cx, cy) <= radius * radius;
  }

  const float projection = Dot(acx, acy, abx, aby) / length_squared;
  const float t = ClampToSegment(projection, 0.0f, 1.0f);
  const float closest_x = ax + t * abx;
  const float closest_y = ay + t * aby;
  return DistanceSquared(closest_x, closest_y, cx, cy) <= radius * radius;
}

bool TriangleIntersectsObstacle(float p1_x, float p1_y,
                                float p2_x, float p2_y,
                                float p3_x, float p3_y,
                                const tObstacle &obstacle,
                                float obstacle_radius)
{
  const float radius_squared = obstacle_radius * obstacle_radius;

  if (DistanceSquared(p1_x, p1_y, obstacle.x, obstacle.y) <= radius_squared ||
      DistanceSquared(p2_x, p2_y, obstacle.x, obstacle.y) <= radius_squared ||
      DistanceSquared(p3_x, p3_y, obstacle.x, obstacle.y) <= radius_squared)
  {
    return true;
  }

  if (PointInTriangle(obstacle.x, obstacle.y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y))
  {
    return true;
  }

  return SegmentIntersectsCircle(p1_x, p1_y, p2_x, p2_y, obstacle.x, obstacle.y, obstacle_radius) ||
         SegmentIntersectsCircle(p2_x, p2_y, p3_x, p3_y, obstacle.x, obstacle.y, obstacle_radius) ||
         SegmentIntersectsCircle(p3_x, p3_y, p1_x, p1_y, obstacle.x, obstacle.y, obstacle_radius);
}

struct tTriangleVertices
{
  float p1_x;
  float p1_y;
  float p2_x;
  float p2_y;
  float p3_x;
  float p3_y;
};

float NormalizeAngle(float angle);

struct tObstacleRayHit
{
  bool found;
  float distance;
  float hit_x;
  float hit_y;
};

tTriangleVertices CalculateTriangleVertices(float position_x, float position_y, float orientation)
{
  const float angle2 = orientation + 2.356f;
  const float angle3 = orientation - 2.356f;

  return {
    position_x + kTriangleLength * std::cos(orientation),
    position_y + kTriangleLength * std::sin(orientation),
    position_x + kTriangleLength * std::cos(angle2),
    position_y + kTriangleLength * std::sin(angle2),
    position_x + kTriangleLength * std::cos(angle3),
    position_y + kTriangleLength * std::sin(angle3)
  };
}

float CalculateGoalDistance(float position_x, float position_y)
{
  return std::sqrt(DistanceSquared(position_x, position_y, kTargetX, kTargetY));
}

float CalculateGoalDirection(float position_x, float position_y, float orientation)
{
  const float target_heading = std::atan2(kTargetY - position_y, kTargetX - position_x);
  return NormalizeAngle(target_heading - orientation);
}

constexpr std::array<tObstacle, 36> kObstacles = {{
  {0.24f, 0.18f},
  {0.41f, 0.73f},
  {0.53f, 0.29f},
  {0.67f, 0.58f},
  {0.79f, 0.14f},
  {0.95f, 0.82f},
  {1.07f, 0.37f},
  {1.21f, 0.66f},
  {1.34f, 0.23f},
  {1.47f, 0.52f},
  {1.62f, 0.11f},
  {1.75f, 0.71f},
  {1.88f, 0.34f},
  {2.01f, 0.86f},
  {2.14f, 0.48f},
  {2.27f, 0.17f},
  {2.38f, 0.63f},
  {2.52f, 0.28f},
  {2.64f, 0.76f},
  {2.78f, 0.41f},
  {2.88f, 0.95f},
  {2.90f, 0.14f},
  {3.08f, 0.88f},
  {3.18f, 0.19f},
  {3.36f, 0.73f},
  {2.48f, 0.16f},
  {2.66f, 0.82f},
  {2.86f, 0.28f},
  {3.06f, 0.74f},
  {3.28f, 0.12f},
  {3.50f, 0.66f},
  {3.72f, 0.34f},
  {3.94f, 0.88f},
  {4.16f, 0.20f},
  {4.38f, 0.58f},
  {4.60f, 0.42f}
}};

tObstacleRayHit FindNextObstacle(float front_x, float front_y, float orientation)
{
  const float direction_x = std::cos(orientation);
  const float direction_y = std::sin(orientation);

  tObstacleRayHit best_hit{false, 0.0f, 0.0f, 0.0f};
  float best_distance = 0.0f;

  for (const auto &obstacle : kObstacles)
  {
    const float ox = front_x - obstacle.x;
    const float oy = front_y - obstacle.y;

    const float b = 2.0f * Dot(ox, oy, direction_x, direction_y);
    const float c = ox * ox + oy * oy - kObstacleRadius * kObstacleRadius;
    const float discriminant = b * b - 4.0f * c;

    if (discriminant < 0.0f)
    {
      continue;
    }

    const float sqrt_discriminant = std::sqrt(discriminant);
    const float first_hit = (-b - sqrt_discriminant) * 0.5f;
    const float second_hit = (-b + sqrt_discriminant) * 0.5f;

    float distance = -1.0f;
    if (first_hit >= 0.0f)
    {
      distance = first_hit;
    }
    else if (second_hit >= 0.0f)
    {
      distance = second_hit;
    }

    if (distance < 0.0f)
    {
      continue;
    }

    if (distance > kObstacleRayMaxDistance)
    {
      continue;
    }

    if (!best_hit.found || distance < best_distance)
    {
      best_hit.found = true;
      best_distance = distance;
      best_hit.distance = distance;
      best_hit.hit_x = front_x + direction_x * distance;
      best_hit.hit_y = front_y + direction_y * distance;
    }
  }

  return best_hit;
}

float NormalizeAngle(float angle)
{
  while (angle > kPi)
  {
    angle -= kTwoPi;
  }

  while (angle < -kPi)
  {
    angle += kTwoPi;
  }

  return angle;
}

}

namespace finroc
{
namespace CollisonAvoidanceProject
{

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mSimulation> cCREATE_ACTION_FOR_M_SIMULATION("Simulation");
#endif

mSimulation::mSimulation(core::tFrameworkElement *parent, const std::string &name) :
  structure::tModule(parent, name),
  position_x(kStartX),
  position_y(kStartY),
  orientation(kStartOrientation),
  finished(false)
{}

mSimulation::~mSimulation()
{}

void mSimulation::OnStaticParameterChange()
{
}

void mSimulation::OnParameterChange()
{
}

void mSimulation::Update()
{
  bool collision = false;
  tTriangleVertices triangle = CalculateTriangleVertices(position_x, position_y, orientation);

  if (!finished)
  {
    float velocity = 0.0f;
    float steering = 0.0f;

    if (in_velocity.IsConnected())
    {
      velocity = in_velocity.Get();
    }

    if (in_steering.IsConnected())
    {
      steering = in_steering.Get() * -1.0f;
    }

    const float speed = velocity * kSpeedScale;
    const float steering_angle = std::clamp(steering, -kMaxSteeringAngle, kMaxSteeringAngle);

    const float previous_orientation = orientation;
    const bool has_motion = std::abs(speed) > 1e-6f;
    const bool has_turn = std::abs(steering_angle) > 1e-4f;

    if (has_motion && has_turn)
    {
      const float turn_rate = std::tan(steering_angle) / kWheelBase;
      const float orientation_delta = speed * turn_rate;
      const float next_orientation = NormalizeAngle(previous_orientation + orientation_delta);
      const float turn_radius = kWheelBase / std::tan(steering_angle);

      position_x += turn_radius * (std::sin(next_orientation) - std::sin(previous_orientation));
      position_y -= turn_radius * (std::cos(next_orientation) - std::cos(previous_orientation));

      orientation = next_orientation;
    }
    else if (has_motion)
    {
      position_x += speed * std::cos(previous_orientation);
      position_y += speed * std::sin(previous_orientation);
    }

    orientation = NormalizeAngle(orientation);
    triangle = CalculateTriangleVertices(position_x, position_y, orientation);

    for (const auto &obstacle : kObstacles)
    {
      if (TriangleIntersectsObstacle(triangle.p1_x, triangle.p1_y,
                                     triangle.p2_x, triangle.p2_y,
                                     triangle.p3_x, triangle.p3_y,
                                     obstacle, kObstacleRadius))
      {
        collision = true;
        break;
      }
    }

    const tObstacle goal{ kTargetX, kTargetY };
    if (!collision && TriangleIntersectsObstacle(triangle.p1_x, triangle.p1_y,
                                                 triangle.p2_x, triangle.p2_y,
                                                 triangle.p3_x, triangle.p3_y,
                                                 goal, kTargetRadius))
    {
      finished = true;
    }

    out_collision.Publish(collision);
    out_goal_distance.Publish(CalculateGoalDistance(position_x, position_y));
    out_goal_direction.Publish(CalculateGoalDirection(position_x, position_y, orientation));

    if (collision)
    {
      position_x = kStartX;
      position_y = kStartY;
      orientation = kStartOrientation;
      triangle = CalculateTriangleVertices(position_x, position_y, orientation);
    }
  }
  else
  {
    out_collision.Publish(false);
    out_goal_distance.Publish(0.0f);
    out_goal_direction.Publish(0.0f);
  }

  const tTriangleVertices rendered_triangle = CalculateTriangleVertices(position_x, position_y, orientation);
  const tObstacleRayHit next_obstacle = FindNextObstacle(rendered_triangle.p1_x, rendered_triangle.p1_y, orientation);
  const tObstacleRayHit next_obstacle_left_10 = FindNextObstacle(rendered_triangle.p1_x, rendered_triangle.p1_y, orientation + kTenDegreeOffset);
  const tObstacleRayHit next_obstacle_right_10 = FindNextObstacle(rendered_triangle.p1_x, rendered_triangle.p1_y, orientation - kTenDegreeOffset);
  const tObstacleRayHit next_obstacle_left_20 = FindNextObstacle(rendered_triangle.p1_x, rendered_triangle.p1_y, orientation + kTwentyDegreeOffset);
  const tObstacleRayHit next_obstacle_right_20 = FindNextObstacle(rendered_triangle.p1_x, rendered_triangle.p1_y, orientation - kTwentyDegreeOffset);

  out_next_obstacle_distance.Publish(next_obstacle.found ? next_obstacle.distance : 0.0f);
  out_next_obstacle_distance_left_10.Publish(next_obstacle_left_10.found ? next_obstacle_left_10.distance : 0.0f);
  out_next_obstacle_distance_right_10.Publish(next_obstacle_right_10.found ? next_obstacle_right_10.distance : 0.0f);
  out_next_obstacle_distance_left_20.Publish(next_obstacle_left_20.found ? next_obstacle_left_20.distance : 0.0f);
  out_next_obstacle_distance_right_20.Publish(next_obstacle_right_20.found ? next_obstacle_right_20.distance : 0.0f);

  auto canvas = out_visualization.GetUnusedBuffer();

  canvas->Clear();

  canvas->SetDefaultViewport(-0.5f, -0.5f, 2.0f, 2.0f);

  canvas->ResetTransformation();
  canvas->Translate(0.5f, 0.5f);
  canvas->Scale(kCameraZoom, kCameraZoom);
  canvas->Translate(-position_x, -position_y);

  canvas->SetColor(220, 80, 60);
  canvas->SetFill(true);

  for (const auto &obstacle : kObstacles)
  {
    canvas->DrawEllipsoid(obstacle.x, obstacle.y, kObstacleRadius * 2.0f, kObstacleRadius * 2.0f);
  }

  canvas->SetColor(255, 255, 0);
  canvas->SetFill(true);
  canvas->DrawEllipsoid(kTargetX, kTargetY, kTargetRadius * 2.0f, kTargetRadius * 2.0f);

  canvas->SetColor(255, 0, 0);
  canvas->SetFill(false);
  if (next_obstacle.found)
  {
    canvas->DrawLineSegment(rrlib::math::tVec2d(rendered_triangle.p1_x, rendered_triangle.p1_y),
                            rrlib::math::tVec2d(next_obstacle.hit_x, next_obstacle.hit_y));
  }

  canvas->SetColor(255, 0, 0);
  canvas->SetFill(false);
  if (next_obstacle_left_10.found)
  {
    canvas->DrawLineSegment(rrlib::math::tVec2d(rendered_triangle.p1_x, rendered_triangle.p1_y),
                            rrlib::math::tVec2d(next_obstacle_left_10.hit_x, next_obstacle_left_10.hit_y));
  }

  canvas->SetColor(255, 0, 0);
  canvas->SetFill(false);
  if (next_obstacle_right_10.found)
  {
    canvas->DrawLineSegment(rrlib::math::tVec2d(rendered_triangle.p1_x, rendered_triangle.p1_y),
                            rrlib::math::tVec2d(next_obstacle_right_10.hit_x, next_obstacle_right_10.hit_y));
  }

  canvas->SetColor(255, 0, 0);
  canvas->SetFill(false);
  if (next_obstacle_left_20.found)
  {
    canvas->DrawLineSegment(rrlib::math::tVec2d(rendered_triangle.p1_x, rendered_triangle.p1_y),
                            rrlib::math::tVec2d(next_obstacle_left_20.hit_x, next_obstacle_left_20.hit_y));
  }

  canvas->SetColor(255, 0, 0);
  canvas->SetFill(false);
  if (next_obstacle_right_20.found)
  {
    canvas->DrawLineSegment(rrlib::math::tVec2d(rendered_triangle.p1_x, rendered_triangle.p1_y),
                            rrlib::math::tVec2d(next_obstacle_right_20.hit_x, next_obstacle_right_20.hit_y));
  }

  canvas->SetColor(255, 255, 0);
  canvas->SetFill(false);
  canvas->DrawLineSegment(rrlib::math::tVec2d(rendered_triangle.p1_x, rendered_triangle.p1_y),
                          rrlib::math::tVec2d(kTargetX, kTargetY));

  if (finished)
  {
    canvas->SetColor(255, 255, 255);
    canvas->SetFill(false);
    canvas->DrawText(position_x - 0.09f, position_y + 0.09f, "Herzlichen Glückwunsch! Ziel erreicht!");
  }
  
  canvas->SetColor(0, 180, 0);
  canvas->SetFill(true);
  canvas->DrawPolygon(rrlib::math::tVec2d(rendered_triangle.p1_x, rendered_triangle.p1_y),
  rrlib::math::tVec2d(rendered_triangle.p2_x, rendered_triangle.p2_y),
  rrlib::math::tVec2d(rendered_triangle.p3_x, rendered_triangle.p3_y));
  
  out_visualization.Publish(canvas);
}

}
}
