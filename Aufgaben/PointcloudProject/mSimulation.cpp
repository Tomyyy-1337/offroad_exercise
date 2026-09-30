//----------------------------------------------------------------------
/*!\file    projects/PointcloudProject/mTriangleSimulation.cpp
 *
 * \author  tom passberg
 *
 * \date    2026-09-17
 *
 */
//----------------------------------------------------------------------
#include "projects/PointcloudProject/mSimulation.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kSpeedScale = 0.01f;
constexpr float kWheelBase = 0.12f;
constexpr float kMaxSteeringAngle = 0.8f;
constexpr float kCameraZoom = 3.5f;
constexpr float kObstacleRadius = 0.045f;
constexpr float kObstacleGenerationRadius = 5.0f;
constexpr float kObstacleGridSpacing = 0.26f;
constexpr float kTargetX = 8.16f;
constexpr float kTargetY = 1.0f;
constexpr float kTargetRadius = 0.05f;
constexpr float kStartX = 0.42f;
constexpr float kStartY = 0.5f;
constexpr float kStartOrientation = 0.0f;
constexpr float kTriangleLength = 0.04f;
constexpr float kObstacleRayMaxDistance = 0.20f;
constexpr unsigned int kScanBeamCount = 360;
constexpr unsigned int kObstacleHeightSamples = 9;
constexpr unsigned int kGroundRangeSamples = 8;
constexpr float kObstacleHeight = 1.0f;

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

uint32_t ObstacleHash(int cell_x, int cell_y)
{
  uint32_t hash = static_cast<uint32_t>(cell_x) * 0x9e3779b9u;
  hash ^= static_cast<uint32_t>(cell_y) + 0x85ebca6bu + (hash << 6) + (hash >> 2);
  hash ^= hash >> 16;
  hash *= 0x7feb352du;
  hash ^= hash >> 15;
  hash *= 0x846ca68bu;
  return hash ^ (hash >> 16);
}

std::vector<tObstacle> GenerateObstacles(float position_x, float position_y)
{
  const int minimum_cell_x = static_cast<int>(std::floor((position_x - kObstacleGenerationRadius) / kObstacleGridSpacing));
  const int maximum_cell_x = static_cast<int>(std::floor((position_x + kObstacleGenerationRadius) / kObstacleGridSpacing));
  const int minimum_cell_y = static_cast<int>(std::floor((position_y - kObstacleGenerationRadius) / kObstacleGridSpacing));
  const int maximum_cell_y = static_cast<int>(std::floor((position_y + kObstacleGenerationRadius) / kObstacleGridSpacing));

  std::vector<tObstacle> obstacles;
  obstacles.reserve(static_cast<std::size_t>((maximum_cell_x - minimum_cell_x + 1) *
                                              (maximum_cell_y - minimum_cell_y + 1) / 3));

  for (int cell_y = minimum_cell_y; cell_y <= maximum_cell_y; ++cell_y)
  {
    for (int cell_x = minimum_cell_x; cell_x <= maximum_cell_x; ++cell_x)
    {
      const uint32_t hash = ObstacleHash(cell_x, cell_y);
      if (hash % 4u == 3u)
      {
        continue;
      }

      const float offset_x = (static_cast<float>((hash >> 8) & 0xffu) / 255.0f - 0.5f) * 0.30f;
      const float offset_y = (static_cast<float>((hash >> 16) & 0xffu) / 255.0f - 0.5f) * 0.30f;
      const float obstacle_x = (static_cast<float>(cell_x) + 0.5f) * kObstacleGridSpacing + offset_x;
      const float obstacle_y = (static_cast<float>(cell_y) + 0.5f) * kObstacleGridSpacing + offset_y;

      if (DistanceSquared(position_x, position_y, obstacle_x, obstacle_y) >
            kObstacleGenerationRadius * kObstacleGenerationRadius ||
          DistanceSquared(kStartX, kStartY, obstacle_x, obstacle_y) <
            (kObstacleRadius + kTriangleLength) * (kObstacleRadius + kTriangleLength) ||
          DistanceSquared(kTargetX, kTargetY, obstacle_x, obstacle_y) <
            (kObstacleRadius + kTargetRadius) * (kObstacleRadius + kTargetRadius))
      {
        continue;
      }

      obstacles.push_back({obstacle_x, obstacle_y});
    }
  }

  return obstacles;
}

tObstacleRayHit FindNextObstacle(float front_x, float front_y, float orientation,
                                 const std::vector<tObstacle> &obstacles)
{
  const float direction_x = std::cos(orientation);
  const float direction_y = std::sin(orientation);

  tObstacleRayHit best_hit{false, 0.0f, 0.0f, 0.0f};
  float best_distance = 0.0f;

  for (const auto &obstacle : obstacles)
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
namespace PointcloudProject
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
  const std::vector<tObstacle> obstacles = GenerateObstacles(position_x, position_y);
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

    for (const auto &obstacle : obstacles)
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
  }

  out_current_position.Publish(rrlib::math::tVec2f(position_x, position_y));
  out_goal_position.Publish(rrlib::math::tVec2f(kTargetX, kTargetY));
  out_orientation.Publish(orientation);
  const float goal_delta_x = kTargetX - position_x;
  const float goal_delta_y = kTargetY - position_y;
  out_goal_distance.Publish(std::sqrt(goal_delta_x * goal_delta_x + goal_delta_y * goal_delta_y));
  out_goal_direction.Publish(NormalizeAngle(std::atan2(goal_delta_y, goal_delta_x) - orientation));

  const tTriangleVertices rendered_triangle = CalculateTriangleVertices(position_x, position_y, orientation);

  const float cos_orientation = std::cos(orientation);
  const float sin_orientation = std::sin(orientation);
  const unsigned int maximum_points = kScanBeamCount * (kObstacleHeightSamples + kGroundRangeSamples);

  auto point_cloud = out_pointcloud.GetUnusedBuffer();
  point_cloud->Resize(rrlib::distance_data::eDF_CARTESIAN_3D_FLOAT, maximum_points, false, 0);
  point_cloud->SetUnit(rrlib::distance_data::eDISTANCE_UNIT_M);

  rrlib::math::tVec3f *points = reinterpret_cast<rrlib::math::tVec3f *>(point_cloud->DataPtr());
  unsigned int point_count = 0;

  // One horizontal ray per degree gives a complete scan around the triangle.
  // Hits are expanded vertically to describe the 2 m high cylindrical
  // obstacles. Several ground returns per ray make the flat ground visible
  // instead of rendering only the outer scan boundary.
  for (unsigned int beam = 0; beam < kScanBeamCount; ++beam)
  {
    const float ray_angle = kTwoPi * static_cast<float>(beam) / static_cast<float>(kScanBeamCount);
    const tObstacleRayHit hit = FindNextObstacle(position_x, position_y, ray_angle, obstacles);

    if (hit.found)
    {
      const float world_dx = hit.hit_x - position_x;
      const float world_dy = hit.hit_y - position_y;
      const float local_x = cos_orientation * world_dx + sin_orientation * world_dy;
      const float local_y = -sin_orientation * world_dx + cos_orientation * world_dy;

      for (unsigned int height_sample = 0; height_sample < kObstacleHeightSamples; ++height_sample)
      {
        points[point_count++] = rrlib::math::tVec3f(local_x, local_y,
          kObstacleHeight * static_cast<float>(height_sample) / static_cast<float>(kObstacleHeightSamples - 1));
      }
    }

    const float ground_end_distance = hit.found ? hit.distance : kObstacleRayMaxDistance;
    for (unsigned int ground_sample = 1; ground_sample <= kGroundRangeSamples; ++ground_sample)
    {
      const float distance = ground_end_distance * static_cast<float>(ground_sample) /
                             static_cast<float>(kGroundRangeSamples);
      const float world_dx = distance * std::cos(ray_angle);
      const float world_dy = distance * std::sin(ray_angle);
      const float local_x = cos_orientation * world_dx + sin_orientation * world_dy;
      const float local_y = -sin_orientation * world_dx + cos_orientation * world_dy;
      points[point_count++] = rrlib::math::tVec3f(local_x, local_y, 0.0f);
    }
  }

  point_cloud->SetDimension(point_count);
  if (vis_point_cloud.IsConnected())
  {
    auto visualization = vis_point_cloud.GetUnusedBuffer();
    visualization->Resize(rrlib::distance_data::eDF_CARTESIAN_3D_FLOAT, point_count, false, 0);
    visualization->SetUnit(rrlib::distance_data::eDISTANCE_UNIT_M);

    const rrlib::math::tVec3f *local_points =
      reinterpret_cast<const rrlib::math::tVec3f *>(point_cloud->DataPtr());
    rrlib::math::tVec3f *world_points =
      reinterpret_cast<rrlib::math::tVec3f *>(visualization->DataPtr());
    for (unsigned int index = 0; index < point_count; ++index)
    {
      const float local_x = local_points[index].X();
      const float local_y = local_points[index].Y();
      world_points[index] = rrlib::math::tVec3f(
        position_x + cos_orientation * local_x - sin_orientation * local_y,
        position_y + sin_orientation * local_x + cos_orientation * local_y,
        local_points[index].Z());
    }
    visualization->SetDimension(point_count);

    vis_point_cloud.Publish(visualization);
  }
  out_pointcloud.Publish(point_cloud);

  auto canvas = out_visualization.GetUnusedBuffer();
  canvas->Clear();

  canvas->SetDefaultViewport(-0.5f, -0.5f, 2.0f, 2.0f);

  canvas->ResetTransformation();
  canvas->Translate(0.5f, 0.5f);
  canvas->Scale(kCameraZoom, kCameraZoom);
  canvas->Translate(-position_x, -position_y);

  canvas->SetColor(220, 80, 60);
  canvas->SetFill(true);

  for (const auto &obstacle : obstacles)
  {
    canvas->DrawEllipsoid(obstacle.x, obstacle.y, kObstacleRadius * 2.0f, kObstacleRadius * 2.0f);
  }

  canvas->SetColor(255, 255, 0);
  canvas->SetFill(true);
  canvas->DrawEllipsoid(kTargetX, kTargetY, kTargetRadius * 2.0f, kTargetRadius * 2.0f);

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
