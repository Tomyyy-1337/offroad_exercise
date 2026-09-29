#include "projects/PointcloudProject/mVirtualDistanceSensor.h"
#include "projects/PointcloudProject/gridmapPainter.h"

#include "rrlib/coviroa/color_spaces/tRGB.h"

#include <algorithm>
#include <cmath>

namespace finroc::PointcloudProject
{

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mVirtualDistanceSensor> cCREATE_ACTION_FOR_M_VIRTUAL_DISTANCE_SENSOR("VirtualDistanceSensor");
#endif

namespace
{
constexpr float kTriangleTipX = 0.04f;
constexpr float kTriangleSideAngle = 2.356f;
}

mVirtualDistanceSensor::mVirtualDistanceSensor(core::tFrameworkElement *parent, const std::string &name,
                                               ib2c::tStimulationMode stimulation_mode) :
  tModule(parent, name, stimulation_mode, false),
  par_max_distance("Maximum Distance", 0.5f, this),
  par_obstacle_height("Obstacle Height", 11.0f, this)
{}

mVirtualDistanceSensor::~mVirtualDistanceSensor()
{}

bool mVirtualDistanceSensor::ProcessTransferFunction()
{
  if (!in_height_map.HasChanged())
  {
    return false;
  }

  auto height_map = in_height_map.GetPointer();
  const float side_x = kTriangleTipX * std::cos(kTriangleSideAngle);
  const float side_y = kTriangleTipX * std::sin(kTriangleSideAngle);
  const tSensorRay center_ray{0.0f, 0.0f, 1.0f, 0.0f};
  const tSensorRay left_ray{side_x, side_y, 1.0f, 0.0f};
  const tSensorRay right_ray{side_x, -side_y, 1.0f, 0.0f};
  const float center_distance = measure_distance(*height_map, center_ray);
  const float left_distance = measure_distance(*height_map, left_ray);
  const float right_distance = measure_distance(*height_map, right_ray);

  out_center_distance.Publish(center_distance);
  out_left_distance.Publish(left_distance);
  out_right_distance.Publish(right_distance);
  VisualizeSensors(*height_map, left_distance, right_distance);
  return true;
}

ib2c::tTargetRating mVirtualDistanceSensor::CalculateTargetRating() const
{
  return 1.0;
}

float mVirtualDistanceSensor::measure_distance(const rrlib::aspect_maps::tGridAspectMap<float> &height_map,
                                               const tSensorRay &ray) const
{
  const float cell_size = height_map.GetCellSize().Value();
  const float max_distance = std::max(par_max_distance.Get(), cell_size);
  const float step = cell_size * 0.5f;

  for (float distance = 0.0f; distance <= max_distance; distance += step)
  {
    const float x = ray.origin_x + distance * ray.direction_x;
    const float y = ray.origin_y + distance * ray.direction_y;
    const int cell_x = static_cast<int>(std::floor(x / cell_size + 0.5f));
    const int cell_y = static_cast<int>(std::floor(y / cell_size + 0.5f));

    if (cell_x < height_map.GetGridLowerBounds().X() || cell_x > height_map.GetGridUpperBounds().X() ||
        cell_y < height_map.GetGridLowerBounds().Y() || cell_y > height_map.GetGridUpperBounds().Y())
    {
      break;
    }

    if (height_map.GetCellValue(cell_x, cell_y) >= par_obstacle_height.Get())
    {
      return distance;
    }
  }
  return max_distance;
}

void mVirtualDistanceSensor::VisualizeSensors(const rrlib::aspect_maps::tGridAspectMap<float> &height_map,
                                              float left_distance, float right_distance)
{
  auto canvas = vis_sensors.GetUnusedBuffer();
  canvas->Clear();

  GridmapPainter<2> painter;
  painter.color_range(
    rrlib::coviroa::color_spaces::tRGB(52.0f, 152.0f, 235.0f),
    rrlib::coviroa::color_spaces::cRGB_BLACK,
    10.0f,
    12.0f);
  painter.value_coloring(0.0f, rrlib::coviroa::color_spaces::cRGB_WHITE);
  painter.draw(*canvas, height_map);

  const float side_x = kTriangleTipX * std::cos(kTriangleSideAngle);
  const float side_y = kTriangleTipX * std::sin(kTriangleSideAngle);
  const tSensorRay left_ray{side_x, side_y, 1.0f, 0.0f};
  const tSensorRay right_ray{side_x, -side_y, 1.0f, 0.0f};
  canvas->SetFill(false);

  canvas->SetColor(255, 80, 80);
  canvas->DrawLineSegment(rrlib::math::tVec2d(left_ray.origin_x, left_ray.origin_y),
                          rrlib::math::tVec2d(left_ray.origin_x + left_distance * left_ray.direction_x,
                                              left_ray.origin_y + left_distance * left_ray.direction_y));
  canvas->SetColor(80, 255, 80);
  canvas->DrawLineSegment(rrlib::math::tVec2d(right_ray.origin_x, right_ray.origin_y),
                          rrlib::math::tVec2d(right_ray.origin_x + right_distance * right_ray.direction_x,
                                              right_ray.origin_y + right_distance * right_ray.direction_y));

  canvas->SetColor(255, 255, 0);
  canvas->SetFill(true);
  canvas->DrawPolygon(rrlib::math::tVec2d(kTriangleTipX, 0.0),
                      rrlib::math::tVec2d(left_ray.origin_x, left_ray.origin_y),
                      rrlib::math::tVec2d(right_ray.origin_x, right_ray.origin_y));

  vis_sensors.Publish(canvas);
}

}  // namespace finroc::PointcloudProject