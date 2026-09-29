#ifndef __projects__PointcloudProject__mVirtualDistanceSensor_h__
#define __projects__PointcloudProject__mVirtualDistanceSensor_h__

#include "plugins/ib2c/tModule.h"
#include "rrlib/aspect_maps/tGridAspectMap.h"
#include "rrlib/canvas/tCanvas2D.h"

namespace finroc::PointcloudProject
{

/** Calculates virtual distance sensors along the triangle center and edges. */
class mVirtualDistanceSensor : public ib2c::tModule
{
public:
  mVirtualDistanceSensor(core::tFrameworkElement *parent, const std::string &name = "Virtual Distance Sensor",
                         ib2c::tStimulationMode stimulation_mode = ib2c::tStimulationMode::AUTO);

  tParameter<float> par_max_distance;
  tParameter<float> par_obstacle_height;

  tInput<rrlib::aspect_maps::tGridAspectMap<float>> in_height_map;
  tOutput<float> out_center_distance;
  tOutput<float> out_left_distance;
  tOutput<float> out_right_distance;

  tVisualizationOutput<rrlib::canvas::tCanvas2D, tLevelOfDetail::ALL> vis_sensors;

protected:
  virtual ~mVirtualDistanceSensor();

private:
  struct tSensorRay
  {
    float origin_x;
    float origin_y;
    float direction_x;
    float direction_y;
  };

  virtual bool ProcessTransferFunction() override;
  virtual ib2c::tTargetRating CalculateTargetRating() const override;

  float measure_distance(const rrlib::aspect_maps::tGridAspectMap<float> &height_map,
                         const tSensorRay &ray) const;
  void VisualizeSensors(const rrlib::aspect_maps::tGridAspectMap<float> &height_map,
                        float left_distance, float right_distance);
};

}  // namespace finroc::PointcloudProject

#endif