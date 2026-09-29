#ifndef __projects__PointcloudProject__mPointcloudToHeightMap_h__
#define __projects__PointcloudProject__mPointcloudToHeightMap_h__

#include "plugins/ib2c/tModule.h"
#include "rrlib/aspect_maps/tGridAspectMap.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/si_units/si_units.h"

namespace finroc::PointcloudProject
{

/** Converts a pointcloud into a gridmap containing the highest point in each cell. */
class mPointcloudToHeightMap : public ib2c::tModule
{
public:
  mPointcloudToHeightMap(core::tFrameworkElement *parent, const std::string &name = "PointcloudToHeightMap",
                         ib2c::tStimulationMode stimulation_mode = ib2c::tStimulationMode::AUTO);

  tParameter<rrlib::si_units::tLength<float>> par_cell_size;
  tParameter<rrlib::si_units::tLength<float>> par_map_size;

  tInput<rrlib::distance_data::tDistanceData> in_pointcloud;
  tOutput<rrlib::aspect_maps::tGridAspectMap<float>> out_height_map;

  tVisualizationOutput<rrlib::canvas::tCanvas2D, tLevelOfDetail::ALL> vis_height_map_2D;

protected:
  virtual ~mPointcloudToHeightMap();

private:
  rrlib::aspect_maps::tGridAspectMap<> height_map;

  void initialize_height_map();
  virtual void OnParameterChange() override;
  virtual bool ProcessTransferFunction() override;
  virtual ib2c::tTargetRating CalculateTargetRating() const override;
  void VisualizeHeightMap();
  void insert_point_in_height_map(const rrlib::math::tVec3f &point);
};

}  // namespace finroc::PointcloudProject

#endif