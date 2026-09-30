//----------------------------------------------------------------------
/*!\file    projects/PointcloudProject/mSimulation.h
 *
 * \author  tom passberg
 *
 * \date    2026-09-17
 *
 * \brief Contains mTriangleSimulation
 *
 * \b mTriangleSimulation
 *
 * This module simulates a triangle moving on a canvas based on velocity and steering inputs.
 * It displays the triangle's position and orientation with visualization.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__PointcloudProject__mSimulation_h__
#define __projects__PointcloudProject__mSimulation_h__

#include "plugins/structure/tModule.h"
#include "rrlib/canvas/tCanvas2D.h"
#include "rrlib/distance_data/tDistanceData.h"

//----------------------------------------------------------------------
namespace finroc
{
namespace PointcloudProject
{

class mSimulation : public structure::tModule
{

public:

  mSimulation(core::tFrameworkElement *parent, const std::string &name = "Simulation");

  tInput<float> in_velocity;      // Velocity in units per update
  tInput<float> in_steering;      // Steering angle in radians

  // 360 degree lidar-style point cloud around the triangle. Points are in the
  // triangle's local frame, with z=0 for the flat ground and z=2 m on obstacles.
  tOutput<rrlib::distance_data::tDistanceData> out_pointcloud;

  // Collision output port for triangle-obstacle intersection
  tOutput<bool> out_collision;
  
  // Current position of the triangle
  tOutput<rrlib::math::tVec2f> out_current_position;
  // Position of the goal
  tOutput<rrlib::math::tVec2f> out_goal_position;
  // Current heading in world coordinates
  tOutput<float> out_orientation;
  // Goal distance and direction relative to the current heading
  tOutput<float> out_goal_distance;
  tOutput<float> out_goal_direction;
  
  // 2D overview of the simulated world
  tVisualizationOutput<rrlib::canvas::tCanvas2D, tLevelOfDetail::ALL> out_visualization;
  // 3d pointcloud visualization
  tVisualizationOutput<rrlib::distance_data::tDistanceData, tLevelOfDetail::ALL> vis_point_cloud;

protected:
  virtual ~mSimulation();

private:
  // State variables
  float position_x;     // X position on canvas (0.0 to 1.0)
  float position_y;     // Y position on canvas (0.0 to 1.0)
  float orientation;    // Angle in radians
  bool finished;

  virtual void OnStaticParameterChange() override;   

  virtual void OnParameterChange() override;   

  virtual void Update() override;

};

}
}

#endif
