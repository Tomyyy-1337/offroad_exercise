//----------------------------------------------------------------------
/*!\file    projects/CollisonAvoidanceProject/mSimulation.h
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
#ifndef __projects__CollisonAvoidanceProject__mSimulation_h__
#define __projects__CollisonAvoidanceProject__mSimulation_h__

#include "plugins/structure/tModule.h"
#include "rrlib/canvas/tCanvas2D.h"

//----------------------------------------------------------------------
namespace finroc
{
namespace CollisonAvoidanceProject
{

class mSimulation : public structure::tModule
{

public:

  mSimulation(core::tFrameworkElement *parent, const std::string &name = "Simulation");

  // Input ports for control
  tInput<float> in_velocity;      // Velocity in units per update
  tInput<float> in_steering;      // Steering angle in radians

  // Visualization output port for drawing on a canvas
  tVisualizationOutput<rrlib::canvas::tCanvas2D, tLevelOfDetail::ALL> out_visualization;

  // Collision output port for triangle-obstacle intersection
  tOutput<bool> out_collision;

  // Goal-relative outputs from the triangle's perspective
  tOutput<float> out_goal_distance;
  tOutput<float> out_goal_direction;

  // Distance to the next obstacle straight ahead of the triangle
  tOutput<float> out_next_obstacle_distance;

  // Distance to the next obstacle 10 degrees to the left/right of the triangle
  tOutput<float> out_next_obstacle_distance_left_10;
  tOutput<float> out_next_obstacle_distance_right_10;

  // Distance to the next obstacle 20 degrees to the left/right of the triangle
  tOutput<float> out_next_obstacle_distance_left_20;
  tOutput<float> out_next_obstacle_distance_right_20;

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
