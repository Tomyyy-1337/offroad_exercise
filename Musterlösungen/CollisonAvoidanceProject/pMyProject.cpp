#include "plugins/structure/default_main_wrapper.h"
#include "plugins/ib2c/mbbFusion.h"
#include "projects/CollisonAvoidanceProject/mSimulation.h"
#include "projects/CollisonAvoidanceProject/mbbDriveToGoal.h"
#include "projects/CollisonAvoidanceProject/mbbAvoidObstacles.h"

const std::string cPROGRAM_DESCRIPTION = "Starts MyProject.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

void StartUp()
{}

void CreateMainGroup(const std::vector<std::string>& remaining_arguments)
{
  auto main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);

  auto simulation = new finroc::CollisonAvoidanceProject::mSimulation(main_thread, "Simulation");

  auto drive_to_goal = new finroc::CollisonAvoidanceProject::mbbDriveToGoal(
    main_thread, "DriveToGoal", finroc::ib2c::tStimulationMode::ENABLED);
  auto avoid_obstacles = new finroc::CollisonAvoidanceProject::mbbAvoidObstacles(
    main_thread, "AvoidObstacles", finroc::ib2c::tStimulationMode::ENABLED);
  auto steering_fusion = new finroc::ib2c::mbbMaximumFusion<2, float>(
    main_thread, "SteeringFusion", finroc::ib2c::tStimulationMode::AUTO, false);

  drive_to_goal->in_goal_distance.ConnectTo(simulation->out_goal_distance);
  drive_to_goal->in_goal_direction.ConnectTo(simulation->out_goal_direction);
  avoid_obstacles->in_next_obstacle_distance_center.ConnectTo(simulation->out_next_obstacle_distance_center);
  avoid_obstacles->in_next_obstacle_distance_left_edge.ConnectTo(simulation->out_next_obstacle_distance_left_edge);
  avoid_obstacles->in_next_obstacle_distance_right_edge.ConnectTo(simulation->out_next_obstacle_distance_right_edge);

  simulation->in_velocity.ConnectTo(drive_to_goal->out_velocity);
  avoid_obstacles->activity.ConnectTo(steering_fusion->InputAt(0).activity);
  drive_to_goal->activity.ConnectTo(steering_fusion->InputAt(1).activity);
  steering_fusion->InputAt(0).data.ConnectTo(avoid_obstacles->out_steering);
  steering_fusion->InputAt(1).data.ConnectTo(drive_to_goal->out_steering);
  steering_fusion->Output().ConnectTo(simulation->in_steering);

}