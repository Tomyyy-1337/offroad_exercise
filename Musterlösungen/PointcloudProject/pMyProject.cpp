#include "plugins/structure/default_main_wrapper.h"
#include "plugins/ib2c/mbbFusion.h"
#include "projects/PointcloudProject/mSimulation.h"
#include "projects/PointcloudProject/mPointcloudToHeightMap.h"
#include "projects/PointcloudProject/mVirtualDistanceSensor.h"
#include "projects/PointcloudProject/mbbDriveToPoint.h"
#include "projects/PointcloudProject/mbbAvoidObstacles.h"

const std::string cPROGRAM_DESCRIPTION = "Starts MyProject.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

void StartUp()
{}

void CreateMainGroup(const std::vector<std::string>& remaining_arguments)
{
  auto main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);

  auto simulation = new finroc::PointcloudProject::mSimulation(main_thread, "Simulation");
  auto pointcloud_to_height_map = new finroc::PointcloudProject::mPointcloudToHeightMap(main_thread, "Pointcloud To Height Map");
  auto virtual_distance_sensor = new finroc::PointcloudProject::mVirtualDistanceSensor(main_thread, "Virtual Distance Sensor");
  auto drive_to_point = new finroc::PointcloudProject::mbbDriveToPoint(
    main_thread, "DriveToPoint", finroc::ib2c::tStimulationMode::ENABLED);
  auto avoid_obstacles = new finroc::PointcloudProject::mbbAvoidObstacles(
    main_thread, "AvoidObstacles", finroc::ib2c::tStimulationMode::ENABLED);
  auto steering_fusion = new finroc::ib2c::mbbMaximumFusion<2, float>(
    main_thread, "SteeringFusion", finroc::ib2c::tStimulationMode::AUTO, false);

  simulation->out_pointcloud.ConnectTo(pointcloud_to_height_map->in_pointcloud);
  pointcloud_to_height_map->out_height_map.ConnectTo(virtual_distance_sensor->in_height_map);

  drive_to_point->in_goal_distance.ConnectTo(simulation->out_goal_distance);
  drive_to_point->in_goal_direction.ConnectTo(simulation->out_goal_direction);
  avoid_obstacles->in_next_obstacle_distance_center.ConnectTo(virtual_distance_sensor->out_center_distance);
  avoid_obstacles->in_next_obstacle_distance_left_edge.ConnectTo(virtual_distance_sensor->out_left_distance);
  avoid_obstacles->in_next_obstacle_distance_right_edge.ConnectTo(virtual_distance_sensor->out_right_distance);

  simulation->in_velocity.ConnectTo(drive_to_point->out_velocity);
  avoid_obstacles->activity.ConnectTo(steering_fusion->InputAt(0).activity);
  drive_to_point->activity.ConnectTo(steering_fusion->InputAt(1).activity);
  steering_fusion->InputAt(0).data.ConnectTo(avoid_obstacles->out_steering);
  steering_fusion->InputAt(1).data.ConnectTo(drive_to_point->out_steering);
  steering_fusion->Output().ConnectTo(simulation->in_steering);

  
}