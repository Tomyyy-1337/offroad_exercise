#include "plugins/structure/default_main_wrapper.h"
#include "projects/CollisonAvoidanceProject/mSimulation.h"

const std::string cPROGRAM_DESCRIPTION = "Starts MyProject.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

void StartUp()
{}

void CreateMainGroup(const std::vector<std::string>& remaining_arguments)
{
  auto main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);

  new finroc::CollisonAvoidanceProject::mSimulation(main_thread, "Simulation");
}