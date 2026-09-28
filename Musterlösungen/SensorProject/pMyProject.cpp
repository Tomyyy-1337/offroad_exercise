#include "plugins/structure/default_main_wrapper.h"
#include "projects/SensorProject/mFakeSensor.h"
#include "projects/SensorProject/mFrequencyEstimator.h"


const std::string cPROGRAM_DESCRIPTION = "Starts MyProject.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

void StartUp()
{}

void CreateMainGroup(const std::vector<std::string>& remaining_arguments)
{
  auto main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);

  auto fake_sensor = new finroc::SensorProject::mFakeSensor(main_thread, "FakeSensor");
  auto frequency_estimator_a1 = new finroc::SensorProject::mFrequencyEstimator(main_thread, "FrequencyEstimator_a1");
  frequency_estimator_a1->in_signal.ConnectTo(fake_sensor->out_a1);

  auto frequency_estimator_a2 = new finroc::SensorProject::mFrequencyEstimator(main_thread, "FrequencyEstimator_a2");
  frequency_estimator_a2->in_signal.ConnectTo(fake_sensor->out_a2);

  auto frequency_estimator_a3 = new finroc::SensorProject::mFrequencyEstimator(main_thread, "FrequencyEstimator_a3");
  frequency_estimator_a3->in_signal.ConnectTo(fake_sensor->out_a3);

  auto frequency_estimator_b1 = new finroc::SensorProject::mFrequencyEstimator(main_thread, "FrequencyEstimator_b1");
  frequency_estimator_b1->in_signal.ConnectTo(fake_sensor->out_b1);

  auto frequency_estimator_b2 = new finroc::SensorProject::mFrequencyEstimator(main_thread, "FrequencyEstimator_b2");
  frequency_estimator_b2->in_signal.ConnectTo(fake_sensor->out_b2);
}