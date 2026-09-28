//----------------------------------------------------------------------
/*!
 * \file    projects/SensorProject/mFrequencyEstimator.h
 *
 * \author  tom passberg
 *
 * \date    2026-09-16
 *
 * \brief Contains mFrequencyEstimator
 *
 * \b mFrequencyEstimator
 *
 * Estimates the frequency of a single incoming sensor signal.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__SensorProject__mFrequencyEstimator_h__
#define __projects__SensorProject__mFrequencyEstimator_h__

#include "plugins/structure/tModule.h"

#include "rrlib/time/time.h"

//----------------------------------------------------------------------
namespace finroc
{
namespace SensorProject
{

class mFrequencyEstimator : public structure::tModule
{

public:

  mFrequencyEstimator(core::tFrameworkElement *parent, const std::string &name = "FrequencyEstimator");

  tInput<float> in_signal;

  tOutput<float> out_frequency;

protected:
  virtual ~mFrequencyEstimator();

private:

  struct tSignalState
  {
    bool initialized;
    float filtered_sample;
    int hysteresis_state;
    bool has_last_rising_crossing;
    rrlib::time::tTimestamp last_rising_crossing;
    float frequency_hz;
  };

  tSignalState state_signal;

  float ProcessSignal(tInput<float> &input, tSignalState &state);

  virtual void OnStaticParameterChange() override;

  virtual void OnParameterChange() override;

  virtual void Update() override;

};

}
}

#endif