//----------------------------------------------------------------------
/*!
 * \file    projects/SensorProject/mFrequencyEstimator.cpp
 *
 * \author  tom passberg
 *
 * \date    2026-09-16
 *
 */
//----------------------------------------------------------------------
#include "projects/SensorProject/mFrequencyEstimator.h"

#include <chrono>

namespace finroc
{
namespace SensorProject
{

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mFrequencyEstimator> cCREATE_ACTION_FOR_M_FREQUENCYESTIMATOR("FrequencyEstimator");
#endif

mFrequencyEstimator::mFrequencyEstimator(core::tFrameworkElement *parent, const std::string &name) :
  structure::tModule(parent, name),
  state_signal { false, 0.0f, 0, false, rrlib::time::tTimestamp(), 0.0f }
{}

mFrequencyEstimator::~mFrequencyEstimator()
{}

void mFrequencyEstimator::OnStaticParameterChange()
{
}

void mFrequencyEstimator::OnParameterChange()
{
}

float mFrequencyEstimator::ProcessSignal(tInput<float> &input, tSignalState &state)
{
  const float low_pass_alpha = 0.2f;
  const float hysteresis_high = 0.1f;
  const float hysteresis_low = -0.1f;
  const float smoothing_alpha = 0.25f;
  const double min_period_seconds = 0.2;
  const double max_period_seconds = 8.0;
  const double silence_timeout_seconds = 10.0;

  const float raw_sample = input.Get();
  const rrlib::time::tTimestamp now = rrlib::time::Now();

  if (!state.initialized)
  {
    state.filtered_sample = raw_sample;
    state.initialized = true;
  }
  else
  {
    state.filtered_sample += low_pass_alpha * (raw_sample - state.filtered_sample);
  }

  const int new_hysteresis_state = (state.filtered_sample >= hysteresis_high) ? 1 :
                                   (state.filtered_sample <= hysteresis_low) ? -1 :
                                   state.hysteresis_state;

  if (state.hysteresis_state == -1 && new_hysteresis_state == 1)
  {
    if (state.has_last_rising_crossing)
    {
      const double period_seconds = std::chrono::duration<double>(now - state.last_rising_crossing).count();

      if (period_seconds >= min_period_seconds && period_seconds <= max_period_seconds)
      {
        const float instantaneous_frequency_hz = static_cast<float>(1.0 / period_seconds);
        state.frequency_hz = (state.frequency_hz <= 0.0f) ?
                             instantaneous_frequency_hz :
                             smoothing_alpha * instantaneous_frequency_hz +
                             (1.0f - smoothing_alpha) * state.frequency_hz;
      }
    }

    state.last_rising_crossing = now;
    state.has_last_rising_crossing = true;
  }

  if (state.has_last_rising_crossing)
  {
    const double silence_seconds = std::chrono::duration<double>(now - state.last_rising_crossing).count();
    if (silence_seconds > silence_timeout_seconds)
    {
      state.frequency_hz = 0.0f;
      state.has_last_rising_crossing = false;
    }
  }

  state.hysteresis_state = new_hysteresis_state;
  return state.frequency_hz;
}

void mFrequencyEstimator::Update()
{
  out_frequency.Publish(ProcessSignal(in_signal, state_signal));
}

}
}
