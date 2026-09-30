//----------------------------------------------------------------------
/*!\file    projects/SensorProject/mFakeSensor.cpp
 *
 * \author  tom passberg
 *
 * \date    2026-08-25
 *
 */
//----------------------------------------------------------------------
#include "projects/SensorProject/mFakeSensor.h"

#include <chrono>
#include <cmath>

#include <cassert>

namespace finroc
{
namespace SensorProject
{

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mFakeSensor> cCREATE_ACTION_FOR_M_FAKESENSOR("FakeSensor");
#endif

mFakeSensor::mFakeSensor(core::tFrameworkElement *parent, const std::string &name) :
  structure::tModule(parent, name),
  start_time(rrlib::time::Now())
{}

mFakeSensor::~mFakeSensor()
{}

void mFakeSensor::OnStaticParameterChange()
{

}

void mFakeSensor::OnParameterChange()
{
 
}

void mFakeSensor::Update()
{
  constexpr float kPi = 3.14159265358979323846f;
  const float elapsed_seconds = std::chrono::duration<float>(rrlib::time::Now() - start_time).count();

  auto phase = [](float frequency, float elapsed_seconds)
  {
    return elapsed_seconds * frequency * 2.0f * kPi;
  };

  auto square = [](float phase)
  {
    return std::sin(phase) >= 0.0f ? 0.5f : -0.5f;
  };

  auto triangle = [](float phase)
  {
    return 0.5f * (2.0f / kPi) * std::asin(std::sin(phase));
  };

  auto noise = [](float amplitude)
  {
    return amplitude * ((rand() % 100) / 100.0f - 0.5f);
  };

  const float a1_frequency = 1.0f;
  const float a2_frequency = 0.55f;
  const float a3_frequency = 0.29f;
  const float challenge1_frequency = 0.59f;
  const float challenge2_frequency = 0.42f;

  out_a1.Publish(std::sin(phase(a1_frequency, elapsed_seconds)) + noise(0.05f));
  out_a2.Publish(square(phase(a2_frequency, elapsed_seconds)) + noise(0.05f));
  out_a3.Publish(triangle(phase(a3_frequency, elapsed_seconds)) + noise(0.05f));

  out_b1.Publish(0.5f * std::sin(phase(challenge1_frequency, elapsed_seconds)) + noise(0.3f));
  out_b2.Publish(triangle(phase(challenge2_frequency, elapsed_seconds)) + noise(0.5f));
}

}
}
