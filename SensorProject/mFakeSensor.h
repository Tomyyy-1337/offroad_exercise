//----------------------------------------------------------------------
/*!\file    projects/SensorProject/mFakeSensor.h
 *
 * \author  tom passberg
 *
 * \date    2026-08-25
 *
 * \brief Contains mFakeSensor
 *
 * \b mFakeSensor
 *
 * This module provides fake sensor readings for the exercise.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__SensorProject__mFakeSensor_h__
#define __projects__SensorProject__mFakeSensor_h__

#include "plugins/structure/tModule.h"
#include "rrlib/time/time.h"
//----------------------------------------------------------------------
namespace finroc
{
namespace SensorProject
{

class mFakeSensor : public structure::tModule
{

public:

  mFakeSensor(core::tFrameworkElement *parent, const std::string &name = "FakeSensor");

  tOutput<float> out_a1;
  tOutput<float> out_a2;
  tOutput<float> out_a3;

  tOutput<float> out_b1;
  tOutput<float> out_b2;

protected:
  virtual ~mFakeSensor();

private:
  rrlib::time::tTimestamp start_time;

  virtual void OnStaticParameterChange() override;   

  virtual void OnParameterChange() override;   

  virtual void Update() override;

};

}
}

#endif
