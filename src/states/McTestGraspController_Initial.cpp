#include "McTestGraspController_Initial.h"

#include "../McTestGraspController.h"

void McTestGraspController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void McTestGraspController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

bool McTestGraspController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  output("OK");
  return true;
}

void McTestGraspController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

EXPORT_SINGLE_STATE("McTestGraspController_Initial", McTestGraspController_Initial)
