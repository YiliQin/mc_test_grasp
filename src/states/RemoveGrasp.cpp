#include "RemoveGrasp.h"

#include "../McTestGraspController.h"

void RemoveGrasp::configure(const mc_rtc::Configuration & config)
{
}

void RemoveGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

bool RemoveGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  output("OK");
  return true;
}

void RemoveGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

EXPORT_SINGLE_STATE("RemoveGrasp", RemoveGrasp)
