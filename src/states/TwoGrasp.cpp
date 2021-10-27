#include "TwoGrasp.h"

#include "../McTestGraspController.h"

void TwoGrasp::configure(const mc_rtc::Configuration & config)
{
}

void TwoGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

bool TwoGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  output("OK");
  return true;
}

void TwoGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

EXPORT_SINGLE_STATE("TwoGrasp", TwoGrasp)
