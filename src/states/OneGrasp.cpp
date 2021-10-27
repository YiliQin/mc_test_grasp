#include "OneGrasp.h"

#include "../McTestGraspController.h"

void OneGrasp::configure(const mc_rtc::Configuration & config)
{
}

void OneGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

bool OneGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  output("OK");
  return true;
}

void OneGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
}

EXPORT_SINGLE_STATE("OneGrasp", OneGrasp)
