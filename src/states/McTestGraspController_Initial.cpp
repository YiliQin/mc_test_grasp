#include "McTestGraspController_Initial.h"

void McTestGraspController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void McTestGraspController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);
}

bool McTestGraspController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (select_)
  {
    select_ = false;
    ctl.auto_mode_ = auto_mode_;
    if (auto_mode_)
      output("AutoMode");
    else
      output("ManualMode");
    return true;
  }

  return false;
}

void McTestGraspController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  auto & gui = *ctl_.gui();

  gui.removeCategory({"SelectTestMode"});
}

void McTestGraspController_Initial::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"SelectTestMode"},
                 mc_rtc::gui::Button("Auto mode and Run", [this]() {auto_mode_ = true; select_ = true; }),
                 mc_rtc::gui::Button("Manual mode", [this]() {auto_mode_ = false; select_ = true; })
                 );
}


EXPORT_SINGLE_STATE("McTestGraspController_Initial", McTestGraspController_Initial)
