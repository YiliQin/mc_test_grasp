#include "TwoGrasp.h"

#include "../McTestGraspController.h"

void TwoGrasp::configure(const mc_rtc::Configuration & config)
{
  config("hand_to_remove", hand_to_remove_);
  config("action", action_);
  config("target_relative", target_relative_);
}

void TwoGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);
  
  if (ctl.auto_mode_ == true && action_ == "Remove" && hand_to_remove_ == "Left")
    remove_left_ = true;
  if (ctl.auto_mode_ == true && action_ == "Remove" && hand_to_remove_ == "Right")
    remove_right_ = true; 
}

bool TwoGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  
  if (remove_left_ == true)
  {
    remove_left_ = false;
    output("ReqRemoveLeft");
    return true;
  }
  else if (remove_right_ == true)
  {
    remove_right_ = false;
    output("ReqRemoveRight");
    return true;
  }

  return false;
}

void TwoGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  auto & gui = *ctl_.gui();

  gui.removeCategory({"TwoGrasp"});

}

void TwoGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"TwoGrasp"},
                 mc_rtc::gui::Button("Remove left hand", [this]() {remove_left_ = true;}),
                 mc_rtc::gui::Button("Remove right hand", [this]() {remove_right_ = true;})
                 );
}

EXPORT_SINGLE_STATE("TwoGrasp", TwoGrasp)
