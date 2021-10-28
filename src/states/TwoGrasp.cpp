#include "TwoGrasp.h"

#include "../McTestGraspController.h"

void TwoGrasp::configure(const mc_rtc::Configuration & config)
{
}

void TwoGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);
}

bool TwoGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  
  if (remove_left_ == true)
  {
    remove_left_ = false;
    output("RemoveLeft");
    return true;
  }
  else if (remove_right_ == true)
  {
    remove_right_ = false;
    output("RemoveRight");
    return true;
  }

  return false;
}

void TwoGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  auto & gui = *ctl_.gui();

  gui.removeCategory({"Grasp"});

}

void TwoGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"Grasp"},
                 //mc_rtc::gui::ComboInput("hand", {"Left", "Right"},
                 //[this]() -> const std::string & {return hand_;},
                 //[this](const std::string & s) { hand_ = s; }),
                 //mc_rtc::gui::ArrayInput(
                 //"Target position [m/deg]", {"x", "y", "theta"},
                 //[this]() -> const Eigen::Vector3d & { return target_pos_; },
                 //[this](const Eigen::Vector3d & t) { target_pos_ = t; computeTarget(); }), 
                 mc_rtc::gui::Button("Remove Left", [this]() {remove_left_ = true;}),
                 mc_rtc::gui::Button("Remove Right", [this]() {remove_right_ = true;})
                 //mc_rtc::gui::Transform("preTarget", [this]() -> const sva::PTransformd & { return preTarget_; }),
                 //mc_rtc::gui::Transform("target", [this]() -> const sva::PTransformd & { return target_; })
                 );
}
EXPORT_SINGLE_STATE("TwoGrasp", TwoGrasp)
