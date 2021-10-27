#include "OneGrasp.h"

void OneGrasp::configure(const mc_rtc::Configuration & config)
{
  config("depth", depth_);
  config("approach", approachDepth_);
  config("threshold1", threshold1_);
  config("threshold2", threshold2_);
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

void OneGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"Grasp"},
                 //mc_rtc::gui::ComboInput("hand", {"Left", "Right"},
                 //[this]() -> const std::string & {return hand_;},
                 //[this](const std::string & s) { hand_ = s; }),
                 mc_rtc::gui::ArrayInput(
                 "Target position [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_pos_; },
                 [this](const Eigen::Vector3d & t) { target_pos_ = t; computeTarget(); }), 
                 mc_rtc::gui::Button("Add opposite", [this]() {add_ = true;}),
                 mc_rtc::gui::Button("Remove", [this]() {remove_ = true;}),
                 mc_rtc::gui::Transform("preTarget", [this]() -> const sva::PTransformd & { return preTarget_; }),
                 mc_rtc::gui::Transform("target", [this]() -> const sva::PTransformd & { return target_; })
                 );
}

void OneGrasp::computeTarget()
{
  world_to_surface_.translation() = Eigen::Vector3d::Identity();
  world_to_surface_.rotation() = sva::RotY(-M_PI/2)*sva::RotX(-M_PI/2);
  target_.translation() = Eigen::Vector3d(depth_, -target_pos_.x(), target_pos_.y());
  //target_.rotation() = sva::RotX(M_PI*target_pos_.z()/180);
  target_.rotation() = sva::RotX(M_PI*target_pos_.z()/180)*world_to_surface_.rotation();
  preTarget_ = target_;
  preTarget_.translation().x() -= approachDepth_;
}

EXPORT_SINGLE_STATE("OneGrasp", OneGrasp)
