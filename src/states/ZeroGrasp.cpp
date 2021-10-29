#include "ZeroGrasp.h"

void ZeroGrasp::configure(const mc_rtc::Configuration & config)
{
  config("depth", depth_);
  config("approach", approach_depth_);
  config("threshold1", threshold1_);
  config("threshold2", threshold2_);
  config("threshold3", threshold3_);
}

void ZeroGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);
  
  hand_surface_pose_.translation() = Eigen::Vector3d::Identity();
  hand_surface_pose_.rotation() = sva::RotY(-mc_rtc::constants::PI /2)*sva::RotX(-mc_rtc::constants::PI /2);

}

bool ZeroGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (step_ == 0 && add_ == true)
  {
    if (hand_ == "Left")
    {
      activeTask_ = ctl.leftHandTask_; 
    }
    else if (hand_ == "Right")
    {
      activeTask_ = ctl.rightHandTask_; 
    }
    else ;

    add_ = false;
    step_ = 1;
    activeTask_->target(pre_target_);
    
    return false; 
  }
  if (step_ == 1 && activeTask_->eval().norm() < threshold1_)
  {
    activeTask_->target(target_);
    step_ = 2;
    return false; 
  }
  if (step_ == 2 && activeTask_->eval().norm() < threshold2_)
  {
    step_ = 10;
    mc_rtc::log::info("ZeroGrasp added hand, add new target for [Move hand] or [Next step] to next");
    return false;
  }
  if (step_ == 10 && move_ == true)
  {
    move_ = false;
    step_ = 11;
    activeTask_->target(target_);
    mc_rtc::log::info("Added new target for [Move hand]");
    
    return false;
  }
  if (step_ == 11 && activeTask_->eval().norm() < threshold3_)
  {
    step_ = 10; 
    mc_rtc::log::info("Moved to new target");

    return false;
  }
  if (step_ == 10 && next_ == true)
  {    
    next_ = false;
    if (hand_ == "Left")
      output("AddedLeft");
    else if (hand_ == "Right")
      output("AddedRight");
    else ;

    return true;
  }
  
  return false;

}

void ZeroGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  auto & gui = *ctl_.gui();

  gui.removeCategory({"Grasp"});

}

void ZeroGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"Grasp"},
                 mc_rtc::gui::ComboInput("Choose hand", {"Left", "Right"},
                 [this]() -> const std::string & {return hand_;},
                 [this](const std::string & s) { hand_ = s; }),
                 mc_rtc::gui::ArrayInput(
                 "Target position [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_pose_; },
                 [this](const Eigen::Vector3d & t) { target_pose_ = t; computeTarget(); }), 
                 mc_rtc::gui::Button("Add hand", [this]() {add_ = true;}),
                 mc_rtc::gui::Button("Move hand", [this]() {move_ = true;}),
                 mc_rtc::gui::Button("Next step", [this]() {next_ = true;}),
                 mc_rtc::gui::Transform("[pre_target]", [this]() -> const sva::PTransformd & { return pre_target_; }),
                 mc_rtc::gui::Transform("[target]", [this]() -> const sva::PTransformd & { return target_; })
                 );
}

void ZeroGrasp::computeTarget()
{
  target_.translation() = Eigen::Vector3d(depth_, -target_pose_.x(), target_pose_.y());
  if (hand_ == "Left")
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*target_pose_.z()/180);
  else if (hand_ == "Right")
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1-target_pose_.z()/180));
  else ;

  pre_target_ = target_;
  pre_target_.translation().x() -= approach_depth_;
}

EXPORT_SINGLE_STATE("ZeroGrasp", ZeroGrasp)
