#include "OneGrasp.h"

void OneGrasp::configure(const mc_rtc::Configuration & config)
{
  config("depth", depth_);
  config("approach", approach_depth_);
  config("threshold1", threshold1_);
  config("threshold2", threshold2_);
  config("hand", active_hand_);
}

void OneGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);

  if (active_hand_ == "Left")
    opposite_hand_pose_ = ctl.rightHandTask_->surfacePose();
  else if (active_hand_ == "Right")
    opposite_hand_pose_ = ctl.leftHandTask_->surfacePose();
  else ;

  hand_surface_pose_.translation() = Eigen::Vector3d::Identity();
  hand_surface_pose_.rotation() = sva::RotY(-mc_rtc::constants::PI /2)*sva::RotX(-mc_rtc::constants::PI /2);
}

bool OneGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (remove_)
  {
    output("ReqRemove");
    return true;
  }
  if (add_)
  {
    if (active_hand_ == "Left")
    {
      activeTask_ = ctl.leftHandTask_;
    }
    else if (active_hand_ == "Right")
    {
      activeTask_ = ctl.rightHandTask_;
    }
    else ; 

    ctl.solver().addTask(activeTask_);
    activeTask_->target(pre_target_);
    add_ = false;
    step_ = 1;

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
    output("AddedHand");                                            
    return true;                                             
  }                                                          

  return false;
}

void OneGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  auto & gui = *ctl_.gui();

  gui.removeCategory({"Grasp"});
}

void OneGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"Grasp"},
                 mc_rtc::gui::ArrayInput(
                 "Target position (World) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_pos_; },
                 [this](const Eigen::Vector3d & t) { target_pos_ = t; computeTarget(); }), 
                 mc_rtc::gui::ArrayInput(
                 "Target position (Relative) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_relative_pos_; },
                 [this](const Eigen::Vector3d & t) { target_relative_pos_ = t; computeTargetRelative(); }), 
                 mc_rtc::gui::Button("Add hand", [this]() {add_ = true;}),
                 mc_rtc::gui::Button("Remove hand", [this]() {remove_ = true;}),
                 mc_rtc::gui::Transform("[pre_target]", [this]() -> const sva::PTransformd & { return pre_target_; }),
                 mc_rtc::gui::Transform("[target]", [this]() -> const sva::PTransformd & { return target_; })
                 );
}

void OneGrasp::computeTarget()
{
  target_.translation() = Eigen::Vector3d(depth_, -target_pos_.x(), target_pos_.y());
  if (active_hand_ == "Left") 
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*target_pos_.z()/180);
  else if (active_hand_ == "Right")
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1-target_pos_.z()/180));
  else ;

  pre_target_ = target_;
  pre_target_.translation().x() -= approach_depth_;
}

void OneGrasp::computeTargetRelative()
{
  target_.translation() = Eigen::Vector3d(depth_, 
              -target_relative_pos_.x()+opposite_hand_pose_.translation().y(), 
              target_relative_pos_.y()+opposite_hand_pose_.translation().z());
  if (active_hand_ == "Left") 
    target_.rotation() = opposite_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*target_relative_pos_.z()/180);
  else if (active_hand_ == "Right")
    target_.rotation() = opposite_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1-target_relative_pos_.z()/180));
  else ;

  pre_target_ = target_;
  pre_target_.translation().x() -= approach_depth_;
}

EXPORT_SINGLE_STATE("OneGrasp", OneGrasp)
