#include "OneGrasp.h"

void OneGrasp::configure(const mc_rtc::Configuration & config)
{
  config("depth", depth_);
  config("approach", approach_depth_);
  config("threshold1", threshold1_);
  config("threshold2", threshold2_);
  config("threshold3", threshold3_);
  config("hand", hand_to_add_);
}

void OneGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);

  if (hand_to_add_ == "Left")
    grasping_hand_pose_ = ctl.rightHandTask_->surfacePose();
  else if (hand_to_add_ == "Right")
    grasping_hand_pose_ = ctl.leftHandTask_->surfacePose();
  else ;

  hand_surface_pose_.translation() = Eigen::Vector3d::Identity();
  hand_surface_pose_.rotation() = sva::RotY(-mc_rtc::constants::PI /2)*sva::RotX(-mc_rtc::constants::PI /2);
}

bool OneGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (hand_to_add_ == "Left")
    grasping_hand_pose_ = ctl.rightHandTask_->surfacePose();
  else if (hand_to_add_ == "Right")
    grasping_hand_pose_ = ctl.leftHandTask_->surfacePose();
  else ;

  if (remove_)
  {
    output("ReqRemove");
    return true;
  }
  // move the grasping hand
  if (move_)
  {
    if (hand_to_add_ == "Left")
    {
      activeTask_ = ctl.rightHandTask_;
    }
    else if (hand_to_add_ == "Right")
    {
      activeTask_ = ctl.leftHandTask_;
    }
    else ; 

    activeTask_->target(grasping_hand_target_);
    move_ = false;
    step_ = 1;
  }
  if (step_ == 1 && activeTask_->eval().norm() < threshold1_)
  {
    step_ = 0;
    return false;
  }
  // add the other hand
  if (add_)
  {
    if (hand_to_add_ == "Left")
    {
      activeTask_ = ctl.leftHandTask_;
    }
    else if (hand_to_add_ == "Right")
    {
      activeTask_ = ctl.rightHandTask_;
    }
    else ; 

    activeTask_->target(pre_target_);
    add_ = false;
    step_ = 10;

    return false;
  }
  if (step_ == 10 && activeTask_->eval().norm() < threshold2_)
  {                                                          
    activeTask_->target(target_);                            
    step_ = 11;                                               

    return false;                                            
  }                                                          
  if (step_ == 11 && activeTask_->eval().norm() < threshold3_)
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
                 "Target position of added hand (World) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_pos_; },
                 [this](const Eigen::Vector3d & t) { target_pos_ = t; computeTarget(); }), 

                 mc_rtc::gui::ArrayInput(
                 "Target position of added hand (Relative) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_relative_pos_; },
                 [this](const Eigen::Vector3d & t) { target_relative_pos_ = t; computeTargetRelative(); }), 

                 mc_rtc::gui::ArrayInput(
                 "Move grasping hand (Relative) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return move_relative_pos_; },
                 [this](const Eigen::Vector3d & t) { move_relative_pos_ = t; computeGraspingHandMoveRelative(); }), 

                 mc_rtc::gui::Button("Add hand", [this]() {add_ = true;}),
                 mc_rtc::gui::Button("Remove grasping hand", [this]() {remove_ = true;}),
                 mc_rtc::gui::Button("Move grasping hand", [this]() {move_ = true;}),
                 mc_rtc::gui::Transform("[pre_target]", [this]() -> const sva::PTransformd & { return pre_target_; }),
                 mc_rtc::gui::Transform("[target]", [this]() -> const sva::PTransformd & { return target_; }),
                 mc_rtc::gui::Transform("[grasping_hand_target]", [this]() -> const sva::PTransformd & { return grasping_hand_target_; })
                 );
}

void OneGrasp::computeTarget()
{
  target_.translation() = Eigen::Vector3d(depth_, -target_pos_.x(), target_pos_.y());
  if (hand_to_add_ == "Left") 
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*target_pos_.z()/180);
  else if (hand_to_add_ == "Right")
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1-target_pos_.z()/180));
  else ;

  pre_target_ = target_;
  pre_target_.translation().x() -= approach_depth_;
}

void OneGrasp::computeTargetRelative()
{
  target_.translation() = Eigen::Vector3d(depth_, 
              -target_relative_pos_.x()+grasping_hand_pose_.translation().y(), 
              target_relative_pos_.y()+grasping_hand_pose_.translation().z());
  if (hand_to_add_ == "Left") 
    target_.rotation() = grasping_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1+target_relative_pos_.z()/180));
  else if (hand_to_add_ == "Right")
    target_.rotation() = grasping_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1-target_relative_pos_.z()/180));
  else ;

  pre_target_ = target_;
  pre_target_.translation().x() -= approach_depth_;
}

void OneGrasp::computeGraspingHandMoveRelative()
{
  grasping_hand_target_.translation() = Eigen::Vector3d(depth_, 
              -move_relative_pos_.x()+grasping_hand_pose_.translation().y(), 
              move_relative_pos_.y()+grasping_hand_pose_.translation().z());
  grasping_hand_target_.rotation() = grasping_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*move_relative_pos_.z()/180);
}

EXPORT_SINGLE_STATE("OneGrasp", OneGrasp)
