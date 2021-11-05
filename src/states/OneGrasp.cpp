#include "OneGrasp.h"

void OneGrasp::configure(const mc_rtc::Configuration & config)
{
  config("hand_to_add", hand_to_add_);
  config("action", action_);
  config("plane_depth", plane_depth_);
  config("approach_depth", approach_depth_);
  config("approach_duration", approach_duration_);
  config("move_duration", move_duration_);
  config("target_pose", target_pose_);
  config("target_relative_pose", target_relative_pose_);
  config("move_relative_pose", move_relative_pose_);
}

void OneGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);

  if (hand_to_add_ == "Left")
    grasping_hand_pose_ = ctl.robot().surfacePose("RightGripper");
  else if (hand_to_add_ == "Right")
    grasping_hand_pose_ = ctl.robot().surfacePose("LeftGripper");
  else ;

  hand_surface_pose_.translation() = Eigen::Vector3d::Zero();
  hand_surface_pose_.rotation() = sva::RotY(-mc_rtc::constants::PI /2)*sva::RotX(-mc_rtc::constants::PI /2);

  if (ctl.auto_mode_)
  {
    if (action_ == "Add")
    {
      computeTarget();
      add_ = true; 
    }
    else if (action_ == "AddRelative")
    {
      computeTargetRelative();
      add_ = true;
    }
    else if (action_ == "Move")
    {
      computeHoldHandMoveRelative();
      move_ = true;
    }
    else if (action_ == "Remove")
    {
      remove_ = true;
    }
    else ;
  }

}

bool OneGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (hand_to_add_ == "Left")
    grasping_hand_pose_ = ctl.robot().surfacePose("RightGripper");
  else if (hand_to_add_ == "Right")
    grasping_hand_pose_ = ctl.robot().surfacePose("LeftGripper");
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
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"RightGripper", move_duration_, 1000.0, 1000, target_));
      activeTask_ = ctl.rightHandTask_;
    }
    else if (hand_to_add_ == "Right")
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"LeftGripper", move_duration_, 1000.0, 1000, target_));
      activeTask_ = ctl.leftHandTask_;
    }
    else ; 

    activeTask_->target(grasping_hand_target_);
    ctl.solver().addTask(activeTask_);
    move_ = false;
    step_ = 1;
  }
  if (step_ == 1 && activeTask_->timeElapsed() == true)
  {
    step_ = 0;

    if (ctl.auto_mode_)
    {
      output("MovedOK");                                            
      return true;
    }
    else 
      return false;
  }
  // add the other hand
  if (add_)
  {
    if (hand_to_add_ == "Left")
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"LeftGripper", approach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.leftHandTask_;
    }
    else if (hand_to_add_ == "Right")
    {
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"RightGripper", approach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.rightHandTask_;
    }
    else ; 

    activeTask_->target(pre_target_);
    ctl.solver().addTask(activeTask_);
    add_ = false;
    step_ = 10;

    return false;
  }
  if (step_ == 10 && activeTask_->timeElapsed() == true)
  {                                                          
    if (hand_to_add_ == "Left")
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"LeftGripper", reach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.leftHandTask_;
    }
    else if (hand_to_add_ == "Right")
    {
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"RightGripper", reach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.rightHandTask_;
    }
    else ; 

    activeTask_->target(target_);                            
    ctl.solver().addTask(activeTask_);
    step_ = 11;                                               

    return false;                                            
  }                                                          
  if (step_ == 11 && activeTask_->timeElapsed() == true)
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

  gui.removeCategory({"OneGrasp"});
}

void OneGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"OneGrasp"},
                 mc_rtc::gui::ArrayInput(
                 "Target position of added hand (World) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_pose_; },
                 [this](const Eigen::Vector3d & t) { target_pose_ = t; computeTarget(); }), 

                 mc_rtc::gui::ArrayInput(
                 "Target position of added hand (Relative) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_relative_pose_; },
                 [this](const Eigen::Vector3d & t) { target_relative_pose_ = t; computeTargetRelative(); }), 

                 mc_rtc::gui::ArrayInput(
                 "Move grasping hand (Relative) [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return move_relative_pose_; },
                 [this](const Eigen::Vector3d & t) { move_relative_pose_ = t; computeHoldHandMoveRelative(); }), 

                 mc_rtc::gui::Button("Add hand", [this]() {add_ = true;}),
                 mc_rtc::gui::Button("Remove grasping hand", [this]() {remove_ = true;}),
                 mc_rtc::gui::Button("Move grasping hand", [this]() {move_ = true;}),
                 mc_rtc::gui::Transform("[grasping_hand_target]", [this]() -> const sva::PTransformd & { return grasping_hand_target_; }),
                 mc_rtc::gui::Transform("[pre_target]", [this]() -> const sva::PTransformd & { return pre_target_; }),
                 mc_rtc::gui::Transform("[target]", [this]() -> const sva::PTransformd & { return target_; })
                 );
}

void OneGrasp::computeTarget()
{
  target_.translation() = Eigen::Vector3d(plane_depth_, -target_pose_.x(), target_pose_.y());
  if (hand_to_add_ == "Left") 
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*target_pose_.z()/180);
  else if (hand_to_add_ == "Right")
    target_.rotation() = hand_surface_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1-target_pose_.z()/180));
  else ;

  pre_target_ = target_;
  pre_target_.translation().x() -= approach_depth_;
}

void OneGrasp::computeTargetRelative()
{
  target_.translation() = Eigen::Vector3d(plane_depth_, 
              -target_relative_pose_.x()+grasping_hand_pose_.translation().y(), 
              target_relative_pose_.y()+grasping_hand_pose_.translation().z());
  if (hand_to_add_ == "Left") 
    target_.rotation() = grasping_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1+target_relative_pose_.z()/180));
  else if (hand_to_add_ == "Right")
    target_.rotation() = grasping_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*(1-target_relative_pose_.z()/180));
  else ;

  pre_target_ = target_;
  pre_target_.translation().x() -= approach_depth_;
}

void OneGrasp::computeHoldHandMoveRelative()
{
  grasping_hand_target_.translation() = Eigen::Vector3d(plane_depth_, 
              -move_relative_pose_.x()+grasping_hand_pose_.translation().y(), 
              move_relative_pose_.y()+grasping_hand_pose_.translation().z());
  grasping_hand_target_.rotation() = grasping_hand_pose_.rotation()*sva::RotX(mc_rtc::constants::PI*move_relative_pose_.z()/180);
}

EXPORT_SINGLE_STATE("OneGrasp", OneGrasp)
