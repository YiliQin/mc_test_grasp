#include "ZeroGrasp.h"

void ZeroGrasp::configure(const mc_rtc::Configuration & config)
{
  config("hand", hand_);
  config("plane_depth", depth_);
  config("approach_depth", approach_depth_);
  config("approach_duration", approach_duration_);
  config("reach_duration", reach_duration_);
  config("target_pose", target_pose_);
}

void ZeroGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);
  
  hand_surface_pose_.translation() = Eigen::Vector3d::Zero();
  hand_surface_pose_.rotation() = sva::RotY(-mc_rtc::constants::PI /2)*sva::RotX(-mc_rtc::constants::PI /2);

  // in auto_mode, automately run the process 
  if (ctl.auto_mode_)
  {
    computeTarget();
    add_ = true; 
  }

}

bool ZeroGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
 
  if (add_ == true)
  {
    if (hand_ == "Left")
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"LeftGripper", approach_duration_, 1000.0, 1000, {}));                    
      activeTask_ = ctl.leftHandTask_; 
    }
    else if (hand_ == "Right")
    {
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"RightGripper",approach_duration_, 1000.0, 1000, {}));                    
      activeTask_ = ctl.rightHandTask_; 
    }
    else ;

    add_ = false;
    step_ = 1;
    activeTask_->target(pre_target_);
    ctl.solver().addTask(activeTask_);

    return false; 
  }
  if (step_ == 1 && activeTask_->timeElapsed() == true)
  {
    if (hand_ == "Left")
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"LeftGripper", reach_duration_, 1000.0, 1000, {}));                    
      activeTask_ = ctl.leftHandTask_; 
    }
    else if (hand_ == "Right")
    {
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"RightGripper", reach_duration_, 1000.0, 1000, {}));                    
      activeTask_ = ctl.rightHandTask_; 
    }

    activeTask_->target(target_);
    ctl.solver().addTask(activeTask_);
    step_ = 2;
    return false; 
  }
  if (step_ == 2 && activeTask_->timeElapsed() == true)
  {
    step_ = 0;
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

  gui.removeCategory({"ZeroGrasp"});

}

void ZeroGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"ZeroGrasp"},
                 mc_rtc::gui::ComboInput("Choose hand", {"Left", "Right"},
                 [this]() -> const std::string & {return hand_;},
                 [this](const std::string & s) { hand_ = s; }),
                 mc_rtc::gui::ArrayInput(
                 "Target position [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_pose_; },
                 [this](const Eigen::Vector3d & t) { target_pose_ = t; computeTarget(); }), 
                 mc_rtc::gui::Button("Add hand", [this]() {add_ = true;}),
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
