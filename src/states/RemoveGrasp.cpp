#include "RemoveGrasp.h"

void RemoveGrasp::configure(const mc_rtc::Configuration & config)
{
  config("hand", active_hand_);
  config("approach_depth", approach_depth_);
  config("appraoch_duration", approach_duration_);
  config("reach_duration", reach_duration_);
  config("left_waypoints", left_waypoints_);
  config("right_waypoints", right_waypoints_);
}

void RemoveGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (active_hand_ == "Left")
  {
    remove_hand_pose_ = ctl.robot().surfacePose(ctl.leftHandSurface);
    target_ = ctl.left_init_pose_;
  }
  else if (active_hand_ == "Right")
  {
    remove_hand_pose_ = ctl.robot().surfacePose(ctl.rightHandSurface);
    target_ = ctl.right_init_pose_;
  }
  else ;
  
  computePreTarget();                                       

  createGui(ctl);

}

bool RemoveGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (step_ == 0)
  {
    if (active_hand_ == "Left")        
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(), ctl.leftHandSurface, reach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.leftHandTask_; 
    }                                  
    else if (active_hand_ == "Right")  
    {                                  
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(), ctl.rightHandSurface, reach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.rightHandTask_;
    }                                  
    else ;                             

    activeTask_->target(pre_target_);   
    ctl.solver().addTask(activeTask_);
    step_ = 1;                         

    return false;
  }
  if (step_ == 1 && activeTask_->timeElapsed() == true)
  {                                                          
    if (active_hand_ == "Left")        
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(), ctl.leftHandSurface, approach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.leftHandTask_; 
      activeTask_->posWaypoints(left_waypoints_);
    }                                  
    else if (active_hand_ == "Right")  
    {                                  
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(), ctl.rightHandSurface, approach_duration_, 1000.0, 1000, {}));
      activeTask_ = ctl.rightHandTask_;
      activeTask_->posWaypoints(right_waypoints_);
    }                                  
    else ;                             

    activeTask_->target(target_);                            
    ctl.solver().addTask(activeTask_);
    step_ = 2;                                               

    return false;                                            
  }                                                          
  if (step_ == 2 && activeTask_->timeElapsed() == true)
  {                                                          
    if (active_hand_ == "Left")
      output("RemovedLeft");                                            
    else if (active_hand_ == "Right")
      output("RemovedRight");
    else ;
    
    return true;                                             
  }                                                          

  return false;
}

void RemoveGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  auto & gui = *ctl_.gui();

  gui.removeCategory({"RemoveGrasp"});
}

void RemoveGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"RemoveGrasp"},
                 mc_rtc::gui::Transform("[pre_target]", [this]() -> const sva::PTransformd & { return pre_target_; }),
                 mc_rtc::gui::Transform("[target]", [this]() -> const sva::PTransformd & { return target_; })
                 );
}

void RemoveGrasp::computePreTarget()
{
  pre_target_ = remove_hand_pose_;
  pre_target_.translation().x() -= approach_depth_;
}

EXPORT_SINGLE_STATE("RemoveGrasp", RemoveGrasp)
