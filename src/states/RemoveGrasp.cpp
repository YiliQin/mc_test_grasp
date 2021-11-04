#include "RemoveGrasp.h"

void RemoveGrasp::configure(const mc_rtc::Configuration & config)
{
  config("approach", approach_depth_);
  //config("threshold1", threshold1_);
  //config("threshold2", threshold2_);
  config("hand", active_hand_);
}

void RemoveGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (active_hand_ == "Left")
  {
    remove_hand_pose_ = ctl.robot().surfacePose("LeftGripper");
    target_ = ctl.left_init_pose_;
  }
  else if (active_hand_ == "Right")
  {
    remove_hand_pose_ = ctl.robot().surfacePose("RightGripper");
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
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"LeftGripper", 4.0, 1000.0, 1000, target_));
      activeTask_ = ctl.leftHandTask_; 
    }                                  
    else if (active_hand_ == "Right")  
    {                                  
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"RightGripper", 4.0, 1000.0, 1000, target_));
      activeTask_ = ctl.rightHandTask_;
    }                                  
    else ;                             

    activeTask_->target(pre_target_);   
    ctl.solver().addTask(activeTask_);
    step_ = 1;                         

    return false;
  }
  //if (step_ == 1 && activeTask_->eval().norm() < threshold1_)
  if (step_ == 1 && activeTask_->timeElapsed() == true)
  {                                                          
    if (active_hand_ == "Left")        
    {
      ctl.solver().removeTask(ctl.leftHandTask_);
      ctl.leftHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"LeftGripper", 8.0, 1000.0, 1000, target_));
      activeTask_ = ctl.leftHandTask_; 
    }                                  
    else if (active_hand_ == "Right")  
    {                                  
      ctl.solver().removeTask(ctl.rightHandTask_);
      ctl.rightHandTask_.reset(new mc_tasks::BSplineTrajectoryTask(ctl.robots(), ctl.robot().robotIndex(),"RightGripper", 8.0, 1000.0, 1000, target_));
      activeTask_ = ctl.rightHandTask_;
    }                                  
    else ;                             

    activeTask_->target(target_);                            
    ctl.solver().addTask(activeTask_);
    step_ = 2;                                               

    return false;                                            
  }                                                          
  //if (step_ == 2 && activeTask_->eval().norm() < threshold2_)
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
