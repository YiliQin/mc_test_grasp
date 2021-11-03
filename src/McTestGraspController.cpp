#include "McTestGraspController.h"
#include <mc_tasks/MetaTaskLoader.h>

McTestGraspController::McTestGraspController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("McTestGraspController init done ");

  // init tasks
  comTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::CoMTask>(solver(), config("CoMTask"));
  solver().addTask(comTask_);
  bodyOrientationTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::OrientationTask>(solver(), config("BodyOrientationTask"));
  solver().addTask(bodyOrientationTask_);
  chestOrientationTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::OrientationTask>(solver(), config("ChestOrientationTask"));
  solver().addTask(chestOrientationTask_);
  lookAtHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::LookAtTask>(solver(), config("LookAtHandTask"));
  solver().addTask(lookAtHandTask_);

  // store initial poses for two hand
  left_init_pose_ = robot().surfacePose("LeftGripper"); 
  right_init_pose_ = robot().surfacePose("RightGripper"); 

  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::BSplineTrajectoryTask>(solver(), config("LeftHandTask"));
  leftHandTask_->target(left_init_pose_);
  solver().addTask(leftHandTask_);
  rightHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::BSplineTrajectoryTask>(solver(), config("RightHandTask"));
  rightHandTask_->target(right_init_pose_);
  solver().addTask(rightHandTask_);

}

bool McTestGraspController::run()
{
  return mc_control::fsm::Controller::run();
}

void McTestGraspController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  comTask_->reset();
  solver().addTask(comTask_);

  bodyOrientationTask_->reset();
  solver().addTask(bodyOrientationTask_);

  chestOrientationTask_->reset();
  solver().addTask(chestOrientationTask_);

  lookAtHandTask_->reset();
  solver().addTask(lookAtHandTask_);
  
  leftHandTask_->reset();
  //solver().addTask(leftHandTask_);

  rightHandTask_->reset();
  //solver().addTask(rightHandTask_);

  mc_rtc::log::success("McTestGraspController reset done ");
}


