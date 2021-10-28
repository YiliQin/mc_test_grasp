#include "McTestGraspController.h"
#include <mc_tasks/MetaTaskLoader.h>

McTestGraspController::McTestGraspController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("McTestGraspController init done ");

  // init tasks
  comTask = mc_tasks::MetaTaskLoader::load<mc_tasks::CoMTask>(solver(), config("CoMTask"));

  leftHandTask_.reset(new mc_tasks::SurfaceTransformTask("LeftGripper", robots(), robots().robotIndex(), 10.0, 1000));

  rightHandTask_.reset(new mc_tasks::SurfaceTransformTask("RightGripper", robots(), robots().robotIndex(), 10.0, 1000));

}

bool McTestGraspController::run()
{
  return mc_control::fsm::Controller::run();
}

void McTestGraspController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  comTask->reset();
  solver().addTask(comTask);

  mc_rtc::log::success("McTestGraspController reset done ");
}


