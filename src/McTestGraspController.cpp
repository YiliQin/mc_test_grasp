#include "McTestGraspController.h"
#include <mc_tasks/MetaTaskLoader.h>

McTestGraspController::McTestGraspController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("McTestGraspController init done ");

  // add tasks for solver
  comTask = mc_tasks::MetaTaskLoader::load<mc_tasks::CoMTask>(solver(), config("CoM_task"));

  leftHandTask.reset(new mc_tasks::SurfaceTransformTask("LeftGripper", robots(), robots().robotIndex(), 2.0, 500));

  rightHandTask.reset(new mc_tasks::SurfaceTransformTask("RightGripper", robots(), robots().robotIndex(), 2.0, 500));

}

bool McTestGraspController::run()
{
  return mc_control::fsm::Controller::run();
}

void McTestGraspController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


