#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include "api.h"

struct McTestGraspController_DLLAPI McTestGraspController : public mc_control::fsm::Controller
{
    McTestGraspController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
    
    // Tasks
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> leftHandTask;
		std::shared_ptr<mc_tasks::SurfaceTransformTask> rightHandTask;

private:
    mc_rtc::Configuration config_;
};
