#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/LookAtTask.h>
//#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include "api.h"

struct McTestGraspController_DLLAPI McTestGraspController : public mc_control::fsm::Controller
{
    McTestGraspController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
    
    // tasks
    std::shared_ptr<mc_tasks::CoMTask> comTask_;
    std::shared_ptr<mc_tasks::OrientationTask> bodyOrientationTask_; 
    std::shared_ptr<mc_tasks::OrientationTask> chestOrientationTask_;
    std::shared_ptr<mc_tasks::LookAtTask> lookAtHandTask_; 
    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> leftHandTask_;
		std::shared_ptr<mc_tasks::BSplineTrajectoryTask> rightHandTask_;
    // store intial left and right hands pose
    sva::PTransformd left_init_pose_;
    sva::PTransformd right_init_pose_;

private:
    mc_rtc::Configuration config_;
};
