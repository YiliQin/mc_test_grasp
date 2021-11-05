#pragma once

#include <mc_control/fsm/State.h>
#include "../McTestGraspController.h"

struct ZeroGrasp : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    double depth_ = 0.6;
    double approach_depth_ = 0.1;
    double approach_duration_ = 10.0;
    double reach_duration_ = 4.0;

    int step_ = 0;
    bool add_ = false;
    std::string hand_ = "Left";
    Eigen::Vector3d target_pose_;
    sva::PTransformd target_;
    sva::PTransformd pre_target_;
    sva::PTransformd hand_surface_pose_;
    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> activeTask_ = nullptr;

    void createGui(mc_control::fsm::Controller & ctl);
    void computeTarget();
};
