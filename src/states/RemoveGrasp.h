#pragma once

#include <mc_control/fsm/State.h>
#include "../McTestGraspController.h"

struct RemoveGrasp : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    double approach_depth_ = 0.1;
    double threshold1_ = 0.01;
    double threshold2_ = 0.01;
    std::string active_hand_ = "Left";

    int step_ = 0;
    sva::PTransformd target_;
    sva::PTransformd pre_target_;
    sva::PTransformd active_hand_pose_;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> activeTask_ = nullptr;

    void createGui(mc_control::fsm::Controller & ctl);
    void computePreTarget();

};
