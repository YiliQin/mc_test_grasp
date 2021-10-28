#pragma once

#include <mc_control/fsm/State.h>
#include "../McTestGraspController.h"

struct OneGrasp : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    Eigen::Vector3d target_pos_;
    sva::PTransformd target_;
    sva::PTransformd preTarget_;
    sva::PTransformd world_to_surface_;
    int step_ = 0;
    double threshold1_ = 0.01;
    double threshold2_ = 0.01;
    bool add_ = false;
    bool remove_ = false;
    std::string active_hand_ = "Left";
    std::string opposite_hand_ = "Right";
    std::string opposite_surface_ = "RightGripper";
    sva::PTransformd opposite_pose_;
    double depth_ = 0.6;
    double approachDepth_ = 0.1;

    std::shared_ptr<mc_tasks::SurfaceTransformTask> activeTask_ = nullptr;

    void createGui(mc_control::fsm::Controller & ctl);
    void computeTarget();
    void computeTargetRelative();

};
