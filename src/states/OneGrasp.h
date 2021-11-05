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
    double plane_depth_ = 0.6;
    double approach_depth_ = 0.1;
    double approach_duration_ = 10.0;
    double reach_duration_ = 4.0;
    double move_duration_ = 5.0;
    std::string hand_to_add_ = "Left";
    // {Add, AddRelative, Move, Remove}
    std::string action_ = "Add";

    int step_ = 0;
    bool add_ = false;
    bool move_ = false;
    bool remove_ = false;
    Eigen::Vector3d target_pose_;
    Eigen::Vector3d target_relative_pose_;
    Eigen::Vector3d move_relative_pose_;
    sva::PTransformd target_;
    sva::PTransformd pre_target_;
    sva::PTransformd grasping_hand_target_;
    sva::PTransformd hand_surface_pose_;
    sva::PTransformd grasping_hand_pose_;
    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> activeTask_ = nullptr;

    void createGui(mc_control::fsm::Controller & ctl);
    void computeTarget();
    void computeTargetRelative();
    void computeHoldHandMoveRelative();

};
