#pragma once

#include <mc_control/fsm/State.h>

struct TwoGrasp : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    std::string hand_to_remove_ = "Left";
    // {Move, Remove}
    std::string action_ = "Move";
    Eigen::Vector3d target_relative_;

    bool remove_left_ = false;
    bool remove_right_ = false;

    void createGui(mc_control::fsm::Controller & ctl);

};
