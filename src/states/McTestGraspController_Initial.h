#pragma once

#include <mc_control/fsm/State.h>
#include "../McTestGraspController.h"

struct McTestGraspController_Initial : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    // default is manual mode
    bool auto_mode_ = false;
    bool select_ = false;

    void createGui(mc_control::fsm::Controller & ctl);
};
