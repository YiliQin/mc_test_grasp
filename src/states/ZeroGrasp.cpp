#include "ZeroGrasp.h"

void ZeroGrasp::configure(const mc_rtc::Configuration & config)
{
  config("depth", depth_);
  config("approach", approachDepth_);
  config("threshold1", threshold1_);
  config("threshold2", threshold2_);

}

void ZeroGrasp::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  createGui(ctl);
  
}

bool ZeroGrasp::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);

  if (add_)
  {
    if (hand_ == "Left")
    {
      activeTask_ = ctl.leftHandTask_; 
    }
    else if (hand_ == "Right")
    {
      activeTask_ = ctl.rightHandTask_; 
    }
    else ;

    ctl.solver().addTask(activeTask_);
    activeTask_->target(preTarget_);
    add_ = false;
    step_ = 1;
    
    return false; 
  }
  if (step_ == 1 && activeTask_->eval().norm() < threshold1_)
  {
    activeTask_->target(target_);
    step_ = 2;

    return false; 
  }
  if (step_ == 2 && activeTask_->eval().norm() < threshold2_)
  {
    if (hand_ == "Left")
      output("AddLeft");
    else if (hand_ == "Right")
      output("AddRight");
    else ;
    return true;
  }

  return false;

}

void ZeroGrasp::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<McTestGraspController &>(ctl_);
  auto & gui = *ctl_.gui();

  gui.removeCategory({"Grasp"});

}

void ZeroGrasp::createGui(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();

  gui.addElement({"Grasp"},
                 mc_rtc::gui::ComboInput("hand", {"Left", "Right"},
                 [this]() -> const std::string & {return hand_;},
                 [this](const std::string & s) { hand_ = s; }),
                 mc_rtc::gui::ArrayInput(
                 "Target position [m/deg]", {"x", "y", "theta"},
                 [this]() -> const Eigen::Vector3d & { return target_pos_; },
                 [this](const Eigen::Vector3d & t) { target_pos_ = t; computeTarget(); }), 
                 mc_rtc::gui::Button("Add", [this]() {add_ = true;}),
                 mc_rtc::gui::Transform("preTarget", [this]() -> const sva::PTransformd & { return preTarget_; }),
                 mc_rtc::gui::Transform("target", [this]() -> const sva::PTransformd & { return target_; })
                 );
}

void ZeroGrasp::computeTarget()
{
  world_to_surface_.translation() = Eigen::Vector3d::Identity();
  world_to_surface_.rotation() = sva::RotY(-M_PI/2)*sva::RotX(-M_PI/2);
  target_.translation() = Eigen::Vector3d(depth_, -target_pos_.x(), target_pos_.y());
  if (hand_ == "Left")
    target_.rotation() = world_to_surface_.rotation()*sva::RotX(M_PI*target_pos_.z()/180);
  else if (hand_ == "Right")
    target_.rotation() = world_to_surface_.rotation()*sva::RotX(M_PI*(1-target_pos_.z()/180));
  else ;
  preTarget_ = target_;
  preTarget_.translation().x() -= approachDepth_;
}

EXPORT_SINGLE_STATE("ZeroGrasp", ZeroGrasp)
