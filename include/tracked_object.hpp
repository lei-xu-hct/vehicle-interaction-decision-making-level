/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-05-26 21:14:51
 * @LastEditTime: 2024-10-31 01:00:29
 * @FilePath: /vehicle-interaction-decision-making/include/tracked_object.hpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "utils.hpp"

struct PredictTraj {
    double confidence;
    StateList traj;
};

class TrackedObject {
  public:
    TrackedObject() : name_("") {
        state_ = State(0, 0, 0, 0);
        predict_trajs_.clear();
    }
    TrackedObject(std::string _name) : name_(_name) {
        state_ = State(0, 0, 0, 0);
        predict_trajs_.clear();
    }
    ~TrackedObject() {}

    const std::string& name() const { return name_; }
    const AgentParam& agent_param() const { return agent_param_; }
    const State& state() const { return state_; }
    const State& target() const { return target_; }
    const std::vector<PredictTraj>& predict_trajs() const { return predict_trajs_; }
    std::shared_ptr<XYSLConverter> target_line_converter() const { return target_line_converter_; }

    // mutable
    std::string& mutable_name() { return name_; }
    AgentParam& mutable_agent_param() { return agent_param_; }
    State& mutable_state() { return state_; }
    State& mutable_target() { return target_; }
    std::vector<PredictTraj>& mutable_predict_trajs() { return predict_trajs_; }
    std::shared_ptr<XYSLConverter>& mutable_target_line_converter() {
        return target_line_converter_;
    }

  private:
    std::string name_;
    AgentParam agent_param_;
    State state_;
    State target_;
    std::vector<PredictTraj> predict_trajs_;

    std::shared_ptr<XYSLConverter> target_line_converter_;
};
