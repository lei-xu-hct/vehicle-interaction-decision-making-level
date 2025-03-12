/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-05-17 23:21:15
 * @LastEditTime: 2024-10-31 01:00:37
 * @FilePath: /vehicle-interaction-decision-making/include/vehicle_base.hpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#pragma once

#include <cmath>
#include <string>
#include <memory>

#include <Eigen/Core>

#include "env.hpp"
#include "utils.hpp"
#include "tracked_object.hpp"

class AgentBase {
  public:
    AgentBase(const std::string& name, const std::vector<Point>& refline, const AgentParam& param)
        : name_(name), agent_param_(param), level_(0), have_got_target_(false) {
        target_line_converter_ = std::make_shared<XYSLConverter>(refline);
        state_ = State(0, 0, 0, 0);
        target_ = State(0, 0, 0, 0);
    }

    virtual ~AgentBase() {};
    virtual void set_target(const State& tar);
    virtual void set_level(int l);
    virtual bool is_get_target(void) const;

    virtual const std::string& name() const { return name_; }

    virtual const int level() const { return level_; }
    virtual const bool have_got_target() const { return have_got_target_; }

    virtual const State& state() const { return state_; }
    virtual const State& target() const { return target_; }
    virtual const std::vector<TrackedObject>& tracked_objects() const { return tracked_objects_; }
    virtual std::shared_ptr<XYSLConverter> target_line_converter() const {
        return target_line_converter_;
    }
    virtual const Action& cur_action() const { return cur_action_; }
    virtual const StateList& excepted_traj() const { return excepted_traj_; }
    virtual const std::vector<State>& footprint() const { return footprint_; }

    // mutable
    virtual int& mutable_level() { return level_; }
    virtual bool& mutable_have_got_target() { return have_got_target_; }
    virtual State& mutable_state() { return state_; }
    virtual State& mutable_target() { return target_; }
    virtual std::vector<TrackedObject>& mutable_tracked_objects() { return tracked_objects_; }

    virtual void reset(void) {}
    virtual void excute(void) {}
    virtual void draw_vehicle(std::string draw_style = "realistic", bool fill_mode = false) {}

    virtual bool operator==(const AgentBase& other) const { return name_== other.name_; }
    virtual bool operator!=(const AgentBase& other) const { return name_ != other.name_; }

  protected:
    struct Outlook {
        int rows;
        int cols;
        int colors;
        std::vector<float> data;
    };

    std::string name_;
    double dt_;

    int level_;
    bool have_got_target_;

    State state_;
    State target_;
    Action cur_action_;
    StateList excepted_traj_;
    std::vector<State> footprint_;
    std::vector<TrackedObject> tracked_objects_;
    std::shared_ptr<XYSLConverter> target_line_converter_;

    Eigen::Matrix<double, 2, 5, Eigen::RowMajor> vehicle_box2d_;
    Eigen::Matrix<double, 2, 5, Eigen::RowMajor> safezone_;

    virtual void imshow(const Outlook& out, const State& state, std::vector<double> para) {}

  public:
    AgentParam agent_param_;
    static std::shared_ptr<EnvCrossroads> env_;

    // for simu
    double init_x_min;
    double init_x_max;
    double init_y_min;
    double init_y_max;
    double init_v_min;
    double init_v_max;
    double init_yaw;

    // for display
    Outlook outlook_;
    std::string color;
    State vis_text_pos;

  public:
    static void initialize(std::shared_ptr<EnvCrossroads> _env) { AgentBase::env_ = _env; }

    static Eigen::Matrix<double, 2, 5> get_box2d(const State& tar_offset, const AgentParam& param) {
        const double half_length = 0.5 * param.length;
        const double half_width = 0.5 * param.width;
        Eigen::Matrix<double, 2, 5, Eigen::RowMajor> vehicle;
        vehicle << -half_length, half_length, half_length, -half_length, -half_length, half_width,
            half_width, -half_width, -half_width, half_width;
        Eigen::Matrix2d rot;
        rot << cos(tar_offset.yaw), -sin(tar_offset.yaw), sin(tar_offset.yaw), cos(tar_offset.yaw);

        vehicle = rot * vehicle;
        vehicle += Eigen::Vector2d(tar_offset.x, tar_offset.y).replicate(1, 5);

        return vehicle;
    }

    /**
     * @brief Get the safezone polygon object
     * (-2, 1) —— (2, 1)
     *  |           |
     *  |           |
     *(-2, -1) —— (2, -1)
     */
    static Eigen::Matrix<double, 2, 5> get_safezone(const State& tar_offset,
                                                    const AgentParam& param) {
        const double half_safe_length = 0.5 * param.safe_length;
        const double half_safe_width = 0.5 * param.safe_width;
        Eigen::Matrix<double, 2, 5, Eigen::RowMajor> safezone;
        safezone << -half_safe_length, half_safe_length, half_safe_length, -half_safe_length,
            -half_safe_length, half_safe_width, half_safe_width, -half_safe_width, -half_safe_width,
            half_safe_width;
        Eigen::Matrix2d rot;
        rot << cos(tar_offset.yaw), -sin(tar_offset.yaw), sin(tar_offset.yaw), cos(tar_offset.yaw);

        safezone = rot * safezone;
        safezone += Eigen::Vector2d(tar_offset.x, tar_offset.y).replicate(1, 5);

        return safezone;
    }
};
