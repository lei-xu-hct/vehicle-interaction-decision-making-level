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
    AgentBase(const std::string& _name, const std::vector<Point>& refline, const AgentParam& param)
        : name(_name), agent_param_(param), level_(0), have_got_target_(false) {
        target_line_converter_ = std::make_shared<XYSLConverter>(refline);
        state = State(0, 0, 0, 0);
        target_ = State(0, 0, 0, 0);
    }

    virtual ~AgentBase() {};
    virtual void set_target(const State& tar);
    virtual void set_level(int l);
    virtual bool is_get_target(void) const;

    virtual void reset(void) {}
    virtual void excute(void) {}
    virtual void draw_vehicle(std::string draw_style = "realistic", bool fill_mode = false) {}

    virtual bool operator==(const AgentBase& other) const { return name == other.name; }
    virtual bool operator!=(const AgentBase& other) const { return name != other.name; }

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

  protected:
    struct Outlook {
        int rows;
        int cols;
        int colors;
        std::vector<float> data;
    };
    double dt_;

    Outlook outlook_;

    virtual void imshow(const Outlook& out, const State& state, std::vector<double> para) {}

  public:
    AgentParam agent_param_;
    static std::shared_ptr<EnvCrossroads> env_;

    double init_x_min;
    double init_x_max;
    double init_y_min;
    double init_y_max;
    double init_v_min;
    double init_v_max;
    double init_yaw;

    std::string name;
    State state;
    State target_;
    int level_;
    bool have_got_target_;
    std::vector<TrackedObject> tracked_objects_;
    std::shared_ptr<XYSLConverter> target_line_converter_;

    std::string color;
    Action cur_action_;
    StateList excepted_traj_;
    std::vector<State> footprint_;

    Eigen::Matrix<double, 2, 5, Eigen::RowMajor> vehicle_box2d;
    Eigen::Matrix<double, 2, 5, Eigen::RowMajor> safezone;
    State vis_text_pos;
};
