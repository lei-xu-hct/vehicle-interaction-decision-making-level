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
        : name(_name), agent_param_(param), level(0), have_got_target(false) {
        target_line_converter = std::make_shared<XYSLConverter>(refline);
        state = State(0, 0, 0, 0);
        target = State(0, 0, 0, 0);
    }

    virtual ~AgentBase() {};
    void set_target(State tar);
    void set_level(int l);
    bool is_get_target(void) const;

    static void initialize(std::shared_ptr<EnvCrossroads> _env) { AgentBase::env = _env; }

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

  public:
    AgentParam agent_param_;

    static double length;
    static double width;
    static double safe_length;
    static double safe_width;
    static std::shared_ptr<EnvCrossroads> env;

    std::string name;
    State state;
    State target;
    int level;
    bool have_got_target;
    std::vector<TrackedObject> tracked_objects;
    std::shared_ptr<XYSLConverter> target_line_converter;
};
