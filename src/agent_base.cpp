/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-05-05 11:59:30
 * @LastEditTime: 2024-10-31 01:00:03
 * @FilePath: /vehicle-interaction-decision-making/src/vehicle_base.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include <spdlog/spdlog.h>

#include "agent_base.hpp"

std::shared_ptr<EnvCrossroads> AgentBase::env_ = nullptr;

void AgentBase::set_target(const State& tar) {
    if (tar.x >= -25 && tar.x <= 25 && tar.y >= -25 && tar.y <= 25) {
        target_ = tar;
    } else {
        spdlog::error("set_target error, the target range must >= -25 and <= 25 !");
    }
}

void AgentBase::set_level(int l) {
    if (l >= 0 && l < 3) {
        level_ = l;
    } else {
        spdlog::error("set_level error, the level must be >= 0 and > 3 !");
    }
}

bool AgentBase::is_get_target(void) const {
    return have_got_target_ || hypot(state_.x - target_.x, state_.y - target_.y) < 1.7;
}
