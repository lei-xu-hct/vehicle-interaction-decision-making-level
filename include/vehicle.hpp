/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-05-28 01:17:14
 * @LastEditTime: 2024-10-31 01:00:41
 * @FilePath: /vehicle-interaction-decision-making/include/vehicle.hpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#pragma once

#include <set>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "matplotlibcpp.h"
#include "utils.hpp"
#include "agent_base.hpp"
#include "planner.hpp"

class Vehicle : public AgentBase {
  
  public:
    Vehicle(std::string _name,
            const YAML::Node& cfg,
            const std::vector<Point>& refline,
            const AgentParam& param);
    ~Vehicle() {}

    void reset(void) override;
    void excute(void) override;
    void draw_vehicle(std::string draw_style = "realistic", bool fill_mode = false) override;

  private:
    KLevelPlanner& planner;
   
    // display
    static int global_vehicle_idx;
    static PyObject* imshow_func;

    void imshow(const Outlook& out, const State& state, std::vector<double> para);
};

class VehicleList {
  private:
    std::vector<std::shared_ptr<Vehicle>> vehicle_list;
    std::set<std::string> vehicle_names;

  public:
    VehicleList() {
        vehicle_list.clear();
        vehicle_names.clear();
    }
    VehicleList(std::vector<std::shared_ptr<Vehicle>> vehicles) : vehicle_list(vehicles) {}
    ~VehicleList() {}

    size_t size(void) { return vehicle_list.size(); }
    bool is_all_get_target(void);
    bool is_any_collision(void);
    void push_back(std::shared_ptr<Vehicle> vehicle);
    void pop_back(void);
    void reset(void);
    void set_track_objects(void);
    void update_track_objects(void);
    std::vector<AgentBase> exclude(int ego_idx);
    std::vector<AgentBase> exclude(std::shared_ptr<Vehicle> ego);
    std::shared_ptr<Vehicle> operator[](size_t index) { return vehicle_list[index]; }
    std::shared_ptr<Vehicle> operator[](std::string name);
    auto begin() { return vehicle_list.begin(); }

    auto end() { return vehicle_list.end(); }

    auto begin() const { return vehicle_list.begin(); }

    auto end() const { return vehicle_list.end(); }
};
