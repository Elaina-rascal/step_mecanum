#ifndef __PLANNER_H
#define __PLANNER_H
#include "Lib_Math.h"
#include "Lib_Common.h"
#include "controller.h"

enum PlannerMode_t
{
    OpenControl,
    CloseControl,
};
class Planner_t
{
public:
    Planner_t() = default;
    Planner_t(Controller_t *controller) : _controller(controller)
    {
        _target_odom = {0, 0, 0};
        _spline[0] = CubicSpline();
        _spline[1] = CubicSpline();
        _spline[2] = CubicSpline();
    }
    SimpleStatus_t &LoactaionOpenControl(const odom_t &target_odom, float max_v, const cmd_vel_t &target_vel = {0, 0, 0}, bool clearodom = false);
    SimpleStatus_t &LoactaionCloseControl(const odom_t &target_odom, float max_v, const odom_t &target_error = {0.1, 0.1, 0.2}, bool clearodom = false);
    void update(uint16_t dt);

private:
    float _target_t;
    uint16_t _current_t;
    odom_t start_odom;
    odom_t _target_odom;
    CubicSpline _spline[3];
    Controller_t *_controller;
    SimpleStatus_t _promise = SimpleStatus_t();
    PlannerMode_t _controlmode = PlannerMode_t::OpenControl;
};

#endif
