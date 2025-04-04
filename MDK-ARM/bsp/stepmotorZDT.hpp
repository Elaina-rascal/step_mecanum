/**
 * @file stepmotorZDT.hpp
 * @author Elaina (1463967532@qq.com)
 * @brief 张大头步进电机控制器
 * @version 0.1
 * @date 2025-04-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef _STEPMOTORZDT_HPP
#define _STEPMOTORZDT_HPP
#include "main.h"
#include "Motor.h"
#include "stdint.h"
uint8_t Step_Pos_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
class StepMotorZDT_t : public IMotorSpeed_t
{
public:
    StepMotorZDT_t(uint8_t id, UART_HandleTypeDef *USART, bool have_pub_permission = false, uint8_t forward = 1) : IMotorSpeed_t(id), _USART(USART)
    {
        _have_pub_permission = have_pub_permission;
        _forward = forward;
    }
    void set_speed_target(float target) override;
    void update(void *param) override;
    float get_linear_speed() override;

private:
    UART_HandleTypeDef *_USART;
    float _target_speed = 0;
    uint8_t _forward = 1; // 正反转
    uint8_t _target_rpm = 0;
    float _wheel_diameter = 0.08;      // 轮子直径
    bool _have_pub_permission = false; // 是否有发布权限
    uint8_t _cmd_buffer[20] = {0};     // 命令缓冲区
};

#endif