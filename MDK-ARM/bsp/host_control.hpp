/*
 * @Author: Elaina
 * @Date: 2024-07-11 16:06:19
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-19 01:11:19
 * @FilePath: \MDK-ARM\Hardware\connect.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __CONNECT_H
#define __CONNECT_H
#include "main.h"
#include <stdarg.h>
#include "string.h"
#include "stdio.h"
#include "controller.h"
#include "planner.h"
#include "bsp_usart.h"
// 上位机控制类
class HostControl_t
{
public:
    HostControl_t()
    {
    }
    HostControl_t(UART_HandleTypeDef *huart, Controller_t *controller, Planner_t *planner) : _controller(controller)
    {
        _planner = planner;
        // USARTServiceInit(_huart);

        USART_Init_Config_s init_config;
        init_config.recv_buff_size = 20;
        init_config.usart_handle = huart;
        init_config.param = this;
        init_config.module_callback = USARTCallback; // 这里传入的是静态函数,需要注意参数类型
        // { reciveCallback(_instance.recv_buff); };
        USARTRegister(&_instance, &init_config);
    }
    // 兼容c的函数指针回调
    static void USARTCallback(void *param)
    {
        // 把 void* 转回 HostControl 指针
        HostControl_t *self = static_cast<HostControl_t *>(param);

        // 调用类内的成员函数
        self->reciveCallback(self->_instance.recv_buff);
    }
    /// @brief
    /// @param data
    void reciveCallback(uint8_t *data)
    {
        static float speed[3];
        if (data[0] == 0xFF && data[13] == 0xFE)
        {
            // 四字节转浮点数
            for (int i = 0; i < 12; i += 4)
            {
                speed[i / 4] = Byte2Float(data + 1 + i);
                // 限幅
                if (speed[i / 4] > 0.8)

                {
                    speed[i / 4] = 0.8;
                }
                else if (speed[i / 4] < -0.8)
                {
                    speed[i / 4] = -0.8;
                }
            }
            _controller->set_vel_target({speed[0], speed[1], speed[2]}, false);
        }
        // 大地速度
        else if (data[0] == 0xFD && data[13] == 0xFC)
        {
            for (int i = 0; i < 12; i += 4)
            {
                speed[i / 4] = Byte2Float(data + 1 + i);
                // 限幅
                if (speed[i / 4] > 0.8)

                {
                    speed[i / 4] = 0.8;
                }
                else if (speed[i / 4] < -0.8)
                {
                    speed[i / 4] = -0.8;
                }
                _controller->set_vel_target({speed[0], speed[1], speed[2]}, true);
            }
        }
        else if (data[0] == 0xFB && data[13] == 0xFA)
        {
            for (int i = 0; i < 12; i += 4)
            {
                speed[i / 4] = Byte2Float(data + 1 + i);
                // _controller->SetClosePosition({speed[0], speed[1], speed[2]}, {0.01, 0.01, 0.02});
            }
            _planner->LoactaionCloseControl({speed[0], speed[1], speed[2]}, 0.8, {0.01, 0.01, 0.02}, false);
            // _controller->SetClosePosition({speed[0], speed[1], speed[2]}, {0.01, 0.01, 0.02});
        }
        else if (data[0] == 0xEC && data[2] == 0XCE)
        {
            task_id = (int)data[1];
        }
    }
    static float Byte2Float(uint8_t *byte)
    {
        float f;
        uint8_t *p = (uint8_t *)&f;
        p[0] = byte[0];
        p[1] = byte[1];
        p[2] = byte[2];
        p[3] = byte[3];
        return f;
    }
    int8_t task_id = -1;

private:
    USARTInstance _instance;
    Controller_t *_controller;
    Planner_t *_planner;
};

#endif