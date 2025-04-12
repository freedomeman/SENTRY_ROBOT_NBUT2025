/**
  *************************(C) COPYRIGHT 2021 SUMMERPRAY************************
  * @file       interact_task.c/h
  * @brief      用户交互任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-06-2021     summerpray      1. doing
  *
  @verbatim
  ==============================================================================
  ==============================================================================
 *      ┌─┐       ┌─┐
 *   ┌──┘ ┴───────┘ ┴──┐
 *   │                 │
 *   │       ───       │
 *   │  ─┬┘       └┬─  │
 *   │                 │
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘
 *                神兽保佑
 *               代码无BUG!
  ==============================================================================
  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2021 SUMMERPRAY************************
  */

#include "interact_task.h"
#include "detect_task.h"
#include "communicate.h"


#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_buzzer.h"

#ifdef __cplusplus
}
#endif

extern Communicate communicate;

void interact_task(void *pvParameters)
{
    vTaskDelay(INTERACT_TASK_INIT_TIME);

    led.init();

    while (1)
    {
//      if (communicate.robot_mode == YAW2_MODE)
//      {
        led.RGB_flow();
//        /* code */
//      }
//      if (communicate.robot_mode == YAW1_MODE)
//      {
//        
//        /* code */
//      }
//      
      
        
        vTaskDelay(INTERACT_CONTROL_TIME_MS);
    }
}
