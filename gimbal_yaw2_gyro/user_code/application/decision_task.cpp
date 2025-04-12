#include "decision_task.h"

//Decision decision;

void decision_task(void *pvParameters)
{
    decision.decision_init();
    while (1)
    {

    decision.remote_switch();
    decision.robot_set_control();
    vTaskDelay(DECISION_DELAY_TIME);
        /* code */
    }
    
}