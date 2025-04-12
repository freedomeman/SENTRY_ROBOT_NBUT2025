#ifndef __DECISION_H_
#define __DECISION_H_


#include "remote_control.h"

typedef enum
{
    ROBOT_ZERO_FORCE,
    YAW2_CTRL,
    YAW1_CTRL,
    SENTRY_CTRL,
} robot_mode_e;


class Decision
{

public:
    robot_mode_e robot_mode;

    const RC_ctrl_t   *remote_control_robot;
    RC_ctrl_t   *remote_control_robot_last;

    RC_ctrl_t   remote_sive;
    RC_ctrl_t   remote_ctrl_yaw2;
    RC_ctrl_t   remote_ctrl_yaw2_last;
    RC_ctrl_t   remote_ctrl_yaw1;
    RC_ctrl_t   remote_ctrl_yaw1_last;

    void remote_switch(void);
    void robot_set_mode(void);
    void robot_set_control(void);
    void decision_init(void);

    const RC_ctrl_t* get_yaw2_ctrl_point(void);
    RC_ctrl_t* get_yaw2_ctrl_point_last(void);
    const RC_ctrl_t* get_yaw1_ctrl_point(void);
    RC_ctrl_t* get_yaw1_ctrl_point_last(void);


};


extern Decision decision;




#endif
