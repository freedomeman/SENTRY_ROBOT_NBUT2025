#ifndef COMMUNICATe_H
#define COMMUNICATe_H

#include "cmsis_os.h"
#include "main.h"
#include "Remote_control.h"
#include "Can_receive.h"
#include "Referee.h"
#include "Config.h"

class Communicate
{
public:
    void init();

    void run();

    uint32_t ui_send_cnt;
};

extern Remote_control remote_control;
extern Can_receive can_receive;
extern Referee referee;
extern Communicate communicate;

#endif
