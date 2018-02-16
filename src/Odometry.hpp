//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_ODOMETRY_HPP
#define MOTORDAEMON_ODOMETRY_HPP

#include <Arduino.h>

#define CHAN_AL 22
#define CHAN_BL 23
#define CHAN_AR 24
#define CHAN_BR 25


class Odometry {


private:

    static long leftTicks;
    static long rightTicks;
    static int valueAL;
    static int valueBL;
    static int valueAR;
    static int valueBR;

    static void onTickChanALeft(void);
    static void onTickChanBLeft(void);
    static void onTickChanARight(void);
    static void onTickChanBRight(void);

public:
    Odometry();
    long getLeftValue();
    long getRightValue();

};



#endif //MOTORDAEMON_ODOMETRY_HPP
