//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_ODOMETRY_HPP
#define MOTORDAEMON_ODOMETRY_HPP





class Odometry {


private:

    static long leftTicks;
    static long rightTicks;
    static int valueAL;
    static int valueBL;
    static int valueAR;
    static int valueBR;


    static void mainWorker(char chanAL, char chanBL, char chanAR, char chanBR);

    static void onTickChanALeft(void);
    static void onTickChanBLeft(void);
    static void onTickChanARight(void);
    static void onTickChanBRight(void);
    static void get_lead(int&, char);

public:
    Odometry();
    long getLeftValue();
    long getRightValue();

};



#endif //MOTORDAEMON_ODOMETRY_HPP
