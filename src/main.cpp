
#include "MotionController.hpp"

MotionController motion;

void motionControllerWrapper()
{
    motion.mainHandler();
}


void setup()
{
    Serial.begin(115200);

    motion = MotionController();
    Timer3.attachInterrupt(motionControllerWrapper).setFrequency(1000).start();
}

void loop()
{

}