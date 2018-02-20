
#include <lib/cpu_com_structs.h>
#include "MotionController.hpp"

MotionController motion;

char buffer[4096];
unsigned short pos = 0;

void motionControllerWrapper()
{
    motion.mainHandler();
}

void orderHandler()
{
    char* order = strtok(buffer, " ");
    struct cpu_com_result result{};

    if(!strcmp(order, "d"))
    {
        char* d = strtok(buffer, " ");
        motion.orderTranslation(strtol(d, nullptr, 10));
        free(d);
    }
    else if(!strcmp(order, "stop"))
    {
        motion.stop();
    }
    else if(!strcmp(order, "cr"))
    {
        char* d = strtok(buffer, " ");
        motion.orderCurveRadius(strtol(d, nullptr, 10));
        free(d);
    }
    else if(!strcmp(order, "setspeed"))
    {
        char* s = strtok(buffer, " ");
        motion.setSpeedTranslation(strtol(s, nullptr, 10));
        free(s);
    }
    else if(!strcmp(order, "traj"))
    {
        char * info = strtok(buffer, ";");
        Trajectory trajectory(static_cast<unsigned short>(strtol(strtok(info, ","), nullptr, 10)),
                              strtol(strtok(info, ","), nullptr, 10));

        char* point = strtok(buffer, ";");

        while(point != nullptr)
        {
            int dist = strtol(strtok(point, ","), nullptr, 10);
            Cinematic* p = new Cinematic(ABS(dist),
                                         strtol(strtok(point, ","), nullptr, 10), dist > 0);
            trajectory.setPoint(p);
        }

        trajectory.seek(0);
        motion.setTrajectory(&trajectory);
        free(info);
        free(point);
    }
    else if(!strcmp(order, "testspeed"))
    {
        char* s = strtok(buffer, " ");
        motion.testSpeed(strtol(s, nullptr, 10));
        free(s);
    }
    else if(!strcmp(order, "testpos"))
    {
        char* d = strtok(buffer, " ");
        motion.testPosition(strtol(d, nullptr, 10));
        free(d);
    }
    else if(!strcmp(order, "setangle"))
    {
        char* a = strtok(buffer, " ");
        motion.setAngle(strtod(a, nullptr));
        free(a);
    }
    else if(!strcmp(order, "setpos"))
    {
        char* x = strtok(buffer, " ");
        char* y = strtok(buffer, " ");
        motion.setPosition(strtod(x, nullptr), strtod(y, nullptr));
        free(x);
        free(y);
    }
    else if(!strcmp(order, "setConsts"))
    {
        motion.setLeftSpeedTunings(static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
        motion.setRightSpeedTunings(static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
        motion.setTranslationTunings(static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
        motion.setCurveTunings(    static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
    }
    else
    {
        result.resultCode = 1;
        sprintf(result.content, "Bad order : %s", order);
    }

    memset(buffer, 0, 4096);
    pos = 0;
    free(order);
    Serial.write((uint8_t)5);
    Serial.write((char*)(&result), sizeof(struct cpu_com_result));
    Serial.flush();
}


void setup()
{
    Serial.begin(115200);

    motion = MotionController();
    Timer3.attachInterrupt(motionControllerWrapper).setFrequency(1000).start();
}

void loop()
{
    if(Serial.available())
    {
        int c = Serial.read();

        if(c == 10)
        {
            buffer[pos++] = 0;
            orderHandler();
        }
        else
        {
            buffer[pos++] = static_cast<char>(c);
        }

    }
}

