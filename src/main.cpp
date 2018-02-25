
#include <lib/cpu_com_structs.h>
#include <lib/parson.h>
#include "MotionController.hpp"

MotionController* motion;

char buffer[4096];
unsigned short pos = 0;

void motionControllerWrapper()
{
    motion->mainHandler();
}

void orderHandler()
{
    char* order = strtok(buffer, " ");

    char content[1000];
    int resultCode = 0;

    if(!strcmp(order, "d"))
    {
        char* d = strtok(buffer, " ");
        motion->orderTranslation(strtol(d, nullptr, 10));
    }
    else if(!strcmp(order, "stop"))
    {
        motion->stop();
    }
    else if(!strcmp(order, "cr"))
    {
        char* d = strtok(buffer, " ");
        motion->orderCurveRadius(strtol(d, nullptr, 10));
    }
    else if(!strcmp(order, "setspeed"))
    {
        char* s = strtok(buffer, " ");
        motion->setSpeedTranslation(strtol(s, nullptr, 10));
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
        motion->setTrajectory(&trajectory);
    }
    else if(!strcmp(order, "testspeed"))
    {
        char* s = strtok(buffer, " ");
        motion->testSpeed(strtol(s, nullptr, 10));
    }
    else if(!strcmp(order, "testpos"))
    {
        char* d = strtok(buffer, " ");
        motion->testPosition(strtol(d, nullptr, 10));
    }
    else if(!strcmp(order, "setangle"))
    {
        char* a = strtok(buffer, " ");
        motion->setAngle(strtod(a, nullptr));
    }
    else if(!strcmp(order, "setpos"))
    {
        char* x = strtok(buffer, " ");
        char* y = strtok(buffer, " ");
        motion->setPosition(strtod(x, nullptr), strtod(y, nullptr));
    }
    else if(!strcmp(order, "setConsts"))
    {
        motion->setLeftSpeedTunings(static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
        motion->setRightSpeedTunings(static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
        motion->setTranslationTunings(static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
        motion->setCurveTunings(    static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)),
                                   static_cast<float>(strtod(strtok(buffer, " "), nullptr)));
    }
    else
    {
        resultCode = 1;
        sprintf(content, "Bad order : %s", order);
    }

    memset(&buffer, 0, 4096);
    pos = 0;
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    json_object_set_number(root_object, "code", resultCode);
    json_object_set_string(root_object, "content", content);

    char *serialized_string = json_serialize_to_string(root_value);

    Serial.write((uint8_t)18);
    Serial.print(serialized_string);
    Serial.write((uint8_t)13);
//    Serial.flush();

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
}


void setup()
{
    Serial.begin(115200);

    motion = new MotionController();

    motion->init();

    Timer5.attachInterrupt(motionControllerWrapper).setFrequency(1000).start();
}

void loop()
{

    if(Serial.available() > 0)
    {
        int c = Serial.read();

        if(c == 13)
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

