//
// Created by discord on 2/19/18.
//

#ifndef MOTORDAEMONSAM3X_TRAJECTORY_H
#define MOTORDAEMONSAM3X_TRAJECTORY_H

#include "Cinematic.hpp"
#include "../../../../.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/malloc.h"

class Trajectory
{
public:

    Trajectory(unsigned short nbPoints, double totalDistance) : nbPoints(nbPoints), totalDistance(totalDistance)
    {
        trajectory = static_cast<volatile Cinematic **>(malloc(nbPoints * sizeof(Cinematic*)));
    }

    Trajectory(volatile Cinematic** trajectory, unsigned short nbPoints, double totalDistance) : nbPoints(nbPoints), trajectory(trajectory), totalDistance(totalDistance)
    {}

    ~Trajectory()
    {
        for(int i=0 ; i<nbPoints ; i++) delete trajectory[i];
        free(trajectory);
    }

    volatile unsigned short setPoint(Cinematic* point) volatile
    {
        trajectory[cursor] = point;
        return cursor++;
    }

    Cinematic* getPoint() volatile
    {
        return const_cast<Cinematic *>(trajectory[cursor++]);
    }

    Cinematic* peekPoint() volatile
    {
        return const_cast<Cinematic *>(trajectory[cursor]);
    }


    int seek(unsigned short cursor) volatile
    {
        if(cursor >= nbPoints || cursor < 0) return -1;
        else
        {
            this->cursor = cursor;
            return cursor;
        }
    }

    volatile unsigned short getCursorPosition() volatile
    {
        return this->cursor;
    }

    volatile long getTotalDistance() volatile
    {
        return this->totalDistance;
    }

    bool ended() volatile
    {
        return cursor==nbPoints;
    }


private:

    volatile Cinematic** trajectory;

    volatile unsigned short nbPoints;

    volatile unsigned short cursor = 0;

    volatile long totalDistance;

};

#endif //MOTORDAEMONSAM3X_TRAJECTORY_H
