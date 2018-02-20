//
// Created by discord on 2/19/18.
//

#ifndef MOTORDAEMONSAM3X_CPU_COM_STRUCTS_H
#define MOTORDAEMONSAM3X_CPU_COM_STRUCTS_H

#define CPU_RESULT_BUFFER_SIZE 2048

struct cpu_com_result
{
    int resultCode;
    char content[CPU_RESULT_BUFFER_SIZE];
};

struct cpu_com_status
{
    double x, y;
    double angle;
    bool stop;
    long curveRadius;
    double speedL, speedR;
    char pwmL, pwmR;
    bool stopPhy, stopSoft;
};

#endif //MOTORDAEMONSAM3X_CPU_COM_STRUCTS_H
