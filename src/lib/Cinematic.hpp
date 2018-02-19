//
// Created by discord on 06/03/17.
//

#ifndef MOTORDAEMON_CINEMATIC_HPP
#define MOTORDAEMON_CINEMATIC_HPP

class Cinematic {
public:

    Cinematic(long rel, long cur, bool way) : relativeDistance(rel), curvePoint(cur), way(way){}

    long relativeDistance;
    long curvePoint;
    bool way;
};

#endif //MOTORDAEMON_CINEMATIC_HPP
