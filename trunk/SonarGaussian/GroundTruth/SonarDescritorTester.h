#ifndef SONARDESCRITORTESTER_H
#define SONARDESCRITORTESTER_H

#include "Sonar/Sonar.h"
#include "GraphMatcher/GraphMatcher.h"

class SonarDescritorTest
{
    Sonar sn;
    GraphMatcher gm;

public:
    SonarDescritorTest();

    void matchDescriptor(vector<Point2f> &srcMatch, vector<Point2f> &destMatch,
                         Mat &imgGray16_1, Mat &imgGray16_2);

};

#endif // SONARDESCRITORTESTER_H
