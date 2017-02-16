#include "Cronometer.h"
#include <cstdio>

Cronometer::Cronometer()
{
    gettimeofday (&begin, NULL);
}

double Cronometer::read()
{
    gettimeofday (&end, NULL);
    return (end.tv_sec - begin.tv_sec) * 1000000 + end.tv_usec - begin.tv_usec;
}

double Cronometer::reset()
{
    gettimeofday (&end, NULL);
    double diff = (end.tv_sec - begin.tv_sec) * 1000000 + end.tv_usec - begin.tv_usec;
    gettimeofday (&begin, NULL);
    return diff;
}
