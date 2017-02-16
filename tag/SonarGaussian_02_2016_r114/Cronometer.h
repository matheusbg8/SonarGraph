#ifndef CRONOMETER_H
#define CRONOMETER_H

#include <sys/time.h>

/**
 * @brief This is not multplataform implementation (only linux)
 *
 *  See C++11 lib chrono to implement multiplataform high resolution cronometer
 *
 */
class Cronometer
{
    struct timeval begin,end;
public:
    Cronometer();

    /**
     * @brief Return cronometer read in useconds
     *
     * @return double
     */
    double read();

    /**
     * @brief Reset cronometer and return cronometer read in useconds
     *
     * @return double
     */
    double reset();

};

/*      //  Cronometer Resolution Test

#include "Cronometer.h"
#include <iostream>
using namespace std;

int main(int argc, char* argv[])
{
    Cronometer c;
    int d = 100;
    while(d--)
    {
        cout << c.reset() << endl;
    }
    return 0;
}
*/

#endif // CRONOMETER_H
