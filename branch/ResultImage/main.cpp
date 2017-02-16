#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include "Compare.h"

using namespace std;


int main()
{
    Compare c;
    c.loadCSV("../ResultImage/autoAdjust.csv");

    return 0;
}

