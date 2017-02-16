#include <iostream>

using namespace std;

#include "GTLoop.h"

int main()
{
    cout << "OpenCV version " << CV_VERSION << endl;

    GTLoop gt;
    gt.start();

    cout << "Hello World!" << endl;
    return 0;
}

