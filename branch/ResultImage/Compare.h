#ifndef COMPARE_H
#define COMPARE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace std;
using namespace cv;

class Compare
{    
public:
    Compare();
    ~Compare();

    void loadCSV(const char* csvFileName);

};

#endif // COMPARE_H
