#include "SingleImageFeature.h"

#include <opencv2/ml/ml.hpp>

#ifndef TESTSVMFEATURE_H
#define TESTSVMFEATURE_H

#include "WindowTool/SingleImageWindow.h"

class TestSVMData
{
public:
    TestSVMData(float label,float x,float y):
        label(label),x(x),y(y){}

    float label,x,y;
};

class TestSVMFeature : public SingleImageFeature
{
public:
    TestSVMFeature();

    float currentClass;
    float gamma, C;
    int kernel_type;
    bool normalizeData, autoTrain;

    float minV1, minV2, maxV1,maxV2;

    Scalar getClassColor(float label);

    vector<TestSVMData> vs;

    CvSVM SVM;

    void autoSetMaxMinValues();
    void train();
    void showTrainResults(CvSVM &SVM);
    void reset();

    // SingleImageFeature interface
public:
    void mouseClick(int x, int y);
    void mouseDrag(int x, int y);
    void render(Mat &imgBgr);
    void keyPress(char c);
    void start();
};

#endif // TESTSVMFEATURE_H
