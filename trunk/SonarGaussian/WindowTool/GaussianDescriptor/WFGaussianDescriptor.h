#ifndef WFGAUSSIANDESCRIPTOR_H
#define WFGAUSSIANDESCRIPTOR_H

#include "WindowTool/WindowTool.h"
#include "Sonar/Sonar.h"
#include "Sonar/SonarConfig/ConfigLoader.h"
#include "GaussianFrame.h"
#include "WindowTool/GaussianDescriptor/GausianDescriptorFeature.h"

class WFGaussianDescriptor : public WindowFeature
{
public:
    WFGaussianDescriptor(ConfigLoader &config);
    void loadConfigFile(ConfigLoader &config);

    GausianDescriptorFeature *_gdf;

    Sonar sonar;
    vector<GaussianFrame> frames;

    int leftSelecGaussian, rightSelecGaussian,
        hightlightLeftGaussian, hightlightRightGaussian;

    int selectGaussianOnFrame(int x,  int y , int frameID, float precision=50.f);
    void removeGaussianSelections();

    void setGDF(GausianDescriptorFeature *gdf);

    void _processImage(Mat &img16Bits, int frameId);

    // WindowFeature interface
public:
    void start();

//    Frame *newFrame(const string &fileName, unsigned frameNumber);    //Not used

    void keyPress(char c);
    void mouseEvent(int event, int x, int y);

    // Img visualization
    void renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
//    void renderFrameTogether(Mat &screen,
//                             const Scalar_<float> &el, unsigned leftFrameId,
//                             const Scalar_<float> &er, unsigned rightFrameId); // not used

    // Img processing
    void processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId);

    bool save(const char *fileName);
    bool load(const char *fileName);

    Gaussian &getGaussian(unsigned frameId, unsigned gaussianId);

    int getLeftSelectedGaussian();
    int getRighSelectedGaussian();
};

#endif // WFGAUSSIANDESCRIPTOR_H
