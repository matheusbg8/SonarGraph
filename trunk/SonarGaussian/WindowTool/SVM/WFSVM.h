#ifndef WFSVM_H
#define WFSVM_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include "WindowTool/WindowFeature.h"
#include "WindowTool/GaussianDescriptor/GausianDescriptorFeature.h"
#include "WindowTool/SVM/SVMFrame.h"

/**
 * @brief This is the image classification window feture
 * used to classify sonar objects using suport vector machine (SVM)
 *
 */
class WFSVM : public WindowFeature, public GausianDescriptorFeature
{
public:
    WFSVM();

    // ==== SVM stufs ===
    CvSVM SVM;

    float gamma, C, traningDataPercent;
    int kernel_type;
    bool normalizeData, autoTrain, trainingDataReady;
    // The vector above are only avalible after makeTrainingData call (trainingDataReady=true)
    vector<double> minVal, maxVal;

    static char classesName[10][200];
    Scalar getClassColor(float label);

    void doTraining();
    void showTrainResults(CvSVM &SVM);

    void takeMinVal(vector<double> &min, vector<double> &data);
    void takeMaxVal(vector<double> &max, vector<double> &data);


    void makeTraningData(Mat &traningLabels, Mat &trainingData,
                         Mat &validationLabels, Mat &validationData);

    double computeHitPercentage(Mat &labels, Mat &data, vector<pair<unsigned, unsigned> > &results);
    double computeHitPercentage(vector<pair<unsigned, unsigned> > &results);

    void computeVector(vector<double> &v, Gaussian &g);
    //===============

    int lastSelectedObjectId,lastSelecFrameId;
    bool showFrameInfo;
    bool svmTrained;
    Size trainResultWindowSize;

    vector<SVMFrame> frames; // Only avalible after makeTrainData call
    vector< vector<SVMObject> > allSVMByClass; // It's valid only after svmTrained= true by calling doTraining() or makeTraningData() mehods

    void renderFrame(Mat &img, int frameId);
    void _renderFrameTogether(Mat &screen,const Scalar_<float> &e,unsigned frameId);

    void alocateFrame(unsigned frameId);

    void showSVMFeatures(unsigned fremeId, unsigned svmId);


    // WindowFeature interface
public:
    void start();

//    Frame *newFrame(const string &fileName, unsigned frameNumber); // not used
    void selectedFrame(int frameId);
    void keyPress(char c);
    void mouseEvent(int event, int x, int y);
    void renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
    void renderFrameTogether(Mat &screen,
                             const Scalar_<float> &el, unsigned leftFrameId,
                             const Scalar_<float> &er, unsigned rightFrameId);
    void processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId);

    double euclidianDist(vector<double> v1, vector<double> v2);

    void searchSimilar();

    // GausianDescriptorFeature interface
public:
    void newGaussian(int gId, int frameId);
    void leftGaussianSelected(int gId, int frameId);
    void rightGaussianSeleced(int gId, int frameId);
    void cleanedGaussians(int frameId);


// Save and load config
    bool save(const char *fileName);
    bool load(const char *fileName);

    bool loadGt(const char *fileName);
    bool saveGt(const char *fileName);

    string workFileName;
    void setWorkFile(const char *fileName);
    void autoSave();
    bool autoLoad();

};

#endif // WFSVM_H
