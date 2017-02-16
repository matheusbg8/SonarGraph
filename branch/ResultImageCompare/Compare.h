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
    bool computeAngDiff;

    Mat colors;

    // Orig Images
    Mat gtImg, angDiffImg, resultImg, featureImg;
    // After feature filter
    Mat f_gtImg, f_angDiffImg, f_resultImg, f_featureImg;
    // After Binarization
    Mat b_gtImg, b_resultImg;
    // Final results
    Mat finalResultImg;

    string pathDir;
    unsigned minFetures,maxFeatures;
    vector<char>enableFrames;

    int minNFeaturesTh, similarityTh, intersecToLoop;

    bool loadImags();

    void turnFramesEnable();
    void filterFeatures();
    void filterResults();
    void compareResults(float *results=0x0);

    void showImages();

    void process();

    void autoAdjust(unsigned optimizationParameter=6,unsigned minFeture=15, unsigned maxFeatures=15);


    void generateFeaturesCountBar();

    void initColors(int colorMap= COLORMAP_JET);

public:
    Compare(string pathDir="../../../../SonarGraphData/grayData/Results/Imgs/");
    ~Compare();

    void start();

    void tb_minNFeatures(int v);
    void tb_similarityTh(int v);
    void tb_intersecToLoop(int v);
};

// Trackbar callbacks
void Compare_tb_minNFeatures(int v, void *dt);
void Compare_tb_similarityTh(int v, void *dt);
void Compare_tb_intersecToLoop(int v, void *dt);


#endif // COMPARE_H
