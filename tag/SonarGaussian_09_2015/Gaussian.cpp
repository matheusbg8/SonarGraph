#include "Gaussian.h"

void Gaussian::createGaussian(Segment *seg)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:createGaussian:: use!" << endl;
    #endif

    Mat sample = seg->result(Rect(0,0,seg->N,3)),
        mCov, mMean, eVal, eVect;

    // Calculate Covariance Matrix of the sample found
    calcCovarMatrix(sample, mCov, mMean,
                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE ,
                    CV_32F);

    // Calculate EigenValues and EigenVector of Covariance Matrix
    eigen(mCov, eVal, eVect);

    x = mMean.at<float>(1,0);
    y = mMean.at<float>(0,0);
    z = mMean.at<float>(2,0);

    dx = sqrt(eVal.at<float>(1,0));
    dy = sqrt(eVal.at<float>(0,0));
    dz = sqrt(eVal.at<float>(2,0));
    N = seg->N;

    float fAng;
    fAng = 180.0*atan2(eVect.at<float>(1,0),eVect.at<float>(1,1))/M_PI;
    if(fAng<0.f) fAng+= 360.f;
    if(fAng>180.f)fAng-=180.f;

    ang = fAng;
}

