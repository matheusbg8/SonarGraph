#include "Compare.h"
#include <cstdio>
#include <iostream>
#include <iomanip>
using namespace std;

Compare::Compare()
{
}

Compare::~Compare()
{
}

typedef struct
{
    float tp,tn,fp,fn,acy,sen,spe,effc,ppv,npv,mCoef;
    int intersecToLoop,similarityTh;
}StatistResult;

void Compare::loadCSV(const char *csvFileName)
{
    int cellSize=10, gridSize = 100;

    FILE *f = fopen(csvFileName, "r");
    if(f == 0x0) return;

    Mat mTP(gridSize,gridSize,CV_8UC1),
        mTN(gridSize,gridSize,CV_8UC1),
        mFP(gridSize,gridSize,CV_8UC1),
        mFN(gridSize,gridSize,CV_8UC1);

    Mat mAcy= Mat(gridSize,gridSize,CV_8UC1),
        mSen(gridSize,gridSize,CV_8UC1),
        mSpe(gridSize,gridSize,CV_8UC1),
        mEffc(gridSize,gridSize,CV_8UC1),
        mPpv(gridSize,gridSize,CV_8UC1),
        mNpv(gridSize,gridSize,CV_8UC1),
        mMCoef(gridSize,gridSize,CV_8UC1);

    StatistResult sr, bestSr;

    bestSr.intersecToLoop = bestSr.similarityTh = -1;
    bestSr.tn = bestSr.tp = bestSr.acy = bestSr.sen = bestSr.spe = bestSr.effc = bestSr.ppv = bestSr.npv = bestSr.mCoef = 0.f;
    bestSr.fn = bestSr.fp = 1.f;

    while(fscanf(f,"%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                 &sr.intersecToLoop, &sr.similarityTh,
                 &sr.tp,&sr.tn, &sr.fp,&sr.fn,
                 &sr.acy,&sr.sen,&sr.spe,&sr.effc,&sr.ppv,&sr.npv,&sr.mCoef)!= -1)
    {
        sr.tp*=0.01f;sr.tn*=0.01f;sr.fp*=0.01f;sr.fn*=0.01f;
        if(sr.intersecToLoop >= gridSize || sr.similarityTh >= gridSize ||
           sr.intersecToLoop < 0 || sr.similarityTh < 0 )
        {
            cout << "Problem with matrix dimension or csv data!" << endl;
        }
        mTP.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.tp*2.55f;
        mTN.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.tn*2.55f;
        mFP.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.fp*2.55f;
        mFN.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.fn*2.55f;

        mAcy.at<uchar>(sr.intersecToLoop,sr.similarityTh) = uchar(sr.acy*2.55f);
        mSen.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.sen*2.55f;
        mSpe.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.spe*2.55f;
        mEffc.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.effc*2.55f;
        mPpv.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.ppv*2.55f;
        mNpv.at<uchar>(sr.intersecToLoop,sr.similarityTh) = sr.npv*2.55f;
        mMCoef.at<uchar>(sr.intersecToLoop,sr.similarityTh) = (sr.mCoef+1.f)*127.5f;

        if(!(sr.intersecToLoop > 5 && sr.intersecToLoop < 95 &&
           sr.similarityTh > 5 && sr.similarityTh < 95))
                continue;

        // Precision and Recall
//        if((bestSr.sen + bestSr.ppv) < (sr.sen + sr.ppv))
//            bestSr = sr;

//        // Best efficience
//        if(bestSr.effc < sr.effc)
//            bestSr = sr;

        // Best Mattew Coef
//        if(bestSr.mCoef < sr.mCoef)
//            bestSr = sr;

//        // Best TP
//        if(bestSr.tp < sr.tp)
//            bestSr = sr;

        // Best TP min FP
//        if(bestSr.tp < sr.tp && bestSr.fp > sr.fp)
//            bestSr = sr;

        // Best TP min FP equation
//        if( (bestSr.tp - bestSr.fp) < (sr.tp - sr.fp))
//            bestSr = sr;

        // Min FP
//        if(bestSr.fp > sr.fp)
//            bestSr = sr;

        // Best ppv
//        if(bestSr.ppv < sr.ppv)
//            bestSr = sr;

//        if(bestSr.tp + bestSr.tn < sr.tp + sr.tn)
//            bestSr = sr;

//        if(bestSr.sen + bestSr.ppv < sr.sen + sr.ppv)
//        {
//            bestSr = sr;
//        }

//        cout << sr.sen << " , " << sr.ppv << endl;
//        if( bestSr.sen < sr.sen && bestSr.ppv < sr.ppv)
//        {
//            cout << "Best " << bestSr.sen << " , " << bestSr.ppv << endl;
//            bestSr = sr;
//        }
        cout << bestSr.tp << " , " << bestSr.tn << " - " << endl;
        cout << sr.tp << " , " << sr.tn << endl;
//        if( bestSr.tp < sr.tp && bestSr.tn < sr.tn)
//        {
//            bestSr = sr;
//            cout << "Best " << bestSr.tp << " , " << bestSr.tn << endl;
//        }

//        if( sr.tp > 10 && sr.tn > 10 && bestSr.fn > sr.fp &&
//            bestSr.tp + bestSr.tn < sr.tp + sr.tn)
//        {
//            bestSr = sr;
//            cout << "Best " << bestSr.tp << " , " << bestSr.tn << endl;
//        }

        if( sr.tp > 10 && sr.tn > 10 &&
            bestSr.fp > sr.fp && bestSr.ppv < sr.ppv )
        {
            bestSr = sr;
            cout << "Best " << bestSr.tp << " , " << bestSr.tn << endl;
        }

//        if( sr.tp > 10 && sr.tn > 10 &&
//            bestSr.sen < sr.sen && bestSr.ppv < sr.ppv )
//        {
//            bestSr = sr;
//            cout << "Best " << bestSr.tp << " , " << bestSr.tn << endl;
//        }


    }
    fclose(f);

    int imgSize = gridSize*cellSize;

    resize(mTP,mTP,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mTN,mTN,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mFP,mFP,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mFN,mFN,Size(imgSize,imgSize),0,0,INTER_NEAREST);

    resize(mAcy,mAcy,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mSen,mSen,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mSpe,mSpe,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mEffc,mEffc,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mPpv,mPpv,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mNpv,mNpv,Size(imgSize,imgSize),0,0,INTER_NEAREST);
    resize(mMCoef,mMCoef,Size(imgSize,imgSize),0,0,INTER_NEAREST);

    normalize(mTP, mTP,0,255,NORM_MINMAX);
    applyColorMap(mTP,mTP,COLORMAP_JET);

    normalize(mTN, mTN,0,255,NORM_MINMAX);
    applyColorMap(mTN,mTN,COLORMAP_JET);

    normalize(mFP, mFP,0,255,NORM_MINMAX);
    applyColorMap(mFP,mFP,COLORMAP_JET);

    normalize(mFN, mFN,0,255,NORM_MINMAX);
    applyColorMap(mFN,mFN,COLORMAP_JET);

    applyColorMap(mAcy,mAcy,COLORMAP_JET);
    applyColorMap(mSen,mSen,COLORMAP_JET);
    applyColorMap(mSpe,mSpe,COLORMAP_JET);
    applyColorMap(mEffc,mEffc,COLORMAP_JET);
    applyColorMap(mPpv,mPpv,COLORMAP_JET);
    applyColorMap(mNpv,mNpv,COLORMAP_JET);
    applyColorMap(mMCoef,mMCoef,COLORMAP_JET);

    // Draw grid
    for(unsigned i = 0 ; i < gridSize; i++)
    {
        int lPos = i*10;

        // Horizontal lines
        line(mTP,Point(0,lPos),Point(mTP.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mTN,Point(0,lPos),Point(mTN.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mFP,Point(0,lPos),Point(mFP.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mFN,Point(0,lPos),Point(mFN.cols,lPos),Scalar(0.f,0.f,0.f));

        line(mAcy,Point(0,lPos),Point(mAcy.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mSen,Point(0,lPos),Point(mSen.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mSpe,Point(0,lPos),Point(mSpe.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mEffc,Point(0,lPos),Point(mEffc.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mPpv,Point(0,lPos),Point(mPpv.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mNpv,Point(0,lPos),Point(mNpv.cols,lPos),Scalar(0.f,0.f,0.f));
        line(mMCoef,Point(0,lPos),Point(mMCoef.cols,lPos),Scalar(0.f,0.f,0.f));

        // Vertical lines
        line(mTP,Point(lPos,0),Point(lPos,mTP.rows),Scalar(0.f,0.f,0.f));
        line(mTN,Point(lPos,0),Point(lPos,mTN.rows),Scalar(0.f,0.f,0.f));
        line(mFP,Point(lPos,0),Point(lPos,mFP.rows),Scalar(0.f,0.f,0.f));
        line(mFN,Point(lPos,0),Point(lPos,mFN.rows),Scalar(0.f,0.f,0.f));

        line(mAcy,Point(lPos,0),Point(lPos,mAcy.rows),Scalar(0.f,0.f,0.f));
        line(mSen,Point(lPos,0),Point(lPos,mSen.rows),Scalar(0.f,0.f,0.f));
        line(mSpe,Point(lPos,0),Point(lPos,mSpe.rows),Scalar(0.f,0.f,0.f));
        line(mEffc,Point(lPos,0),Point(lPos,mEffc.rows),Scalar(0.f,0.f,0.f));
        line(mPpv,Point(lPos,0),Point(lPos,mPpv.rows),Scalar(0.f,0.f,0.f));
        line(mNpv,Point(lPos,0),Point(lPos,mNpv.rows),Scalar(0.f,0.f,0.f));
        line(mMCoef,Point(lPos,0),Point(lPos,mMCoef.rows),Scalar(0.f,0.f,0.f));
    }

    // Draw position
//    int pX =0*cellSize, // Similarity threshold 34
//        pY =0*cellSize; // Intersection Loop 20
    int pX =bestSr.similarityTh*cellSize, // Similarity threshold 34
        pY =bestSr.intersecToLoop*cellSize; // Intersection Loop 20

//    rectangle(mAcy,Rect(pX,pY,cellSize,cellSize),Scalar(255,0,0),2);
//    rectangle(mSen,Rect(pX,pY,cellSize,cellSize),Scalar(255,0,0),2);
//    rectangle(mSpe,Rect(pX,pY,cellSize,cellSize),Scalar(255,0,0),2);
//    rectangle(mEffc,Rect(pX,pY,cellSize,cellSize),Scalar(255,0,0),2);
//    rectangle(mPpv,Rect(pX,pY,cellSize,cellSize),Scalar(255,0,0),2);
//    rectangle(mNpv,Rect(pX,pY,cellSize,cellSize),Scalar(255,0,0),2);
//    rectangle(mMCoef,Rect(pX,pY,cellSize,cellSize),Scalar(255,0,0),2);

    imwrite("TruePositive.png",mTP);
    imwrite("TrueNegative.png",mTN);
    imwrite("FalsePositive.png",mFP);
    imwrite("FalseNegative.png",mFN);

    imwrite("Accuracy.png",mAcy);
    imwrite("Sensibility.png",mSen);
    imwrite("Specificity.png",mSpe);
    imwrite("Efficiency.png",mEffc);
    imwrite("PositivePredition.png",mPpv);
    imwrite("NegativePredition.png",mNpv);
    imwrite("MatthewsCoef.png",mMCoef);


    imshow("TruePositive.png",mTP);
    imshow("TrueNegative.png",mTN);
    imshow("FalsePositive.png",mFP);
    imshow("FalseNegative.png",mFN);

    imshow("Accuracy.png",mAcy);
    imshow("Sensibility.png",mSen);
    imshow("Specificity.png",mSpe);
    imshow("Efficiency.png",mEffc);
    imshow("PositivePredition.png",mPpv);
    imshow("NegativePredition.png",mNpv);
    imshow("MatthewsCoef.png",mMCoef);

    cout << "Intersect Loop = " << bestSr.intersecToLoop << endl
         << "Similarity= " << bestSr.similarityTh << endl
         << "FalsePositive= " << bestSr.fp << endl
         << "TruePositive= " << bestSr.tp << endl
         << "TP - TP= " << bestSr.tp - bestSr.fp << endl
         << endl;

    waitKey();
}

