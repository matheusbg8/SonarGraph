#include "Compare.h"
#include <cstdio>
#include <iostream>
#include <iomanip>
using namespace std;

bool Compare::loadImags()
{
    gtImg = imread(pathDir +"gtImage.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    if(computeAngDiff)
        angDiffImg = imread(pathDir +"gtHeadingDiffImage.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    resultImg = imread(pathDir +"ResultImage.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
//    resultImg = imread(pathDir +"ResultPedro_ModeloV2_eval.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    featureImg = imread(pathDir +"frameInfImage.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

    unsigned cropPoint = 197;
    Rect crop(cropPoint,cropPoint,gtImg.rows-cropPoint, gtImg.cols-cropPoint);
    gtImg = gtImg(crop);

    if(computeAngDiff)
        angDiffImg = angDiffImg(crop);

    resultImg = resultImg(crop);
    featureImg = featureImg(crop);

    FILE *f = fopen((pathDir + "frameInfImageNormMinMax.csv").c_str(),"r");
    if(f==0x0)
    {
        cout << "Problem to read min max normalization, file frameInfImageNormMinMax.csv not found!!" << endl;
        return false;
    }
    // Ignore first line
    fscanf(f, "%*[^\n]\n", NULL);
    fscanf(f, "%u,%u\n", &minFetures, &maxFeatures);
    fclose(f);
    return true;
}

void Compare::turnFramesEnable()
{
    enableFrames.clear();
    enableFrames.resize(gtImg.rows,1);
}

void Compare::filterFeatures()
{
    turnFramesEnable();
    unsigned enableFrCount= featureImg.cols;

    if(minNFeaturesTh > minFetures)
    {
        // Normalization 0 to 255
        unsigned normMinNFeatures= ((float)(minNFeaturesTh-minFetures)/(float)(maxFeatures-minFetures))*255.f;

//        double minV,maxV;
//        minMaxLoc(featureImg,&minV,&maxV);
//        cout << "MinMax " << minV << " , " << maxV << endl;
        for(unsigned fr = 0 ; fr < featureImg.cols; fr++)
        {
            if(featureImg.at<uchar>(fr,fr) < normMinNFeatures)
            {
                enableFrames[fr]= 0;
                enableFrCount--;
            }
        }
    }

    f_gtImg = Mat(enableFrCount,enableFrCount,CV_8UC1);
    if(computeAngDiff)
        f_angDiffImg =Mat(enableFrCount,enableFrCount,CV_8UC1);
    f_resultImg = Mat(enableFrCount,enableFrCount,CV_8UC1);
    f_featureImg = Mat(enableFrCount,enableFrCount,CV_8UC1);

    unsigned uRFr=0;
    for(unsigned uFr = 0 ; uFr < featureImg.cols; uFr++)
    {
        unsigned vRFr=0;
        if(enableFrames[uFr])
        {
        for(unsigned vFr = 0 ; vFr < featureImg.cols; vFr++)
        {
            if(enableFrames[vFr])
            {
                f_gtImg.at<uchar>(uRFr,vRFr) = gtImg.at<uchar>(uFr,vFr);
                if(computeAngDiff)
                f_angDiffImg.at<uchar>(uRFr,vRFr) = angDiffImg.at<uchar>(uFr,vFr);
                f_resultImg.at<uchar>(uRFr,vRFr) = resultImg.at<uchar>(uFr,vFr);
                f_featureImg.at<uchar>(uRFr,vRFr) = featureImg.at<uchar>(uFr,vFr);
                vRFr++;
            }
        }
        uRFr++;
        }
    }
}

void Compare::filterResults()
{
    threshold( f_gtImg, b_gtImg, intersecToLoop*2.55f, 255,THRESH_BINARY);
    threshold( f_resultImg, b_resultImg, similarityTh*2.55f, 255,THRESH_BINARY);

}

void Compare::compareResults(float *results)
{
    finalResultImg = Mat(b_gtImg.rows,b_gtImg.cols,CV_8UC3);
    unsigned truePositive=0,falsePositive=0,falseNegative=0,trueNegative=0;
    for(unsigned uFr = 0 ; uFr< b_gtImg.rows; uFr++)
    {
        for(unsigned vFr = uFr ; vFr< b_gtImg.cols; vFr++)
        {
            const uchar &pGt = b_gtImg.at<uchar>(uFr,vFr),
                        &pRt = b_resultImg.at<uchar>(uFr,vFr);

            if(pGt == 0 && pRt == 0)
            {
                finalResultImg.at<Vec3b>(uFr,vFr) = Vec3b(0,0,0); // Black
                finalResultImg.at<Vec3b>(vFr,uFr) = Vec3b(0,0,0); // Black
                trueNegative++;
            }else if(pGt == pRt)
            {  // Match (correct detection)
                finalResultImg.at<Vec3b>(uFr,vFr) = Vec3b(0,255,0); // Green
                finalResultImg.at<Vec3b>(vFr,uFr) = Vec3b(0,255,0); // Green
                truePositive++;
            }else if(pGt > pRt)
            {  // Miss  (loop wasn't detected)
                finalResultImg.at<Vec3b>(uFr,vFr) = Vec3b(255,0,0); // Blue
                finalResultImg.at<Vec3b>(vFr,uFr) = Vec3b(255,0,0); // Blue
                falseNegative++;
            }else
            {  // Wrong (wrong loop detected)
                finalResultImg.at<Vec3b>(uFr,vFr) = Vec3b(0,0,255); // Red
                finalResultImg.at<Vec3b>(vFr,uFr) = Vec3b(0,0,255); // Red
                falsePositive++;
            }
        }
    }

    float total = truePositive+trueNegative+falsePositive+falseNegative,
          fTP = truePositive/total*100.f,
          fTN = trueNegative/total*100.f,
          fFP = falsePositive/total*100.f,
          fFN = falseNegative/total*100.f,
          accuracy = (fTP+fTN)/ 100.f,
          sensibility = fTP   / (fTP+fFN),
          specificity = fTN   / (fTN + fFP),
          efficiency = (sensibility+specificity)/2.f,
          positivePredition = fTP / (fTP+fFP),
          negativePredition = fTN / (fTN+fFN),
          matthewsCoef = (fTP*fTN - fFP*fFN)/sqrt((fTP+fFP)*(fTP+fFN)*(fTN+fFP)*(fTN+fFN));


    if(results!= 0x0)
    {
        results[0] = fTP;
        results[1] = fTN;
        results[2] = fFP;
        results[3] = fFN;
        results[4] = accuracy;
        results[5] = sensibility;
        results[6] = specificity;
        results[7] = efficiency;
        results[8] = positivePredition;
        results[9] = negativePredition;
        results[10] = matthewsCoef;
    }else
    {
    cout << endl
         << "Result:" << setprecision(2) << fixed << endl
         << "True Positive = " << truePositive << " (" << fTP  << "%)" << endl
         << "True Negative = " << trueNegative << " (" << fTN  << "%)" << endl
         << "False Positive = " << falsePositive << " (" << fFP  << "%)" << endl
         << "False Negative = " << falseNegative << " (" << fFN  << "%)" << endl
         << "-----------------------------" << endl
         << "Accuracy = " << accuracy*100.f << "%" << endl
         << "Sensibility = " << sensibility*100.f << "%" << endl
         << "Specificity = " << specificity*100.f << "%" << endl
         << "Efficiency = " << efficiency*100.f << "%" << endl
         << "Positive Predition = " << positivePredition*100.f << "%" << endl
         << "Negative Predition = " << negativePredition*100.f << "%" << endl
         << "Matthews Coef (phi)= " << matthewsCoef << endl
         << "-----------------------------" << endl
         << "similarityTh = " << similarityTh << "%" << endl
         << "intersecToLoop = " << intersecToLoop << "%" << endl;
    }
}

void Compare::showImages()
{
    Mat colorImg;

    applyColorMap(gtImg,colorImg,COLORMAP_JET);
    imshow("gtImg",colorImg);

    if(computeAngDiff)
    applyColorMap(angDiffImg,colorImg,COLORMAP_JET);
    imshow("angDiffImg",colorImg);

    applyColorMap(resultImg,colorImg,COLORMAP_JET);
    imshow("resultImg",colorImg);

    applyColorMap(featureImg,colorImg,COLORMAP_JET);
    imshow("featureImg",colorImg);

    applyColorMap(f_gtImg,colorImg,COLORMAP_JET);
    imshow("f_gtImg",colorImg);
    imwrite("f_gtImg.png",colorImg);

    if(computeAngDiff)
    {
    applyColorMap(f_angDiffImg,colorImg,COLORMAP_JET);
    imshow("f_angDiffImg",colorImg);
    imwrite("f_angDiffImg.png",colorImg);
    }

    applyColorMap(f_resultImg,colorImg,COLORMAP_JET);
    imshow("f_resultImg",colorImg);
    imwrite("f_resultImg.png",colorImg);

    applyColorMap(f_featureImg,colorImg,COLORMAP_JET);
    imshow("f_featureImg",colorImg);
    imwrite("f_featureImg.png",colorImg);

    for(unsigned i = 0 ; i < b_gtImg.rows; i++)
    {
        for(unsigned j = 0 ; j < b_gtImg.cols; j++)
        {
            uchar p = b_gtImg.at<uchar>(i,j);
            Vec3b &cp = colorImg.at<Vec3b>(i,j);
            if(p == 0)
            {
                cp[0] = 0; cp[1] = 0; cp[2] = 0;
            }
            else
            {
                cp[0] = 0; cp[1] = 255; cp[2] = 0;
            }
        }
    }
    imshow("b_gtImg",colorImg);
    imwrite("b_gtImg.png",colorImg);

    for(unsigned i = 0 ; i < b_resultImg.rows; i++)
    {
        for(unsigned j = 0 ; j < b_resultImg.cols; j++)
        {
            uchar p = b_resultImg.at<uchar>(i,j);
            Vec3b &cp = colorImg.at<Vec3b>(i,j);
            if(p == 0)
            {
                cp[0] = 0; cp[1] = 0; cp[2] = 0;
            }
            else
            {
                cp[0] = 0; cp[1] = 255; cp[2] = 0;
            }
        }
    }
    imshow("b_resultImg",colorImg);
    imwrite("b_resultImg.png",colorImg);

    imshow("finalResultImg",finalResultImg);
    imwrite("finalResultImg.png",finalResultImg);
}

void Compare::process()
{
    filterFeatures();
    filterResults();
    compareResults();
    showImages();
    generateFeaturesCountBar();
}

void Compare::autoAdjust(unsigned optimizationParameter, unsigned minFetures, unsigned maxFeatures)
{
    FILE *f = fopen("autoAdjust.csv", "w");

    float bestValue=0.f,
          results[11],
          &tp = results[0],
          &tn = results[1],
          &fp = results[2],
          &fn = results[3],
          &accuracy = results[4],
          &sensibility = results[5],
          &specificity = results[6],
          &efficiency = results[7],
          &positivePredition = results[8],
          &negativePredition = results[9],
          &matthewsCoef = results[10];

    int bestNFeatures=minFetures,
        bestIntersecLoop=50,
        bestSimilarityTh=50;

    fprintf(f, "intersecToLoop,similarityTh,TruePositive,TrueNegative,FalsePositive,FalseNegative,accuracy,sensibility,specificity,efficiency,positivePredition,negativePredition,matthewsCoef\n");
    for(minNFeaturesTh=minFetures; minNFeaturesTh <=maxFeatures; minNFeaturesTh++)
    {
        filterFeatures();
        for(intersecToLoop=0 ; intersecToLoop < 100 ; intersecToLoop+=1)
        {
            for(similarityTh=0; similarityTh <100; similarityTh+=1)
            {
//                cout << "Auto adjust:" << endl
//                     << "best: nFeature = " << bestNFeatures
//                     << " intersecLoop = " << bestIntersecLoop
//                     << " similarityTh = " << bestSimilarityTh << endl
//                     << "current: nFeature= " << minFetures
//                     << " intersecToLoop = " << intersecToLoop
//                     << " similarityTh = " << similarityTh << endl;

                filterResults();
                compareResults(results);
                fprintf(f, "%d,%d,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.2f\n",
                        intersecToLoop,similarityTh,
                        tp,tn,
                        fp,fn,
                        accuracy*100,sensibility*100,
                        specificity*100,efficiency*100,
                        positivePredition*100,negativePredition*100,
                        matthewsCoef);

                bool best=false;
                switch(optimizationParameter)
                {
                case 0:
                    if(accuracy > bestValue)
                    {
                        bestValue = accuracy; best = true;
                    }
                break;
                case 1:
                    if(sensibility > bestValue)
                    {
                        bestValue = sensibility; best = true;
                    }
                break;
                case 2:
                    if(specificity > bestValue)
                    {
                        bestValue = specificity; best = true;
                    }
                break;
                case 3:
                    if(efficiency > bestValue)
                    {
                        bestValue = efficiency, best = true;
                    }
                break;
                case 4:
                    if(positivePredition > bestValue)
                    {
                        bestValue = positivePredition; best = true;
                    }
                break;
                case 5:
                    if(negativePredition > bestValue)
                    {
                        bestValue = negativePredition; best = true;
                    }
                break;
                case 6:
                    if(fabs(matthewsCoef) > bestValue)
                    {
                        bestValue = fabs(matthewsCoef); best = true;
                    }
                break;
                case 7:
                    accuracy+= sensibility + specificity + efficiency + positivePredition + negativePredition + fabs(matthewsCoef);
                    if(accuracy > bestValue)
                    {
                        bestValue = accuracy; best = true;
                    }
                break;
                case 8:
                    accuracy = sensibility + positivePredition;
                    if(accuracy > bestValue)
                    {
                        bestValue = accuracy; best = true;
                    }
                break;
                }

                if(best)
                {
                    bestNFeatures = minNFeaturesTh;
                    bestIntersecLoop = intersecToLoop;
                    bestSimilarityTh = similarityTh;
                }
            }
        }
    }
    minNFeaturesTh = bestNFeatures;
    intersecToLoop = bestIntersecLoop;
    similarityTh = bestSimilarityTh;

    fclose(f);

    cout << "Auto adjust " << optimizationParameter << ":" << endl
         << "best: nFeature = " << bestNFeatures
         << " intersecLoop = " << bestIntersecLoop
         << " similarityTh = " << bestSimilarityTh << endl;
}

void Compare::generateFeaturesCountBar()
{
    Mat featuresBar(1,featureImg.cols, CV_8UC1),
        cropFeaturesBar(1,featureImg.cols, CV_8UC1);

    unsigned normMinNFeatures= ((float)(minNFeaturesTh-minFetures)/(float)(maxFeatures-minFetures))*255.f;

    for(unsigned fr = 0 ; fr < featureImg.cols; fr++)
    {
        uchar pv = featureImg.at<uchar>(fr,fr);
        featuresBar.at<uchar>(0,fr) = pv;

        if(pv < normMinNFeatures)
        {
            cropFeaturesBar.at<uchar>(0,fr) = 0u;
        }else
        {
            cropFeaturesBar.at<uchar>(0,fr) = 255u;
        }
    }

    Mat dstFeature, dstCrop;
    resize(featuresBar,dstFeature,Size(),1,20,INTER_NEAREST);
    resize(cropFeaturesBar,dstCrop,Size(),1,20,INTER_NEAREST);

    applyColorMap(dstFeature,dstFeature,COLORMAP_JET);
    applyColorMap(dstCrop,dstCrop,COLORMAP_JET);

    imwrite("FeatureBar.png", dstFeature);
    imwrite("CropFeaturesBar.png", dstCrop);

    imshow("FeatureBar", dstFeature);
    imshow("CropFeaturesBar", dstCrop);

    waitKey(0);

}

void Compare::initColors(int colorMap)
{
    colors = Mat(1,256,CV_8UC1);
    for(unsigned i = 0; i < 256; i++)
    {
        colors.at<uchar>(0,i) = i;
    }
    applyColorMap(colors,colors,colorMap);
}

Compare::Compare(string pathDir):
    pathDir(pathDir),computeAngDiff(false),
    minFetures(0), maxFeatures(255),
    minNFeaturesTh(15), similarityTh(26), intersecToLoop(6)
{
    initColors(COLORMAP_JET);
}

Compare::~Compare()
{
    destroyWindow("gtImg");
    destroyWindow("angDiffImg");
    destroyWindow("resultImg");
    destroyWindow("featureImg");
    destroyWindow("f_gtImg");

    if(computeAngDiff)
        destroyWindow("f_angDiffImg");

    destroyWindow("f_resultImg");
    destroyWindow("f_featureImg");
    destroyWindow("b_gtImg");
    destroyWindow("b_resultImg");
    destroyWindow("finalResultImg");

    destroyWindow("BarControl");
}

void Compare::start()
{
    if(!loadImags())
        return;

//    autoAdjust(0); // Accuracy
//    autoAdjust(1); // Sensibility
//    autoAdjust(2); // Specificity
//    autoAdjust(3); // Efficiency
//    autoAdjust(4); // PositivePredition
//    autoAdjust(5); // NegativePredition
//    autoAdjust(6); // MatthewsCoef
//    autoAdjust(7); // All at same time
//    autoAdjust(8); // Sensibility + PositivePredition

    namedWindow("gtImg",WINDOW_OPENGL);
    namedWindow("angDiffImg",WINDOW_OPENGL);
    namedWindow("resultImg",WINDOW_OPENGL);
    namedWindow("featureImg",WINDOW_OPENGL);
    namedWindow("f_gtImg",WINDOW_OPENGL);

    if(computeAngDiff)
        namedWindow("f_angDiffImg",WINDOW_OPENGL);

    namedWindow("f_resultImg",WINDOW_OPENGL);
    namedWindow("f_featureImg",WINDOW_OPENGL);
    namedWindow("b_gtImg",WINDOW_OPENGL);
    namedWindow("b_resultImg",WINDOW_OPENGL);
    namedWindow("finalResultImg",WINDOW_OPENGL);

    namedWindow("BarControl");

    createTrackbar("minNFeatures", "BarControl", &minNFeaturesTh, maxFeatures, Compare_tb_minNFeatures, this);
    createTrackbar("similarityTh", "BarControl", &similarityTh, 100, Compare_tb_similarityTh, this);
    createTrackbar("intersecToLoop", "BarControl", &intersecToLoop, 100, Compare_tb_intersecToLoop, this);

    process();
    waitKey();
}

void Compare::tb_minNFeatures(int v)
{
    process();
}

void Compare::tb_similarityTh(int v)
{
    process();
}

void Compare::tb_intersecToLoop(int v)
{
    process();
}


void Compare_tb_minNFeatures(int v, void *dt)
{
    Compare *c = (Compare*) dt;
    c->tb_minNFeatures(v);
}


void Compare_tb_similarityTh(int v, void *dt)
{
    Compare *c = (Compare*) dt;
    c->tb_similarityTh(v);
}


void Compare_tb_intersecToLoop(int v, void *dt)
{
    Compare *c = (Compare*) dt;
    c->tb_intersecToLoop(v);
}
