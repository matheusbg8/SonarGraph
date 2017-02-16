#include "CloseLoopAnaliseResult.h"
#include <cstdio>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>


using namespace std;
using namespace cv;

CloseLoopAnaliseResult::CloseLoopAnaliseResult(const char *destPath):
    destPath(destPath)
{
}

void CloseLoopAnaliseResult::loadResultFrameInformations(const char *fileName)
{
    frResults.clear();
    cout << "Loading frame descriptions" << endl;

    FILE *f =fopen((destPath + fileName).c_str(),"r");
    if(f == 0x0)
    {
        cout << "File FrameDescriptions.csv not found" << endl;
        return;
    }
    // Ignore first line
    fscanf(f, "%*[^\n]\n", NULL);

    unsigned u, nV,nE;
    while(fscanf(f,"%u,",&u)!= -1)
    {
        fscanf(f,"%u,", &nV);
        fscanf(f,"%u", &nE);

        frResults.push_back(PUU(nV,nE));
    }
}

void CloseLoopAnaliseResult::loadResultOfMatch(const char *prefix)
{
    resultGraph.clear();
    cout << "Loading match results" << endl;

    unsigned nFrs= frResults.size();
    resultGraph.resize(nFrs);

    char str[300];
    unsigned u, v;
    float score;
    FILE *f=0x0;

    for(unsigned i =0 ; i < nFrs ; i++)
    {
        sprintf(str, "%s%04u.csv",prefix,i);

        f = fopen((destPath + str).c_str(),"r");
        if(f == 0x0)
        {
            cout << "File " << str << " not found" << endl;
            continue;
        }

        // Ignore first line (It's header)
        fscanf(f, "%*[^\n]\n", NULL);

        while(fscanf(f,"%u,",&u)!= -1)
        {
            fscanf(f,"%u,", &v);
            fscanf(f,"%g", &score);

            resultGraph[u].push_back(PUF(v,score));
        }
        fclose(f);
    }
}


void CloseLoopAnaliseResult::loadDirectResult(const char *csvFileName, unsigned nFrames)
{
    resultGraph.clear();
    cout << "Loading direct match results" << endl;

    FILE *f = fopen((destPath+ csvFileName).c_str(), "r");
    if(f == 0x0)
    {
        cout << "CloseLoopAnaliseResult::loadDirectResult file " << csvFileName << " not found!" << endl;
        return;
    }

    resultGraph.resize(nFrames);
    for(unsigned uFr = 0 ; uFr < nFrames; uFr++)
    {
        resultGraph[uFr].reserve(nFrames-uFr); // -1
        for(unsigned vFr = uFr+1 ; vFr < nFrames; vFr++)
        {
            double value;
            fscanf(f,"%lf", &value);
            if(value <0.0) value = 0.0;

            resultGraph[uFr].push_back(PUF(vFr,value));
        }
    }
    fclose(f);
}


/**
 * @brief
 *  Load gorund truth graph where each vertex
 * is one image and each edge carry the score
 * between two images.
 *  (firs - dest and second - score)
 */
void CloseLoopAnaliseResult::loadGtMatch(const char *gtFileName)
{
    cout << "Loading GT" << endl;
    gtGraph.clear();
//    GTLoopDetection
    FILE *f = fopen((destPath + gtFileName).c_str(),"r");
    if(f == 0x0)
    {
        cout << "Error to load GT results, file " << gtFileName << " not found" << endl;
        return;
    }

    unsigned u, v;
    unsigned biggestId=0;
    float score;

    gtGraph.reserve(3700);

    while(fscanf(f,"%u,",&u)!= -1)
    {
        fscanf(f,"%u,", &v);
        fscanf(f,"%g", &score);

        if(u >= gtGraph.size())
        {
            gtGraph.resize((u+1)*2);
        }
        if(v >= gtGraph.size())
        {
            gtGraph.resize((v+1)*2);
        }

        gtGraph[u].reserve(3700);
        gtGraph[u].push_back(PUF(v,score));

        if(u > biggestId)
            biggestId=u;
        if(v > biggestId)
            biggestId=v;
    }
    gtGraph.resize(biggestId+1);

    fclose(f);
}

void CloseLoopAnaliseResult::normalizeResult()
{
    cout << "Normalize" << endl;
    for(unsigned uFr = 0 ; uFr < resultGraph.size(); uFr++)
    {
        for(unsigned i = 0 ; i < resultGraph[uFr].size(); i++)
        {
            PUF &edge = resultGraph[uFr][i]; // < vFr , score >
            edge.second /= min(frResults[uFr].first,frResults[edge.first].first);
        }
    }
}

void CloseLoopAnaliseResult::normalizeResult2(unsigned maxValue)
{
    cout << "Normalize2 using maxValue = " << maxValue << endl;
    for(unsigned uFr = 0 ; uFr < resultGraph.size(); uFr++)
    {
        for(unsigned i = 0 ; i < resultGraph[uFr].size(); i++)
        {
            PUF &edge = resultGraph[uFr][i]; // < vFr , score >
            if(edge.second > maxValue) edge.second = maxValue;
            edge.second /= maxValue;
        }
    }
}

float CloseLoopAnaliseResult::findGtScore(unsigned uFr, unsigned vFr)
{
    if(uFr >= gtGraph.size())
        return 0.f; // No related data found

    for(unsigned i = 0 ; i < gtGraph[uFr].size(); i++)
    {
        if(gtGraph[uFr][i].first == vFr)
            return gtGraph[uFr][i].second;
    }

    return 0.f; // No related data found
}


/**
 * @brief Save achived results into two files
 * ResultGraph.csv - File with score between two images
 * ResultFramesInformations.csv - File with image descriptions statitics
 *
 */
void CloseLoopAnaliseResult::saveResults(const char *fileName)
{
    FILE *f = fopen((destPath + "ResultGraph.csv").c_str(),"w");
    if(f == 0x0)
    {
        cout << "Erro when saving results." << endl;
        return;
    }

    for(unsigned u =0 ; u < resultGraph.size();u++)
    {
        for(unsigned i = u+1 ; i < resultGraph[u].size(); i++)
        {
            PUF &vertex = resultGraph[u][i];
            fprintf(f,"%u,%u,%g\n",u,vertex.first,vertex.second);
        }
    }
    fclose(f);

    f = fopen((destPath + fileName).c_str(),"w");
    if(f == 0x0)
    {
        cout << "Erro when saving results." << endl;
        return;
    }

    for(unsigned fr =0 ; fr < frResults.size();fr++)
    {
        PUU &frInfo = frResults[fr];
        fprintf(f,"%u,%u,%u\n",fr,
                     frInfo.first,
                    frInfo.second);
    }
    fclose(f);

}


/**
 * @brief
 *
 * @param uFr
 */
void CloseLoopAnaliseResult::saveSplitedGTResults(unsigned uFr)
{
    char str[300];
    sprintf(str,"GTLoopDetection/GtFrame_%04u.csv", uFr);
    FILE *f = fopen( (destPath + str).c_str() ,"w");
    if(f == 0x0)
    {
        cout << "Error when saving file " << str << endl;
        return;
    }

    for(unsigned i = 0 ; i < gtGraph[uFr].size() ; i++)
    {
        unsigned vFr = gtGraph[uFr][i].first;
        float score = gtGraph[uFr][i].second;
        fprintf(f,"%u,%u,%g\n", uFr, vFr,score);
    }

    fclose(f);
}

void CloseLoopAnaliseResult::saveSplitedGTResults()
{
    for(unsigned i = 0 ;i < gtGraph.size(); i++)
    {
        saveSplitedGTResults(i);
    }
}

void CloseLoopAnaliseResult::loadAnalisyAndSave()
{
    // ====== Results stuffs ========
    // Load Frame Descriptions Statistcs
    loadResultFrameInformations("Results/ResultFramesInformations.csv");

    // Load Frame Matchs
    loadResultOfMatch("Results/LoopDetections/MatchResults_fr");

    // Normalize Results
//    normalizeResult();
    normalizeResult2(23);

    // ===== Ground Truth Stuffs ====

    // Load Ground Truth
    loadGtMatch("GTMatchs_gps_compass.csv");

    // ====== Process and save ====
    // Save GT and Result together
    saveAnalisyResults("Results/CompareResult.csv");

    // ==== Direct Result stuffs =====
//    cout << "Loading direc results" << endl;
//    loadDirectResult("deppLResults.csv",gtGraph.size());
//    cout << "Finish Loading Direc Results" << endl;

    generateGTImage();
    generateResultImage();
    generatePairFeaturesCount();

}


/**
 * @brief Save analisy results
 *
 */
void CloseLoopAnaliseResult::saveAnalisyResults(const char *fileName)
{
    unsigned nFr = frResults.size();

    FILE *f = fopen( (destPath + fileName).c_str(),"w");

    fprintf(f,"srcId,destId,gtScore,ResultScore\n");
    for(unsigned uFr = 0 ; uFr < nFr ; uFr++ )
    {
        cout << "Saving frame " << uFr << endl;
        for(unsigned i= 0; i < resultGraph[uFr].size(); i++ )
        {
            unsigned vFr = resultGraph[uFr][i].first;
            float rScore = resultGraph[uFr][i].second,
                  gScore = findGtScore(uFr, vFr);

            fprintf(f,
                    "%u,%u,%f,%f\n",uFr, vFr, rScore, gScore
                   );
        }
    }

    fclose(f);
}

void CloseLoopAnaliseResult::splitGTResults()
{
    loadGtMatch();
    saveSplitedGTResults();

}

void CloseLoopAnaliseResult::generateGTImage()
{
    unsigned nFr = gtGraph.size();
    Mat gtImage(nFr,nFr, CV_8UC1);

    for(unsigned uFr = 0; uFr < nFr ; uFr++)
    {
        for(unsigned i = 0; i < gtGraph[uFr].size(); i++)
        {
            unsigned vFr = gtGraph[uFr][i].first;
            float score = gtGraph[uFr][i].second;

            gtImage.at<uchar>(uFr,vFr) = score*255;
            gtImage.at<uchar>(vFr,uFr) = score*255;
        }
    }

    Mat colorImg;
    applyColorMap(gtImage,colorImg,COLORMAP_JET);

    imwrite("gtImage.png",gtImage);
    imwrite("gtImageColor.png",colorImg);

    namedWindow("GT Image",WINDOW_NORMAL);
    imshow("GT Image", colorImg);
    waitKey();
}

void CloseLoopAnaliseResult::generateResultImage()
{
    unsigned nFr = resultGraph.size();

    Mat resultImage(resultGraph.size(),resultGraph.size(), CV_8UC1);

    for(unsigned uFr = 0; uFr <  nFr; uFr++)
    {
        for(unsigned i = 0; i < resultGraph[uFr].size(); i++)
        {
            unsigned vFr = resultGraph[uFr][i].first;
            float score = resultGraph[uFr][i].second;

            resultImage.at<uchar>(uFr,vFr) = score*255;
            resultImage.at<uchar>(vFr,uFr) = score*255;
        }
    }

    Mat colorImg;
    applyColorMap(resultImage,colorImg,COLORMAP_JET);

    imwrite("ResultImage.png",resultImage);
    imwrite("ResultImageColor.png",colorImg);

    namedWindow("Result Image",WINDOW_NORMAL );
    imshow("Result Image", colorImg);
    waitKey();
}

void CloseLoopAnaliseResult::generatePairFeaturesCount()
{
    unsigned nFr = frResults.size(), minF=9999999,maxF=0;

    Mat frameInfImage(resultGraph.size(),resultGraph.size(), CV_8UC1);

    for(unsigned uFr = 0; uFr <  nFr; uFr++)
    {
        for(unsigned vFr = uFr+1; vFr < nFr; vFr++)
        {
            unsigned nVu = frResults[uFr].first,
                     nVv = frResults[vFr].first,
                      nFeatures = min(nVu,nVv);

            if(nFeatures <minF) minF = nFeatures;
            if(nFeatures >maxF) maxF = nFeatures;

            frameInfImage.at<uchar>(uFr,vFr) = nFeatures;
            frameInfImage.at<uchar>(vFr,uFr) = nFeatures;
        }
    }
    normalize(frameInfImage,frameInfImage,0,255,NORM_MINMAX);

    Mat colorImg;
    applyColorMap(frameInfImage,colorImg,COLORMAP_JET);

    imwrite("frameInfImage.png",frameInfImage);
    imwrite("frameInfImageColor.png",colorImg);

    FILE *f = fopen("frameInfImageNormMinMax.csv","w");
    fprintf(f,"#minFeturesCount,maxFeaturesCount\n%u,%u",minF,maxF);
    fclose(f);

    namedWindow("Frame Inf Image",WINDOW_NORMAL );
    imshow("Frame Inf Image", colorImg);
    waitKey();
}
