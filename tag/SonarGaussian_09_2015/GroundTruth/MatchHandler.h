#ifndef MATCHHANDLER_H
#define MATCHHANDLER_H

#include <iostream>
#include <vector>
#include <string>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#include "Frame.h"
#include "Sonar.h"
#include "GraphMatcher/GraphMatcher.h"

// ===== DEBUG SECTION ==========
//#define MATCHHANDLER_TRACKING_DEBUG
//#define MATCHHANDLER_OPTICALFLOW_DEBUG

// ==== END DEBUG SECTION =======

class MatchHandler
{
    typedef pair<int,int> PII;
private:

    GraphMatcher gm;


    /* Find homography transform between two frames, and their match */
    void computeHomography(vector<Point2f> &matchSrc, vector<Point2f> &matchDest,
                            Mat *homography);

    void computeHomographyMatch(Frame &frSrc, Frame &frDest, Mat &homography,
                                vector<Point2f> *matchSrc, vector<Point2f> *matchDst,
                                vector<PII> *matchMap, float precision);

public:

    void spatialMatch(Frame &fr1, Frame &fr2,
                      float linkDistance,
                      Mat &bgrImg1, Mat &bgrImg2);


    // Match points
    vector<Point2f> GTMatchSrc, GTMatchDest, // Ground Truth
                    HMatchSrc, HMatchDest,   // Homography Matchs
                    OFMatchSrc, OFMatchDest, // Optical Flow Matchs
                    CMatchSrc, CMatchDest,   // Continuns Matchs
                    SPMatchSrc, SPMatchDest;   // Spatial Matchs

    // Match Vector Position and dest Gaussian ID
    // This map is used to fast acces to gaussian match given its ID
    vector< PII > GTMatchMap, HMatchMap, OFMatchMap, CMatchMap, SPMatchMap;
    // map[srcGID] = (match vector position , dest gaussian ID)
    Mat H;

    void matchWithOpticalFlow(Frame &frSrc, Frame &frDest, Mat &img1, Mat &img2);

    bool consecutiveMatch(vector<Frame> &frames, unsigned srcFrID, unsigned srcGID,
                          unsigned destFrID, int *destGID);

    void consecutiveMatch(vector<Frame> &frames, unsigned srcFrID, unsigned destFrID);

    /* Transfor a point on src frame to dst frame*/
    void transform(const Point2f &src, Point2f *dst);

    /* Create a list of gaussians ID matched */
    void HMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID);
    void OFMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID);
    void CFMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID);
    void SPMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID);

    /* Clear all matchs */
    void clearMatchs();

    /* Main method, uptade the matchs betwen frame src and frame dest */
    void loadMatchs(vector<Frame> &frames,
                    Frame &srcFr, Frame &destFr,
                    Mat &img1, Mat &img2,
                    float precision=20.f);

    MatchHandler();
    ~MatchHandler();

    bool hasHomography();

    /* Find dest position of Ground Truth Match pair, using src gaussian ID*/
    bool findGTMatch(unsigned gaussianID, Point2f *src, Point2f *dst);

    /* Find dest position of Homograpy Match pair, using src gaussian ID*/
    bool findHMatch(unsigned gaussianID, Point2f *src, Point2f *dst);

    /* Find dest position of Ground Truth Match pair, using aproximate
       position of src. If the pair match is found, src and dst are
       update with exactly position of match
    */
    bool findGTMatch(Point2f *src, Point2f *dst, float squaredPrecision);

    /* Is the same of findGTMatch, but using homograpy matchs */
    bool findHMatch(Point2f *src, Point2f *dst, float squaredPrecision);

    void createMosaicImage(Mat &srcFrame, Mat &dstFrame);

    void createBigMosaicImage(vector<Frame> &frames);

};

#endif // MATCHHANDLER_H
