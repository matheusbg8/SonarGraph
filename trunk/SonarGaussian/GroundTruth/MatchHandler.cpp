#include "MatchHandler.h"
#include "Sonar/Gaussian.h"

#include<queue>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"

#include "Drawing/Drawing.h"


/**
 * @brief This method compute a homography tranformation matrix
 * between two images.
 *
 * @param matchSrc - Match point on source image.
 * @param matchDest - Match points on destination image.
 * @param homography - Output homography matrix computed.
 */
void MatchHandler::computeHomography(vector<Point2f> &matchSrc, vector<Point2f> &matchDest,
                                     Mat *homography)
{
#ifdef MATCHHANDLER_TRACKING_DEBUG
    cout << "MatchHandler::computeHomography use!" << endl;
#endif

    if(matchSrc.size() < 4)
        return;

    //   Homography with 8 DoF
//    *homography = findHomography(matchSrc,matchDest,0);

    //   Homography with 6 DoF
    Mat R = estimateRigidTransform(matchSrc,matchDest,false);
    if(R.rows == 0)
    {
        cout << "Problem when calculating homography!" << endl;
        return;
    }

    *homography = Mat(3,3,R.type());
    homography->at<double>(0,0) = R.at<double>(0,0);
    homography->at<double>(0,1) = R.at<double>(0,1);
    homography->at<double>(0,2) = R.at<double>(0,2);

    homography->at<double>(1,0) = R.at<double>(1,0);
    homography->at<double>(1,1) = R.at<double>(1,1);
    homography->at<double>(1,2) = R.at<double>(1,2);

    homography->at<double>(2,0) = 0.0;
    homography->at<double>(2,1) = 0.0;
    homography->at<double>(2,2) = 1.0;

    cout << "Homograpy: " << *homography << endl;
}


void MatchHandler::computeHomographyMatch(FrameGT &frSrc, FrameGT &frDest, Mat &homography,
                                          vector<Point2f> *matchSrc, vector<Point2f> *matchDst,
                                          vector< PII > *matchMap,
                                          float precision)
{
#ifdef MATCHHANDLER_TRACKING_DEBUG
    cout << "MatchHandler::computeHomographyMatch use!" << endl;
#endif
    vector<Gaussian> &gu = frSrc.gaussians;

    matchMap->resize(gu.size(),PII(-1,-1));
    matchSrc->clear();
    matchDst->clear();

    Mat ugPoints(gu.size(),1, CV_32FC2);

    // Create Mat of points to transform from src to dest plane
    for(unsigned u = 0; u < gu.size(); u++)
    {
        ugPoints.at<Vec2f>(u,0)[0] = gu[u].x;
        ugPoints.at<Vec2f>(u,0)[1] = gu[u].y;
    }

    // Transform to src for dest plane
    perspectiveTransform(ugPoints,ugPoints,homography);

    // Find Homography matchings
    for(unsigned u = 0; u < gu.size(); u++)
    {
        int id;
        Gaussian *gv;
        gv = frDest.findGaussianPrc(ugPoints.at<Vec2f>(u,0)[0],
                                ugPoints.at<Vec2f>(u,0)[1],
                           &id,precision);

        if(gv!= 0x0)
        {
            (*matchMap)[u] = PII(matchSrc->size(),id);
            matchSrc->push_back(Point2f(gu[u].x, gu[u].y));
            matchDst->push_back(Point2f(gv->x, gv->y));
        }
    }
}


void MatchHandler::spatialMatch(FrameGT &fr1, FrameGT &fr2,
                                float linkDistance,
                                Mat &bgrImg1, Mat &bgrImg2)
{
    SonarDescritor sd1,sd2;

    // Get image segmentation
    sd1.gaussians = fr1.gaussians; // O(V) :(
    sd2.gaussians = fr2.gaussians; // O(V) :(

    // Create Grpahs
    sd1.createGraph(linkDistance); // O(V^2)
    sd2.createGraph(linkDistance); // O(V^2)

    Mat temp(bgrImg1.rows, bgrImg1.cols, CV_8UC3, Scalar(0,0,0));
    bgrImg2.copyTo(temp);

//    Drawing::drawDescriptorColor(bgrImg1,&sd1,Scalar(0,255,0),Scalar(0,255,0));
//    Drawing::drawDescriptorColor(bgrImg2,&sd2,Scalar(0,255,0),Scalar(0,255,0));


    Drawing::drawGaussians(temp,sd2.gaussians,Scalar(0,0,255),Scalar(0,255,0),true,false,true);
    Drawing::drawDescriptorColor(temp,&sd2,Scalar(255,255,0),Scalar(255,255,0));
//    resize(temp,temp,Size(900,500));
    imshow("Graph", temp);

    // Compute matchs

    // Debug mode
//    vector<MatchInfoExtended> matchInfo;
//    gm.findMatchDebug(&sd1,&sd2,matchInfo);

//    Drawing::drawGraphVertexMatchs(bgrImg1,bgrImg2,
//                                    sd1,sd2,
//                                    matchInfo,2);

    vector<MatchInfo> matchInfo;
    gm.findMatch(&sd1,&sd2,matchInfo);

    // Clear old matchs
    SPMatchMap.resize(sd1.gaussians.size(),PII(-1,-1));
    SPMatchSrc.clear();
    SPMatchDest.clear();

    vector<Gaussian> &gs1 = sd1.gaussians,
                     &gs2 = sd2.gaussians;

    for(unsigned i = 0 ; i < matchInfo.size() ; i++)
    {
        unsigned gu = matchInfo[i].uID,
                 gv = matchInfo[i].vID;

        Gaussian &g1 = gs1[gu],
                 &g2 = gs2[gv];

        SPMatchMap[gu] = PII(SPMatchDest.size(),gv);

        SPMatchSrc.push_back(Point2f(g1.x, g1.y));
        SPMatchDest.push_back(Point2f(g2.x, g2.y));
    }
}


void MatchHandler::matchWithOpticalFlow(FrameGT &frSrc, FrameGT &frDest,
                                        Mat &img1, Mat &img2)
{
#ifdef MATCHHANDLER_TRACKING_DEBUG
    cout << "MatchHandler::matchWithOpticalFlow use!" << endl;
#endif
    Mat img8bits1, img8bits2;

    vector<Gaussian> &g1 = frSrc.gaussians;

    img1.convertTo(img8bits1,CV_8UC1,1,-200);
    img2.convertTo(img8bits2,CV_8UC1,1,-200);

//    imshow("img1" , img8bits1);
//    imshow("img2" , img8bits2);


    // TermCriteria 20 iteractions or error less than 0.03
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 40, 0.003);
    Size subPixWinSize(11,11),winSize(21,21);
    int maxPixelCount=25;

//    OFMatchSrc.resize(g1.size());
//    OFMatchDest.clear();

//    // Copy source points
//    for(unsigned i = 0 ; i < g1.size(); i++)
//    {
//        OFMatchSrc[i].x = g1[i].x,
//        OFMatchSrc[i].y = g1[i].y;
//    }

//     automatic find best pixels to track
    OFMatchSrc.resize(g1.size());
    OFMatchDest.clear();
    goodFeaturesToTrack(img8bits1, OFMatchSrc, maxPixelCount,
                        0.01, 10, noArray(), 3, 0, 0.04);

    // Find subpixel (ajust points to corners and
    // find pixel position inside of pixel using image gradient)
    // Search insde a window (10,10) where will be use (20,20) pixel window
    cornerSubPix(img8bits1, OFMatchSrc,
                  subPixWinSize, cvSize(-1,-1), //(-1,-1) indicate no zeroZone (dead zone)
                  termcrit);

    vector<uchar> status; // Indicate if was found pixel match
    vector<float> err; // Error of the match
    calcOpticalFlowPyrLK(img8bits1, img8bits2,
                         OFMatchSrc, OFMatchDest,
                         status, err, winSize,
                         6, // max pyramid level used to search big movies
                         termcrit,
                         0, // No FLAGs
                         0.1 // eigen Threshold (used to filter bad flow points)
                        );

    // Filter points found (status = true)
    size_t i, k;
    for( i = k = 0; i < OFMatchDest.size(); i++ )
    {
        #ifdef MATCHHANDLER_OPTICALFLOW_DEBUG
        cout << "Erro " << err[i]
             << " point " << OFMatchSrc[i]
             << " to " << OFMatchDest[i]
             << " diff " << OFMatchDest[i] - OFMatchSrc[i]
             << endl;
        #endif

        if( !status[i] )
        {
        #ifdef MATCHHANDLER_OPTICALFLOW_DEBUG
            cout << "Point was killed!" << endl;
        #endif
            continue;
        }

        OFMatchSrc[k] = OFMatchSrc[i];
        OFMatchDest[k++] = OFMatchDest[i];
    }

    Mat H;
    computeHomography(OFMatchSrc,OFMatchDest,&H);

    OFMatchSrc.clear();
    OFMatchDest.clear();

    if(H.rows == 3)
    {
        /// Do nothing homography test
//        H = Mat(3,3,CV_32FC1,Scalar(0.f));
//        H.at<float>(0,0) = 1.f;
//        H.at<float>(1,1) = 1.f;
//        H.at<float>(2,2) = 1.f;

        computeHomographyMatch(frSrc,frDest,H,&OFMatchSrc,&OFMatchDest,&OFMatchMap,10.f);
    }
}


void MatchHandler::consecutiveMatch(vector<FrameGT> &frames, unsigned srcFrID, unsigned destFrID)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::consecutiveMatch use!" << endl;
    #endif

    vector<Gaussian> &srcGs = frames[srcFrID].gaussians;
    vector<Gaussian> &destGs = frames[destFrID].gaussians;

    CMatchSrc.clear();
    CMatchDest.clear();
    CMatchMap.resize(srcGs.size(),PII(-1,-1));

    int srcGID,destGID;
    for(srcGID = 0; srcGID < srcGs.size(); srcGID++)
    {
        if(consecutiveMatch(frames,srcFrID,srcGID,
                                  destFrID,&destGID))
        {
            CMatchMap[srcGID] = PII(CMatchSrc.size() , destGID);

            CMatchSrc.push_back(Point2f(srcGs[srcGID].x, srcGs[srcGID].y));
            CMatchDest.push_back(Point2f(destGs[destGID].x, destGs[destGID].y));
        }
    }
}


bool MatchHandler::consecutiveMatch(vector<FrameGT> &frames, unsigned srcFrID, unsigned srcGID,
                                    unsigned destFrID, int *destGID)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::consecutiveMatch use!" << endl;
    #endif

    typedef pair<unsigned,unsigned> PUU;

    // FrameID, GaussianID
    queue<PUU> q;

    // Add first gaussian
    q.push(PUU(srcFrID,srcGID));

    unsigned uG, uF, vG, vF;
    while(!q.empty())
    {
        uF = q.front().first;
        uG = q.front().second;
        q.pop();

        vector< vector<PUU> > &match = frames[uF].match;

        for(unsigned m = 0 ; m < match[uG].size(); m++)
        {
            vF = match[uG][m].first;
            vG = match[uG][m].second;
            if(vF == destFrID)
            {
                *destGID = vG;
                return true;
            }else
            {
                q.push(PUU(vF,vG));
            }
        }
    }

    *destGID = -1;
    return false;
}



void MatchHandler::transform(const Point2f &src, Point2f *dst)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::transform use!" << endl;
    #endif

    vector<Point2f> mPosL(1,src);
    if(H.rows>2)
    {
        perspectiveTransform(mPosL,mPosL, H);
    }
    *dst = mPosL[0];
}

void MatchHandler::HMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::HMatchIDList use!" << endl;
    #endif

    for(unsigned u = 0 ; u < HMatchMap.size();u++)
    {
        if(HMatchMap[u].second != -1)
        {
            srcGaussiansID->push_back(u);
            destGaussiansID->push_back(HMatchMap[u].second);
        }
    }
}

void MatchHandler::OFMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::OFMatchIDList use!" << endl;
    #endif

    for(unsigned u = 0 ; u < OFMatchMap.size();u++)
    {
        if(OFMatchMap[u].second != -1)
        {
            srcGaussiansID->push_back(u);
            destGaussiansID->push_back(OFMatchMap[u].second);
        }
    }
}

void MatchHandler::CFMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::CFMatchIDList use!" << endl;
    #endif

    for(unsigned u = 0 ; u < CMatchMap.size();u++)
    {
        if(CMatchMap[u].second != -1)
        {
            srcGaussiansID->push_back(u);
            destGaussiansID->push_back(CMatchMap[u].second);
        }
    }
}

void MatchHandler::SPMatchIDList(vector<unsigned> *srcGaussiansID, vector<unsigned> *destGaussiansID)
{
#ifdef MATCHHANDLER_TRACKING_DEBUG
    cout << "MatchHandler::SPMatchIDList use!" << endl;
#endif

    for(unsigned u = 0 ; u < SPMatchMap.size();u++)
    {
        if(SPMatchMap[u].second != -1)
        {
            srcGaussiansID->push_back(u);
            destGaussiansID->push_back(SPMatchMap[u].second);
        }
    }
}

void MatchHandler::clearMatchs()
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::clearMatchs use!" << endl;
    #endif

    // Ground Trtuh Match
    GTMatchSrc.clear(); GTMatchDest.clear();
    GTMatchMap.clear();

    // Homography Match
    HMatchSrc.clear(); HMatchDest.clear();
    HMatchMap.clear();

    // Optical Flow Matchs
    OFMatchSrc.clear(); OFMatchDest.clear();
    OFMatchMap.clear();

    // Consecutive Feature Matchs
    CMatchSrc.clear(); CMatchDest.clear();
    CMatchMap.clear();

    // Spatial Matchs
    SPMatchSrc.clear(); SPMatchDest.clear();
    SPMatchMap.clear();

}

void MatchHandler::loadMatchs(FrameGT &srcFr, FrameGT &destFr, Mat &img1, Mat &img2, float precision)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::loadMatchs use!" << endl;
    #endif

    clearMatchs();

    vector< vector< pair<unsigned,unsigned> > >
            &matchs = srcFr.match;

    vector<Gaussian> &gSrc = srcFr.gaussians,
                     &gDest = destFr.gaussians;

//    cout << "Match size = " << matchs.size() << " gaussian size = " << gSrc.size() << endl;

    // Looking for ground truth matchs
    GTMatchMap.resize(gSrc.size(),PII(-1,-1));
    for(unsigned u = 0 ; u < matchs.size(); u++)
    {
        for(unsigned i = 0 ; i < matchs[u].size(); i++)
        {
            if(destFr.frameNumber == matchs[u][i].first)
            {
                unsigned v = matchs[u][i].second;
                GTMatchMap[u] = PII(GTMatchSrc.size(), v);
                GTMatchSrc.push_back(Point2f(gSrc[u].x, gSrc[u].y));
                GTMatchDest.push_back(Point2f(gDest[v].x, gDest[v].y));
            }
        }
    }

    if(GTMatchSrc.size() >= 4)
    {
        // Calculating homography from ground truth matchs
        computeHomography(GTMatchSrc,GTMatchDest,&H);

        // Search homography matchs
        computeHomographyMatch(srcFr,destFr,H,
                               &HMatchSrc,&HMatchDest,&HMatchMap,
                               precision);
    }


//    matchWithOpticalFlow(srcFr,destFr,img1,img2);

//    consecutiveMatch(frames,srcFr.frameNumber, destFr.frameNumber);
}

MatchHandler::MatchHandler(ConfigLoader &config):
    gm(config)
{

}

MatchHandler::~MatchHandler()
{
#ifdef MATCHHANDLER_TRACKING_DEBUG
    cout << "MatchHandler:Destructor:: using destructor!" << endl;
#endif
}


bool MatchHandler::hasHomography()
{
#ifdef MATCHHANDLER_TRACKING_DEBUG
    cout << "MatchHandler::hasHomography use!" << endl;
#endif

    if(HMatchSrc.size() > 0)
        return true;
    return false;
}

bool MatchHandler::findGTMatch(unsigned gaussianID, Point2f *src, Point2f *dst)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::findGTMatch use!" << endl;
    #endif

    if(gaussianID < GTMatchMap.size() && GTMatchMap[gaussianID].first !=-1)
    {
        *src = GTMatchSrc[GTMatchMap[gaussianID].first];
        *dst = GTMatchDest[GTMatchMap[gaussianID].first];
        return true;
    }
    return false;
}

bool MatchHandler::findHMatch(unsigned gaussianID, Point2f *src, Point2f *dst)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::findHMatch use!" << endl;
    #endif

    if(gaussianID < HMatchMap.size() && HMatchMap[gaussianID].first !=-1)
    {
        *src = HMatchSrc[HMatchMap[gaussianID].first];
        *dst = HMatchDest[HMatchMap[gaussianID].first];
        return true;
    }
    return false;
}

bool MatchHandler::findGTMatch(Point2f *src, Point2f *dst, float precision)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::findGTMatch use!" << endl;
    #endif

    for(unsigned i = 0; i < GTMatchSrc.size(); i++)
    {
        if(fabs(GTMatchSrc[i].x - src->x) <= precision
           && fabs(GTMatchSrc[i].y - src->y) <= precision)
        {
            *src = GTMatchSrc[i];
            *dst = GTMatchDest[i];
            return true;
        }
    }
    return false;
}

bool MatchHandler::findHMatch(Point2f *src, Point2f *dst,  float precision)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::findHMatch use!" << endl;
    #endif

    for(unsigned i = 0; i < HMatchSrc.size(); i++)
    {
        if(fabs(HMatchSrc[i].x - src->x) <= precision
           && fabs(HMatchSrc[i].y - src->y) <= precision)
        {
            *src = HMatchSrc[i];
            *dst = HMatchDest[i];
            return true;
        }
    }
    return false;
}

void MatchHandler::createMosaicImage(Mat &srcFrame, Mat &dstFrame)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::createMosaicImage use!" << endl;
    #endif

    if(!hasHomography())
        return;

    Mat result;

    warpPerspective( srcFrame, result, H, Size(srcFrame.cols*1.3,srcFrame.rows*1.3));

//    addWeighted(dstFrame,0.5,result,0.5,0,result);

    for(unsigned i = 0 ; i < dstFrame.rows; i++)
    {
        for(unsigned j = 0 ; j < dstFrame.cols; j++)
        {
            if(result.at<uchar>(i,j) > 0 && dstFrame.at<uchar>(i,j) > 0)
                result.at<uchar>(i,j) = (result.at<uchar>(i,j) + dstFrame.at<uchar>(i,j)) / 2;
            else if(dstFrame.at<uchar>(i,j) > 0)
                result.at<uchar>(i,j) = dstFrame.at<uchar>(i,j);
        }
    }
    imshow("Mosaic", result);

}

void MatchHandler::createColorMosaicImage(Mat &srcFrame, Mat &dstFrame)
{
    #ifdef MATCHHANDLER_TRACKING_DEBUG
        cout << "MatchHandler::createColorMosaicImage use!" << endl;
    #endif

    if(!hasHomography())
        return;

    Mat result;

    warpPerspective( srcFrame, result, H, Size(srcFrame.cols*1.3,srcFrame.rows*1.3));

    for(unsigned i = 0 ; i < dstFrame.rows; i++)
    {
        for(unsigned j = 0 ; j < dstFrame.cols; j++)
        {
            Vec3b &p = dstFrame.at<Vec3b>(i,j),
                  &r = result.at<Vec3b>(i,j);

            if(p[0] == 0 && p[1] == 0 && p[2] == 0)
                continue;

            r[0] = (r[0] + p[0])/2;
            r[1] = (r[1] + p[1])/2;
            r[2] = (r[2] + p[2])/2;
        }
    }
    imshow("Color Mosaic", result);
}

// Calculating Rotation and Translation between two images by Vlad stack oveflow
//// rotation and translation in 2D from point correspondences
//void rigidTransform2D(const int N) {

//// Algorithm: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf

//const bool debug = false;      // print more debug info
//const bool add_noise = true; // add noise to imput and output
//srand(time(NULL));           // randomize each time

///*********************************
//* Creat data with some noise
//**********************************/

//// Simulated transformation
//Point2f T(1.0f, -2.0f);
//float a = 30.0; // [-180, 180], see atan2(y, x)
//float noise_level = 0.1f;
//cout<<"True parameters: rot = "<<a<<"deg., T = "<<T<<
//       "; noise level = "<<noise_level<<endl;

//// noise
//vector<Point2f> noise_src(N), noise_dst(N);
//for (int i=0; i<N; i++) {
//   noise_src[i] = Point2f(randf(noise_level), randf(noise_level));
//   noise_dst[i] = Point2f(randf(noise_level), randf(noise_level));
//}

//// create data with noise
//vector<Point2f> src(N), dst(N);
//float Rdata = 10.0f; // radius of data
//float cosa = cos(a*DEG2RAD);
//float sina = sin(a*DEG2RAD);
//for (int i=0; i<N; i++) {

//   // src
//   float x1 = randf(Rdata);
//   float y1 = randf(Rdata);
//   src[i] = Point2f(x1,y1);
//   if (add_noise)
//       src[i] += noise_src[i];

//   // dst
//   float x2 = x1*cosa - y1*sina;
//   float y2 = x1*sina + y1*cosa;
//   dst[i] = Point2f(x2,y2) + T;
//   if (add_noise)
//       dst[i] += noise_dst[i];

//   if (debug)
//       cout<<i<<": "<<src[i]<<"---"<<dst[i]<<endl;
//}

//// Calculate data centroids
//Scalar centroid_src = mean(src);
//Scalar centroid_dst = mean(dst);
//Point2f center_src(centroid_src[0], centroid_src[1]);
//Point2f center_dst(centroid_dst[0], centroid_dst[1]);
//if (debug)
//   cout<<"Centers: "<<center_src<<", "<<center_dst<<endl;

///*********************************
//* Visualize data
//**********************************/

//// Visualization
//namedWindow("data", 1);
//float w = 400, h = 400;
//Mat Mdata(w, h, CV_8UC3); Mdata = Scalar(0);
//Point2f center_img(w/2, h/2);

//float scl = 0.4*min(w/Rdata, h/Rdata); // compensate for noise
//scl/=sqrt(2); // compensate for rotation effect
//Point2f dT = (center_src+center_dst)*0.5; // compensate for translation

//for (int i=0; i<N; i++) {
//   Point2f p1(scl*(src[i] - dT));
//   Point2f p2(scl*(dst[i] - dT));
//   // invert Y axis
//   p1.y = -p1.y; p2.y = -p2.y;
//   // add image center
//   p1+=center_img; p2+=center_img;
//   circle(Mdata, p1, 1, Scalar(0, 255, 0));
//   circle(Mdata, p2, 1, Scalar(0, 0, 255));
//   line(Mdata, p1, p2, Scalar(100, 100, 100));

//}

///*********************************
//* Get 2D rotation and translation
//**********************************/

//markTime();

//// subtract centroids from data
//for (int i=0; i<N; i++) {
//   src[i] -= center_src;
//   dst[i] -= center_dst;
//}

//// compute a covariance matrix
//float Cxx = 0.0, Cxy = 0.0, Cyx = 0.0, Cyy = 0.0;
//for (int i=0; i<N; i++) {
//   Cxx += src[i].x*dst[i].x;
//   Cxy += src[i].x*dst[i].y;
//   Cyx += src[i].y*dst[i].x;
//   Cyy += src[i].y*dst[i].y;
//}
//Mat Mcov = (Mat_<float>(2, 2)<<Cxx, Cxy, Cyx, Cyy);
//if (debug)
//   cout<<"Covariance Matrix "<<Mcov<<endl;

//// SVD
//cv::SVD svd;
//svd = SVD(Mcov, SVD::FULL_UV);
//if (debug) {
//   cout<<"U = "<<svd.u<<endl;
//   cout<<"W = "<<svd.w<<endl;
//   cout<<"V transposed = "<<svd.vt<<endl;
//}

//// rotation = V*Ut
//Mat V = svd.vt.t();
//Mat Ut = svd.u.t();
//float det_VUt = determinant(V*Ut);
//Mat W = (Mat_<float>(2, 2)<<1.0, 0.0, 0.0, det_VUt);
//float rot[4];
//Mat R_est(2, 2, CV_32F, rot);
//R_est = V*W*Ut;
//if (debug)
//   cout<<"Rotation matrix: "<<R_est<<endl;

//float cos_est = rot[0];
//float sin_est = rot[2];
//float ang = atan2(sin_est, cos_est);

//// translation = mean_dst - R*mean_src
//Point2f center_srcRot = Point2f(
//       cos_est*center_src.x - sin_est*center_src.y,
//       sin_est*center_src.x + cos_est*center_src.y);
//Point2f T_est = center_dst - center_srcRot;

//// RMSE
//double RMSE = 0.0;
//for (int i=0; i<N; i++) {
//   Point2f dst_est(
//           cos_est*src[i].x - sin_est*src[i].y,
//           sin_est*src[i].x + cos_est*src[i].y);
//   RMSE += SQR(dst[i].x - dst_est.x) + SQR(dst[i].y - dst_est.y);
//}
//if (N>0)
//   RMSE = sqrt(RMSE/N);

//// Final estimate msg
//cout<<"Estimate = "<<ang*RAD2DEG<<"deg., T = "<<T_est<<"; RMSE = "<<RMSE<<endl;

//// show image
//printTime(1);
//imshow("data", Mdata);
//waitKey(-1);

//return;
//} // rigidTransform2D()

//// --------------------------- 3DOF

//// calculates squared error from two point mapping; assumes rotation around Origin.
//inline float sqErr_3Dof(Point2f p1, Point2f p2,
//       float cos_alpha, float sin_alpha, Point2f T) {

//   float x2_est = T.x + cos_alpha * p1.x - sin_alpha * p1.y;
//   float y2_est = T.y + sin_alpha * p1.x + cos_alpha * p1.y;
//   Point2f p2_est(x2_est, y2_est);
//   Point2f dp = p2_est-p2;
//   float sq_er = dp.dot(dp); // squared distance

//   //cout<<dp<<endl;
//   return sq_er;
//}

//// calculate RMSE for point-to-point metrics
//float RMSE_3Dof(const vector<Point2f>& src, const vector<Point2f>& dst,
//       const float* param, const bool* inliers, const Point2f center) {

//   const bool all_inliers = (inliers==NULL); // handy when we run QUADRTATIC will all inliers
//   unsigned int n = src.size();
//   assert(n>0 && n==dst.size());

//   float ang_rad = param[0];
//   Point2f T(param[1], param[2]);
//   float cos_alpha = cos(ang_rad);
//   float sin_alpha = sin(ang_rad);

//   double RMSE = 0.0;
//   int ninliers = 0;
//   for (unsigned int i=0; i<n; i++) {
//       if (all_inliers || inliers[i]) {
//           RMSE += sqErr_3Dof(src[i]-center, dst[i]-center, cos_alpha, sin_alpha, T);
//           ninliers++;
//       }
//   }

//   //cout<<"RMSE = "<<RMSE<<endl;
//   if (ninliers>0)
//       return sqrt(RMSE/ninliers);
//   else
//       return LARGE_NUMBER;
//}

//// Sets inliers and returns their count
//inline int setInliers3Dof(const vector<Point2f>& src, const vector <Point2f>& dst,
//       bool* inliers,
//       const float* param,
//       const float max_er,
//       const Point2f center) {

//   float ang_rad = param[0];
//   Point2f T(param[1], param[2]);

//   // set inliers
//   unsigned int ninliers = 0;
//   unsigned int n = src.size();
//   assert(n>0 && n==dst.size());

//   float cos_ang = cos(ang_rad);
//   float sin_ang = sin(ang_rad);
//   float max_sqErr = max_er*max_er; // comparing squared values

//   if (inliers==NULL) {

//       // just get the number of inliers (e.g. after QUADRATIC fit only)
//       for (unsigned int i=0; i<n; i++) {

//           float sqErr = sqErr_3Dof(src[i]-center, dst[i]-center, cos_ang, sin_ang, T);
//           if ( sqErr < max_sqErr)
//               ninliers++;
//       }
//   } else  {

//       // get the number of inliers and set them (e.g. for RANSAC)
//       for (unsigned int i=0; i<n; i++) {

//           float sqErr = sqErr_3Dof(src[i]-center, dst[i]-center, cos_ang, sin_ang, T);
//           if ( sqErr < max_sqErr) {
//               inliers[i] = 1;
//               ninliers++;
//           } else {
//               inliers[i] = 0;
//           }
//       }
//   }

//   return ninliers;
//}

//// fits 3DOF (rotation and translation in 2D) with least squares.
//float fit3DofQUADRATICold(const vector<Point2f>& src, const vector<Point2f>& dst,
//       float* param, const bool* inliers, const Point2f center) {

//   const bool all_inliers = (inliers==NULL); // handy when we run QUADRTATIC will all inliers
//   unsigned int n = src.size();
//   assert(dst.size() == n);

//   // count inliers
//   int ninliers;
//   if (all_inliers) {
//       ninliers = n;
//   } else {
//       ninliers = 0;
//       for (unsigned int i=0; i<n; i++){
//           if (inliers[i])
//               ninliers++;
//       }
//   }

//   // under-dermined system
//   if (ninliers<2) {
//       //      param[0] = 0.0f; // ?
//       //      param[1] = 0.0f;
//       //      param[2] = 0.0f;
//       return LARGE_NUMBER;
//   }

//   /*
//    * x1*cosx(a)-y1*sin(a) + Tx = X1
//    * x1*sin(a)+y1*cos(a) + Ty = Y1
//    *
//    * approximation for small angle a (radians) sin(a)=a, cos(a)=1;
//    *
//    * x1*1 - y1*a + Tx = X1
//    * x1*a + y1*1 + Ty = Y1
//    *
//    * in matrix form M1*h=M2
//    *
//    *  2n x 4       4 x 1   2n x 1
//    *
//    * -y1 1 0 x1  *   a   =  X1
//    *  x1 0 1 y1      Tx     Y1
//    *                 Ty
//    *                 1=Z
//    *  ----------------------------
//    *  src1         res      src2
//    */

//   //  4 x 1
//   float res_ar[4]; // alpha, Tx, Ty, 1
//   Mat res(4, 1, CV_32F, res_ar); // 4 x 1

//   // 2n x 4
//   Mat src1(2*ninliers, 4, CV_32F); // 2n x 4

//   // 2n x 1
//   Mat src2(2*ninliers, 1, CV_32F); // 2n x 1: [X1, Y1, X2, Y2, X3, Y3]'

//   for (unsigned int i=0, row_cnt = 0; i<n; i++) {

//       // use inliers only
//       if (all_inliers || inliers[i]) {

//           float x = src[i].x - center.x;
//           float y = src[i].y - center.y;

//           // first row

//           // src1
//           float* rowPtr = src1.ptr<float>(row_cnt);
//           rowPtr[0] = -y;
//           rowPtr[1] = 1.0f;
//           rowPtr[2] = 0.0f;
//           rowPtr[3] = x;

//           // src2
//           src2.at<float> (0, row_cnt) = dst[i].x - center.x;

//           // second row
//           row_cnt++;

//           // src1
//           rowPtr = src1.ptr<float>(row_cnt);
//           rowPtr[0] = x;
//           rowPtr[1] = 0.0f;
//           rowPtr[2] = 1.0f;
//           rowPtr[3] = y;

//           // src2
//           src2.at<float> (0, row_cnt) = dst[i].y - center.y;
//       }
//   }

//   cv::solve(src1, src2, res, DECOMP_SVD);

//   // estimators
//   float alpha_est;
//   Point2f T_est;

//   // original
//   alpha_est = res.at<float>(0, 0);
//   T_est = Point2f(res.at<float>(1, 0), res.at<float>(2, 0));

//   float Z = res.at<float>(3, 0);
//   if (abs(Z-1.0) > 0.1) {
//       //cout<<"Bad Z in fit3DOF(), Z should be close to 1.0 = "<<Z<<endl;
//       //return LARGE_NUMBER;
//   }
//   param[0] = alpha_est; // rad
//   param[1] = T_est.x;
//   param[2] = T_est.y;

//   // calculate RMSE
//   float RMSE = RMSE_3Dof(src, dst, param, inliers, center);
//   return RMSE;
//} // fit3DofQUADRATICOLd()

//// fits 3DOF (rotation and translation in 2D) with least squares.
//float fit3DofQUADRATIC(const vector<Point2f>& src_, const vector<Point2f>& dst_,
//       float* param, const bool* inliers, const Point2f center) {

//   const bool debug = false;                   // print more debug info
//   const bool all_inliers = (inliers==NULL);   // handy when we run QUADRTATIC will all inliers
//   assert(dst_.size() == src_.size());
//   int N = src_.size();

//   // collect inliers
//   vector<Point2f> src, dst;
//   int ninliers;
//   if (all_inliers) {
//       ninliers = N;
//       src = src_; // copy constructor
//       dst = dst_;
//   } else {
//       ninliers = 0;
//       for (int i=0; i<N; i++){
//           if (inliers[i]) {
//               ninliers++;
//               src.push_back(src_[i]);
//               dst.push_back(dst_[i]);
//           }
//       }
//   }
//   if (ninliers<2) {
//       param[0] = 0.0f; // default return when there is not enough points
//       param[1] = 0.0f;
//       param[2] = 0.0f;
//       return LARGE_NUMBER;
//   }

//   /* Algorithm: Least-Square Rigid Motion Using SVD by Olga Sorkine
//    * http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
//    *
//    * Subtract centroids, calculate SVD(cov),
//    * R = V[1, det(VU')]'U', T = mean_q-R*mean_p
//    */

//   // Calculate data centroids
//   Scalar centroid_src = mean(src);
//   Scalar centroid_dst = mean(dst);
//   Point2f center_src(centroid_src[0], centroid_src[1]);
//   Point2f center_dst(centroid_dst[0], centroid_dst[1]);
//   if (debug)
//       cout<<"Centers: "<<center_src<<", "<<center_dst<<endl;

//   // subtract centroids from data
//   for (int i=0; i<ninliers; i++) {
//       src[i] -= center_src;
//       dst[i] -= center_dst;
//   }

//   // compute a covariance matrix
//   float Cxx = 0.0, Cxy = 0.0, Cyx = 0.0, Cyy = 0.0;
//   for (int i=0; i<ninliers; i++) {
//       Cxx += src[i].x*dst[i].x;
//       Cxy += src[i].x*dst[i].y;
//       Cyx += src[i].y*dst[i].x;
//       Cyy += src[i].y*dst[i].y;
//   }
//   Mat Mcov = (Mat_<float>(2, 2)<<Cxx, Cxy, Cyx, Cyy);
//   Mcov /= (ninliers-1);
//   if (debug)
//       cout<<"Covariance-like Matrix "<<Mcov<<endl;

//   // SVD of covariance
//   cv::SVD svd;
//   svd = SVD(Mcov, SVD::FULL_UV);
//   if (debug) {
//       cout<<"U = "<<svd.u<<endl;
//       cout<<"W = "<<svd.w<<endl;
//       cout<<"V transposed = "<<svd.vt<<endl;
//   }

//   // rotation (V*Ut)
//   Mat V = svd.vt.t();
//   Mat Ut = svd.u.t();
//   float det_VUt = determinant(V*Ut);
//   Mat W = (Mat_<float>(2, 2)<<1.0, 0.0, 0.0, det_VUt);
//   float rot[4];
//   Mat R_est(2, 2, CV_32F, rot);
//   R_est = V*W*Ut;
//   if (debug)
//       cout<<"Rotation matrix: "<<R_est<<endl;

//   float cos_est = rot[0];
//   float sin_est = rot[2];
//   float ang = atan2(sin_est, cos_est);

//   // translation (mean_dst - R*mean_src)
//   Point2f center_srcRot = Point2f(
//           cos_est*center_src.x - sin_est*center_src.y,
//           sin_est*center_src.x + cos_est*center_src.y);
//   Point2f T_est = center_dst - center_srcRot;

//   // Final estimate msg
//   if (debug)
//       cout<<"Estimate = "<<ang*RAD2DEG<<"deg., T = "<<T_est<<endl;

//   param[0] = ang; // rad
//   param[1] = T_est.x;
//   param[2] = T_est.y;

//   // calculate RMSE
//   float RMSE = RMSE_3Dof(src_, dst_, param, inliers, center);
//   return RMSE;
//} // fit3DofQUADRATIC()

//// RANSAC fit in 3DOF: 1D rot and 2D translation (maximizes the number of inliers)
//// NOTE: no data normalization is currently performed
//float fit3DofRANSAC(const vector<Point2f>& src, const vector<Point2f>& dst,
//       float* best_param,  bool* inliers,
//       const Point2f center ,
//       const float inlierMaxEr,
//       const int niter) {

//   const int ITERATION_TO_SETTLE = 2;    // iterations to settle inliers and param
//   const float INLIERS_RATIO_OK = 0.95f;  // stopping criterion

//   // size of data vector
//   unsigned int N = src.size();
//   assert(N==dst.size());

//   // unrealistic case
//   if(N<2) {
//       best_param[0] = 0.0f; // ?
//       best_param[1] = 0.0f;
//       best_param[2] = 0.0f;
//       return LARGE_NUMBER;
//   }

//   unsigned int ninliers;                 // current number of inliers
//   unsigned int best_ninliers = 0;     // number of inliers
//   float best_rmse = LARGE_NUMBER;         // error
//   float cur_rmse;                         // current distance error
//   float param[3];                         // rad, Tx, Ty
//   vector <Point2f> src_2pt(2), dst_2pt(2);// min set of 2 points (1 correspondence generates 2 equations)
//   srand (time(NULL));

//   // iterations
//   for (int iter = 0; iter<niter; iter++) {

//#ifdef DEBUG_RANSAC
//       cout<<"iteration "<<iter<<": ";
//#endif

//       // 1. Select a random set of 2 points (not obligatory inliers but valid)
//       int i1, i2;
//       i1 = rand() % N; // [0, N[
//       i2 = i1;
//       while (i2==i1) {
//           i2 = rand() % N;
//       }
//       src_2pt[0] = src[i1]; // corresponding points
//       src_2pt[1] = src[i2];
//       dst_2pt[0] = dst[i1];
//       dst_2pt[1] = dst[i2];
//       bool two_inliers[] = {true, true};

//       // 2. Quadratic fit for 2 points
//       cur_rmse = fit3DofQUADRATIC(src_2pt, dst_2pt, param, two_inliers, center);

//       // 3. Recalculate to settle params and inliers using a larger set
//       for (int iter2=0; iter2<ITERATION_TO_SETTLE; iter2++) {
//           ninliers = setInliers3Dof(src, dst, inliers, param, inlierMaxEr, center);   // changes inliers
//           cur_rmse = fit3DofQUADRATIC(src, dst, param, inliers, center);              // changes cur_param
//       }

//       // potential ill-condition or large error
//       if (ninliers<2) {
//#ifdef DEBUG_RANSAC
//           cout<<" !!! less than 2 inliers "<<endl;
//#endif
//           continue;
//       } else {
//#ifdef DEBUG_RANSAC
//           cout<<" "<<ninliers<<" inliers; ";
//#endif
//       }


//#ifdef DEBUG_RANSAC
//       cout<<"; recalculate: RMSE = "<<cur_rmse<<", "<<ninliers <<" inliers";
//#endif


//       // 4. found a better solution?
//       if (ninliers > best_ninliers) {
//           best_ninliers = ninliers;
//           best_param[0] = param[0];
//           best_param[1] = param[1];
//           best_param[2] = param[2];
//           best_rmse = cur_rmse;


//#ifdef DEBUG_RANSAC
//           cout<<" --- Solution improved: "<<
//                   best_param[0]<<", "<<best_param[1]<<", "<<param[2]<<endl;
//#endif
//           // exit condition
//           float inlier_ratio = (float)best_ninliers/N;
//           if (inlier_ratio > INLIERS_RATIO_OK) {
//#ifdef DEBUG_RANSAC
//               cout<<"Breaking early after "<< iter+1<<
//                       " iterations; inlier ratio = "<<inlier_ratio<<endl;
//#endif
//               break;
//           }
//       } else {
//#ifdef DEBUG_RANSAC
//           cout<<endl;
//#endif
//       }


//   } // iterations

//   // 5. recreate inliers for the best parameters
//   ninliers = setInliers3Dof(src, dst, inliers, best_param, inlierMaxEr, center);

//   return best_rmse;
//} // fit3DofRANSAC()
