#include "Segmentation.h"
#include "Drawing/Drawing.h"
#include "Drawing/Chart.h"

#include "Cronometer.h"

#include <iostream>
#include <queue>

// Respect ant clock wise!! Do not change this values
const int Segmentation::neighborRow[8] = { -1, -1,  0,  1,  1,  1, 0,-1};
const int Segmentation::neighborCol[8] = {  0, -1, -1, -1,  0,  1, 1, 1};

#include "SegmentSearcher/DoubleSegmentSearcher.h"
#include "SegmentSearcher/LinearSegmentSearcher.h"
#include "SegmentSearcher/SegmentWithPeak.h"
#include "SegmentSearcher/ThetaRhoSegmentSearcher.h"
#include "SegmentSearcher/ThetaRhoSortSegSearch.h"
#include "SegmentSearcher/ThetaRhoMeanPeakSegSearch.h"

#include "SegmentExtractor/BorderSegmentExtractor.h"
#include "SegmentExtractor/DistantSegmentExtractor.h"
#include "SegmentExtractor/DistantSegmentExtractorV2.h"
#include "SegmentExtractor/FullSegmentExtractor.h"
#include "SegmentExtractor/PixelRelativeSegmentExtractor.h"
#include "SegmentExtractor/RelativeSegmentExtractor.h"

void Segmentation::reduceToConvexHull(Segment *seg)
{
//        #include <algorithm>
//        #include <vector>
//        using namespace std;

//        typedef double coord_t;         // coordinate type
//        typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2

//        struct Point {
//            coord_t x, y;

//            bool operator <(const Point &p) const {
//                return x < p.x || (x == p.x && y < p.y);
//            }
//        };

//        // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
//        // Returns a positive value, if OAB makes a counter-clockwise turn,
//        // negative for clockwise turn, and zero if the points are collinear.
//        coord2_t cross(const Point &O, const Point &A, const Point &B)
//        {
//            return (long)(A.x - O.x) * (B.y - O.y) - (long)(A.y - O.y) * (B.x - O.x);
//        }

//        // Returns a list of points on the convex hull in counter-clockwise order.
//        // Note: the last point in the returned list is the same as the first one.
//        vector<Point> convex_hull(vector<Point> P)
//        {
//            int n = P.size(), k = 0;
//            vector<Point> H(2*n);

//            // Sort points lexicographically
//            sort(P.begin(), P.end());

//            // Build lower hull
//            for (int i = 0; i < n; ++i) {
//                while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
//                H[k++] = P[i];
//            }

//            // Build upper hull
//            for (int i = n-2, t = k+1; i >= 0; i--) {
//                while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
//                H[k++] = P[i];
//            }

//            H.resize(k);
//            return H;
//        }
}

/**
 * @brief Compute the meigborhood
 *
 * @param grayImg
 * @param row
 * @param col
 * @return unsigned
 */
unsigned Segmentation::neighborSum(Mat &grayImg, unsigned row, unsigned col)
{
    unsigned sum = 0;
    for(unsigned neighbor = 0 ; neighbor < 8 ; neighbor++)
    {
        unsigned nextRow = row + neighborRow[neighbor],
                 nextCol = col + neighborCol[neighbor];

        // If neighbor isn't on image
        if(nextRow >= grayImg.rows ||
           nextCol >= grayImg.cols)
            continue;

        sum+= grayImg.at<ushort>(nextRow,nextCol);
    }
    return sum;
}

Segment *Segmentation::segment(unsigned id)
{
    unsigned segSize = segments.size();
    if(id < segSize)
    {
        return segments[id];
    }

    #ifdef SEGMENTATION_MEMORORY_DEBUG
        cout << "segment: Allocating more segments!!" << endl;
    #endif

    segments.resize((segSize+1)*2);

    for(segSize ; segSize < segments.size() ; segSize++)
    {
        segments[segSize] = new Segment;
    }

    return segments[id];
}

void Segmentation::resetMask(unsigned rows, unsigned cols)
{
    searchMask = Mat(rows, cols, CV_8UC1,Scalar(0));
#ifdef SEGMENTATION_SEARCH_DEBUG
    imgDebug.release();
#endif
}

Segmentation::Segmentation():
    searchDistance(3),
    maxSampleSize(1200),
    minSampleSize(20),
    m_segExtractor(0x0),
    m_segSearcher(0x0),
    pixelThreshold(225) // It's usualy used
{

    cout << "Loading sonar image mask" << endl;
    imgMask = imread("../../GroundTruth/mask.png",CV_LOAD_IMAGE_GRAYSCALE);

    loadDefaultConfig();

}

Segmentation::Segmentation(ConfigLoader &config):
    searchDistance(3),
    maxSampleSize(1200),
    minSampleSize(20),
    m_segExtractor(0x0),
    m_segSearcher(0x0)
{
    cout << "Loading sonar image mask" << endl;
    imgMask = imread("../../GroundTruth/mask.png",CV_LOAD_IMAGE_GRAYSCALE);

    loadDefaultConfig();
    load(config);
}

Segmentation::~Segmentation()
{
    delete m_segSearcher;
    delete m_segExtractor;

    // Delete Segments
    for(unsigned i = 0 ; i < segments.size(); i++)
    {
        delete segments[i];
    }
}


/**
 * @brief Load segmentation configs from a SonarConfig.
 *
 * @param config
 */
void Segmentation::load(ConfigLoader &config)
{
    SegmentSearcher *ss=0x0;
    SegmentExtractor *se=0x0;
    string str;

    if(config.getString("General","SegmentSearcher",&str))
    {
        if(str == "DoubleSegmentSearcher")
        {
            ss = new DoubleSegmentSearcher;

        }else if(str == "LinearSegmentSearcher")
        {
            ss = new LinearSegmentSearcher;

        }else if(str == "SegmentWithPeak")
        {
            ss = new SegmentWithPeak;

        }else if(str == "ThetaRhoSegmentSearcher")
        {
            ss = new ThetaRhoSegmentSearcher;
        }else if(str == "ThetaRhoSortSegSearch")
        {
            ss = new ThetaRhoSortSegSearch;
        }else if(str == "ThetaRhoMeanPeakSegSearch")
        {
            ss = new ThetaRhoMeanPeakSegSearch;
        }
    }

    if(config.getString("General","SegmentExtractor",&str))
    {
        if(str == "BorderSegmentExtractor")
        {
            se = new BorderSegmentExtractor;

        }else if(str == "DistantSegmentExtractor")
        {
            se = new DistantSegmentExtractor;

        }else if(str == "DistantSegmentExtractorV2")
        {
            se = new DistantSegmentExtractorV2;

        }else if(str == "FullSegmentExtractor")
        {
            se = new FullSegmentExtractor;
        }else if(str == "PixelRelativeSegmentExtractor")
        {
            se = new PixelRelativeSegmentExtractor;
        }else if(str == "RelativeSegmentExtractor")
        {
            se = new RelativeSegmentExtractor;
        }
    }

    setSegmentSearcher(ss);
    ss->load(config);

    setSegmentExtractor(se);
    se->load(config);
}

void Segmentation::loadDefaultConfig()
{
    if(m_segExtractor == 0x0)
        m_segExtractor = new DistantSegmentExtractor;

    if(m_segSearcher == 0x0)
        m_segSearcher = new LinearSegmentSearcher;

    m_segSearcher->setExtractor(m_segExtractor);
    m_segSearcher->setSegmentation(this);

    m_segExtractor->setSegmentation(this);
}

void Segmentation::setSegmentExtractor(SegmentExtractor *segExtractor)
{
    if(segExtractor != 0x0)
    {
        if(m_segExtractor!=0x0)
            delete m_segExtractor;

        m_segExtractor = segExtractor;
        m_segExtractor->setSegmentation(this);
        m_segSearcher->setExtractor(m_segExtractor);
    }else
    {
        cout << "Warning: Segmentation didn't get SegmentExtractor configs!" << endl;
    }
}

void Segmentation::setSegmentSearcher(SegmentSearcher *segSearcher)
{
    if(segSearcher != 0x0)
    {
        if(m_segSearcher != 0x0)
            delete m_segSearcher;

        m_segSearcher = segSearcher;
        m_segSearcher->setExtractor(m_segExtractor);
        m_segSearcher->setSegmentation(this);
    }else
    {
        cout << "Warning: Segmentation didn't get SegmentSearcher configs!" << endl;
    }
}


/**
 * @brief Extract segments from an acoustic image.
 *
 * @param img16bits - Image that will be segmented.
 * @param sg - Vector of stracted segments.
 */
void Segmentation::segment(Mat &img16bits, vector<Segment *> *sg)
{
//    resetMask(img16bits.rows,img16bits.cols);
    m_segSearcher->segment(img16bits,sg);
}


/**
 * @brief Create a segment starting on row and
 * line position.
 *
 * @param seg - Segment that will be created.
 * @param img16bits - Image that will be analyzed
 * @param row - Start row
 * @param col - Start column
 */
void Segmentation::createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
    m_segExtractor->createSegment(seg,img16bits,row,col);
}

void Segmentation::interativeCalibUI(Mat &img16bits)
{
    m_segExtractor->calibUI(img16bits);
    m_segSearcher->calibUI(img16bits);
}

void Segmentation::createSegmentImg(Mat &img16bits)
{
    Mat result(img16bits.rows , img16bits.cols, CV_8UC3, Scalar(0,0,0));

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);

    /* Initialize Mask of Visit */
    resetMask(img16bits.rows,img16bits.cols);
    vector<Segment*> sg;

    /* Search for high intensity pixels */
    unsigned linJump = 1, colJump = 1, segCount=0;
    Segment *seg;
    for(unsigned row = 0 ; row < img16bits.rows ; row+=linJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(searchMask.at<uchar>(row,col) == 0 &&
               img16bits.at<ushort>(row,col) >= pixelThreshold)
            {
                // Search the segment on image
                seg = segment(segCount);
//                createSegment(seg,img16bits,row,col);
//                createSegmentDist(seg,img16bits,row,col);
                m_segExtractor->createSegment(seg,img16bits,row,col);

                if(seg->N < minSampleSize)
                {
                    continue;
                }

                segCount++;

                seg->drawSegment(result,colorMap[segCount%6]);
                // Add seg to answer
                sg.push_back(seg);
            }
        }
    }
    imshow("Thresholded Image", result);
}

void Segmentation::createLogImg(Mat &img16bits)
{
    Mat result;
    img16bits.convertTo(result,CV_16UC1,1.0,0.0);
    result.convertTo(result,CV_32FC1,1.0,1);
    cv::log(result,result);
    cv::normalize(result,result,0,255,cv::NORM_MINMAX);
    result.convertTo(result,CV_8UC1);
    imshow("Log Image", result);

//    result = result + 1;
//    cv::log(result,result);
//    imshow("Log Image", result);
}

bool Segmentation::createGradientImg(Mat &img16bits)
{
    Mat result(img16bits.rows , img16bits.cols, CV_16UC1, Scalar(0));

    /* Search for high intensity pixels */
    unsigned linJump = 1, colJump = 1;
    for(unsigned row = 0 ; row < img16bits.rows ; row+=linJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            unsigned r = abs(((int)(neighborSum(img16bits,row,col)/ 8.0 + 0.5)) - img16bits.at<ushort>(row,col));
            if(r > 255) cout << "r = " << r << endl;
            result.at<ushort>(row,col) = r;
        }
    }
    normalize(result, result,0,255, NORM_MINMAX);
    result.convertTo(result,CV_8UC1);
//    imshow("Img Gradient", result);
}

void Segmentation::createRTImage(Mat &img16bits)
{
    int sonVerticalPosition =9;
    unsigned nBeams = 768, startBin=25, nBins = img16bits.rows-startBin;
    float bearing = 130;

    float beamAng = bearing/(nBeams-1),
          cAng = -bearing/2.f,
          endAng = bearing/2.f;

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);


    Point2f sonarPos(img16bits.cols/2.f, img16bits.rows+sonVerticalPosition);

    Mat result(img16bits.rows, img16bits.cols, CV_8UC3),
        rtResult(nBins,nBeams,CV_8UC3, Scalar(0,0,0));

    img16bits.convertTo(result,CV_8UC1);
    cvtColor(result,result, CV_GRAY2BGR);

    unsigned beam=nBeams-1;
    // For each beam
    for(; cAng < endAng ; cAng+=beamAng)
    {
        float radAng = cAng*M_PI/180.f;

        Point2f beamPos(sonarPos.x - startBin*sin(radAng),
                        sonarPos.y - startBin*cos(radAng));

        Point2f beamDir(-sin(radAng),-cos(radAng));

        // For each bin
        for(unsigned bin = nBins-1 ; bin < nBins; bin--)// unsigned overflow!
        {
            beamPos+= beamDir;

            rtResult.at<Vec3b>(bin,beam)[0] = result.at<Vec3b>(beamPos.y,beamPos.x)[0];
            rtResult.at<Vec3b>(bin,beam)[1] = result.at<Vec3b>(beamPos.y,beamPos.x)[1];
            rtResult.at<Vec3b>(bin,beam)[2] = result.at<Vec3b>(beamPos.y,beamPos.x)[2];

//            result.at<Vec3b>(beamPos.y,beamPos.x)[0] = colorMap[beam%6].val[0];
//            result.at<Vec3b>(beamPos.y,beamPos.x)[1] = colorMap[beam%6].val[1];
//            result.at<Vec3b>(beamPos.y,beamPos.x)[2] = colorMap[beam%6].val[2];
        }

        beam--;
    }

    imshow("XY image", result);
    imshow("RT image", rtResult);
}

bool Segmentation::createImg8bitsTruncated(Mat &img16bits)
{
    Mat result;
    img16bits.convertTo(result,CV_8UC1);
    imshow("8bits Truncated", result);
}

void Segmentation::createAdaptativeThreshold(Mat &img16bits)
{
    Mat result;

    img16bits.convertTo(result,CV_8UC1,1.0,0.0);
    medianBlur(result,result,5);
    adaptiveThreshold(result,result,255.0,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,11, 2);
    imshow("AdaptativeThreshold", result);
}

void Segmentation::createAdaptativeThresholdOtsu(Mat &img16bits)
{
    Mat result;

    img16bits.convertTo(result,CV_8UC1,1.0,0.0);

    medianBlur(result,result,5);
    threshold(result,result,0,255,THRESH_BINARY + THRESH_OTSU);

    imshow("AdaptativeThresholdOtsu", result);
}

void Segmentation::createAdaptativeThreshold2(Mat &img16bits)
{

}

void Segmentation::createRTPlot(Mat &img16bits)
{
    // Algorithm parameters
    int sonVerticalPosition =1;

    unsigned nBeams = 1,startBin=20,
            acceptPeakHeight=150; // Minimum height to a peak be acceptable

    float bearing = 20,
          endPeakTax=0.6, // If a peak down low tham resetTax of height, reset a peak.
          takePeakPercent=0.5; // Height percent of acceptable peak used for threshold


    unsigned nBins = img16bits.rows-startBin-11;
    float beamAng,
          cAng = -bearing/2.f,
          endAng = bearing/2.f;

    if(nBeams>1)
        beamAng = bearing/(nBeams-1);
    else beamAng = 2*bearing; // Infinity!

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);

    Point2f sonarPos(img16bits.cols/2.f, img16bits.rows+sonVerticalPosition);

    Mat mPlot,mask;
    Scalar plotInfo;

    Chart chart;

    Drawing::makePlotInfo(mPlot,Size2i(1200,800),
                          0,nBins-1,0,850,plotInfo);

    img16bits.convertTo(mask,CV_8UC1);
    cvtColor(mask,mask,CV_GRAY2BGR);

    unsigned beam=nBeams-1;

    vector<ushort> intensitys;

    // For each beam
    for(unsigned i = 0; i < nBeams; i++, cAng+=beamAng)
    {
        float radAng = cAng*M_PI/180.f;

        Point2f beamPos(sonarPos.x - startBin*sin(radAng),
                        sonarPos.y - startBin*cos(radAng));

        Point2f beamDir(-sin(radAng),-cos(radAng));

        unsigned minBinI=99999, maxBinI=0, minBinN, maxBinN,
                 binI,
                 peakHeight,
                 lastThreshold=99999,
                 peakCount=0,
             accepetedPlot = chart.newLabel(Chart::PLOT_LINE,colorMap[beam%6],1),
          rejectedPeakPlot = chart.newLabel(Chart::PLOT_LINE,Scalar(0,0,255),1);

        // For each bin
        for(unsigned bin = nBins-1 ; bin < nBins; bin--)// unsigned overflow!
        {
            beamPos+= beamDir;

            binI = img16bits.at<ushort>(beamPos.y,beamPos.x);

            intensitys.push_back(binI);

            mask.at<Vec3b>(beamPos.y,beamPos.x)[0] = colorMap[beam%6].val[0];
            mask.at<Vec3b>(beamPos.y,beamPos.x)[1] = colorMap[beam%6].val[1];
            mask.at<Vec3b>(beamPos.y,beamPos.x)[2] = colorMap[beam%6].val[2];

            if(binI < minBinI)
            {
                minBinI = binI;
                minBinN = nBins - bin -1;
                maxBinI = 0;
            }else
            if(binI > maxBinI)
            {
                 maxBinI = binI;
                 maxBinN = nBins - bin -1;
            }else
            if(maxBinI > minBinI)
            {
                peakHeight = maxBinI - minBinI;

                if( (binI-minBinI) < peakHeight*endPeakTax)
                {
                    if(peakHeight > acceptPeakHeight)
                    {
                        // Accept Peak!
                        lastThreshold = minBinI+peakHeight*takePeakPercent;

                        chart.addPoint(accepetedPlot,minBinN,lastThreshold);
                        chart.addPoint(accepetedPlot,nBins - bin -1,lastThreshold);

                        chart.newLabel(Chart::PLOT_CIRCLE,colorMap[peakCount%6],-1);
                        chart.addPoint(maxBinN,maxBinI); // PLot a circle the top

                        chart.newLabel(Chart::PLOT_CIRCLE,Scalar(0,0,0),1);
                        chart.addPoint(maxBinN,maxBinI); // PLot a circle the top

                        Point2f peakPosition;
                        float vecUnit = nBins - bin - maxBinN;
                        peakPosition = beamPos - beamDir * vecUnit;

                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[0] = 0;
                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[1] = 0;
                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[2] = 255;

                        circle(mask,peakPosition,7,colorMap[peakCount%6],-1);
//                        circle(mask,peakPosition,7,Scalar(0,0,0),2);
                        peakCount++;
                    }else
                    {
                        chart.addPoint(rejectedPeakPlot,minBinN,minBinI);
                        chart.addPoint(rejectedPeakPlot,nBins - bin -1,minBinI);
                    }

                    // Reset Peak (not acceptable)
                    minBinI=binI;
                    minBinN= nBins - bin -1;
                    maxBinI=0;


//                    imshow("TR image mask", mask);
//                    imshow("TR plot", mPlot);
//                    waitKey();
                }
            }
        }

        chart.newLabel(Chart::PLOT_CONTINUOS_LINE,colorMap[beam%6]);
        for(unsigned i = 0 ; i < nBins ; i++)
            chart.addPoint(float(i),float(intensitys[i]));

        intensitys.clear();
        beam--;
    }

    imshow("TR image mask", mask);
    Drawing::plot(chart,mPlot);
    imshow("TR plot", mPlot);
}

void Segmentation::interativeRTPlot(Mat &img16bits)
{
    // Algorithm parameters
    int sonVerticalPosition =1;

    unsigned nBeams = 1,startBin=20,
            acceptPeakHeight=150; // Minimum height to a peak be acceptable

    float endPeakTax=0.3, // If a peak down low tham resetTax of height, reset a peak.
          takePeakPercent=0.4, // Height percent of acceptable peak used for threshold
          beamAng;

    unsigned nBins = img16bits.rows-startBin-11;

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);

    Point2f sonarPos(img16bits.cols/2.f, img16bits.rows+sonVerticalPosition);

    Mat mPlot,mask;

    Chart chart;
    vector<ushort> intensitys;
    Segment seg;
    unsigned beam=0;
    while (true)
    {
        resetMask(img16bits.rows,img16bits.cols);

        float radAng = beamAng*M_PI/180.f;

        Point2f beamPos(sonarPos.x - startBin*sin(radAng),
                        sonarPos.y - startBin*cos(radAng));

        Point2f beamDir(-sin(radAng),-cos(radAng));

        unsigned minBinI=99999, maxBinI=0, minBinN, maxBinN,
                 binI,
                 peakHeight,
                 lastThreshold=99999,
                 peakCount=0,
             accepetedPlot = chart.newLabel(Chart::PLOT_LINE,colorMap[beam%6],1),
          rejectedPeakPlot = chart.newLabel(Chart::PLOT_LINE,Scalar(0,0,255),1);

        img16bits.convertTo(mask,CV_8UC1);
        cvtColor(mask,mask,CV_GRAY2BGR);

        // For each bin
        for(unsigned bin = nBins-1 ; bin < nBins; bin--)// unsigned overflow!
        {
            beamPos+= beamDir;

            binI = img16bits.at<ushort>(beamPos.y,beamPos.x);

            intensitys.push_back(binI);

            mask.at<Vec3b>(beamPos.y,beamPos.x)[0] = colorMap[beam%6].val[0];
            mask.at<Vec3b>(beamPos.y,beamPos.x)[1] = colorMap[beam%6].val[1];
            mask.at<Vec3b>(beamPos.y,beamPos.x)[2] = colorMap[beam%6].val[2];

            if(binI < minBinI)
            {
                minBinI = binI;
                minBinN = nBins - bin -1;
                maxBinI = 0;
            }else
            if(binI > maxBinI)
            {
                 maxBinI = binI;
                 maxBinN = nBins - bin -1;
            }else
            if(maxBinI > minBinI)
            {
                peakHeight = maxBinI - minBinI;

                if( (binI-minBinI) < peakHeight*endPeakTax)
                {
                    if(peakHeight > acceptPeakHeight)
                    {
                        // Accept Peak!
                        lastThreshold = minBinI+peakHeight*takePeakPercent;

                        chart.addPoint(accepetedPlot,minBinN,lastThreshold);
                        chart.addPoint(accepetedPlot,nBins - bin -1,lastThreshold);

                        chart.newLabel(Chart::PLOT_CIRCLE,colorMap[peakCount%6],-1);
                        chart.addPoint(maxBinN,maxBinI); // PLot a circle the top

                        chart.newLabel(Chart::PLOT_CIRCLE,Scalar(0,0,0),1);
                        chart.addPoint(maxBinN,maxBinI); // PLot a circle the top

                        Point2f peakPosition;
                        float vecUnit = nBins - bin - maxBinN;
                        peakPosition = beamPos - beamDir * vecUnit;

                        m_segExtractor->setThreshold(lastThreshold);

                        m_segExtractor->createSegment(&seg,img16bits,peakPosition.y,peakPosition.x);
//                        createSegmentFull(&seg,img16bits,peakPosition.y,peakPosition.x);
//                        createSegmentDist(&seg,img16bits,peakPosition.y,peakPosition.x);
                        seg.drawSegment(mask,colorMap[peakCount%6]);

                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[0] = 0;
                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[1] = 0;
                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[2] = 255;

                        circle(mask,peakPosition,7,colorMap[peakCount%6],-1);
                        circle(mask,peakPosition,7,Scalar(0,0,0),1);


                        peakCount++;
                    }else
                    {
                        chart.addPoint(rejectedPeakPlot,minBinN,minBinI);
                        chart.addPoint(rejectedPeakPlot,nBins - bin -1,minBinI);
                    }

                    // Reset Peak (not acceptable)
                    minBinI=binI;
                    minBinN= nBins - bin -1;
                    maxBinI=0;
                }
            }
        }

        chart.newLabel(Chart::PLOT_CONTINUOS_LINE,colorMap[beam%6]);
        for(unsigned i = 0 ; i < nBins ; i++)
            chart.addPoint(float(i),float(intensitys[i]));

        intensitys.clear();

        imshow("TR image mask", mask);
        Drawing::plot(chart,mPlot);
        imshow("TR plot", mPlot);
        chart.clear();

        char c = waitKey();
        switch(c)
        {
        case 'a':
            if(beamAng<65.f)
                beamAng+=1.f;
        break;
        case 'd':
            if(beamAng>-65.f)
                beamAng-=1.f;
        break;
        case 'w':
            acceptPeakHeight+=5;
        break;
        case 's':
            if(acceptPeakHeight>5)
                acceptPeakHeight-=5;
        break;
        case 10: case 13: case 27: // keys Enter or ESC
            return;
        break;
        }
        cout << "bearign " << beamAng << " acceptPeakHeight " << acceptPeakHeight << endl;
    }

}


void Segmentation::extractLine(Mat &result, Point2f src, Point2f dest, Scalar &color)
{
    int x1 = floor(dest.x+0.5), x0 = floor(src.x+0.5),
        y1 = floor(dest.y+0.5), y0 = floor(src.y+0.5),
        dx = x0 - x1,
        dy = y0 - y1,
        D = 2*dy - dx,
        y = y0, x = x0;


    if(x < result.cols && x>=0 &&
       y < result.rows && y >=0)
    {
        result.at<Vec3i>(x,y)[0] = color.val[0];
        result.at<Vec3i>(x,y)[1] = color.val[1];
        result.at<Vec3i>(x,y)[2] = color.val[2];
    }

    for(x = x0+1; x <= x1; x++)
    {
        if( D>0)
        {
            y = y+1;

            if(x < result.cols && x>=0 &&
               y < result.rows && y >=0)
            {
                result.at<Vec3i>(x,y)[0] = color.val[0];
                result.at<Vec3i>(x,y)[1] = color.val[1];
                result.at<Vec3i>(x,y)[2] = color.val[2];
            }else cout << "MISS PIXEL!" << endl;

            D+= 2*dy-2*dx;
        }else
        {

            if(x < result.cols && x>=0 &&
               y < result.rows && y >=0)
            {
                result.at<Vec3i>(x,y)[0] = color.val[0];
                result.at<Vec3i>(x,y)[1] = color.val[1];
                result.at<Vec3i>(x,y)[2] = color.val[2];
            }else cout << "MISS PIXEL!" << endl;

            D += 2*dy;
        }
    }
}

void Segmentation::extractLine2(Mat &result, Point2f src, float radAng, unsigned lenght, Scalar &color)
{
    Point2f vec(-sin(radAng),-cos(radAng));

    while(lenght--)
    {
        src+= vec;
        result.at<Vec3b>(src.y,src.x)[0] = color.val[0];
        result.at<Vec3b>(src.y,src.x)[1] = color.val[1];
        result.at<Vec3b>(src.y,src.x)[2] = color.val[2];
    }
}

void Segmentation::morphologicalOperator()
{
   // http://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=morphologyex#morphologyex
   // morphologyex
}
