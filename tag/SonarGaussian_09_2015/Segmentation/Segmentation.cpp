#include "Segmentation.h"
#include "Drawing/Drawing.h"

#include "Cronometer.h"

#include <iostream>
#include <queue>

const int Segmentation::neighborRow[8] = { -1, -1, -1,  0,  0,  1, 1, 1};
const int Segmentation::neighborCol[8] = { -1,  0,  1, -1,  1, -1, 0, 1};

void Segmentation::createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
    // Initialize segment
    seg->N = 0;
    seg->MCol = seg->MRow = 0;
    seg->mRow = seg->mCol = 99999;

    // Realocate memory if necsessary
    if(seg->result.rows < 3 || seg->result.cols < maxSampleSize)
    {
        #ifdef SEGMENTATION_MEMORORY_DEBUG
                cout << "createSegment: Allocating memory for segments!!" << endl;
        #endif

        seg->result = Mat(3, maxSampleSize , CV_16UC1, Scalar(0));
    }

    // Initialize queue of BFS
    typedef pair<unsigned, unsigned> PUU;
    queue< PUU > q;

    // Add first element
    q.push(PUU(row,col));
    searchMask.at<uchar>(row,col) = 255;

    while(!q.empty() && seg->N < maxSampleSize)
    {
        row = q.front().first;
        col = q.front().second;
        q.pop();

        // Register first element
        seg->result.at<ushort>(0,seg->N) = row;
        seg->result.at<ushort>(1,seg->N) = col;
        seg->result.at<ushort>(2,seg->N) = img16bits.at<ushort>(row,col);
        seg->N++;

        // Take min max col and row
        if(seg->MRow < row)
            seg->MRow = row;
        if(seg->mRow > row)
            seg->mRow = row;
        if(seg->MCol < col)
            seg->MCol = col;
        if(seg->mCol > col)
            seg->mCol = col;

        // Seach the neighbor pixels
        for(unsigned neighbor = 0 ; neighbor < 8 && seg->N < maxSampleSize ; neighbor++)
        {
            unsigned nextRow = row + neighborRow[neighbor],
                     nextCol = col + neighborCol[neighbor];

            // If neighbor exist on the image
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If neighbor wasn't visited
            if(img16bits.at<ushort>(nextRow , nextCol) >= searchThreshold
               && searchMask.at<uchar>(nextRow,nextCol) != 255)
            {
                // Register the visit of pixel
                searchMask.at<uchar>(nextRow,nextCol) = 255;

                // Register new querry
                q.push(PUU(nextRow , nextCol));
            }
        }
    }
}

void Segmentation::createBorderSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col, unsigned searchThreshold)
{
    // Initialize segment
    seg->N = 0;
    seg->MCol = seg->MRow = 0;
    seg->mRow = seg->mCol = 99999;

    // Realocate memory if necsessary
    if(seg->result.rows < 3 || seg->result.cols < maxSampleSize)
    {
        #ifdef SEGMENTATION_MEMORORY_DEBUG
            cout << "createBorderSegment: Allocating memory for segments!!" << endl;
        #endif
        seg->result = Mat(3, maxSampleSize , CV_16UC1, Scalar(0));
    }

    // Initialize queue of BFS
    typedef pair<unsigned, unsigned> PUU;
    queue< PUU > q;

    // Add first element
    q.push(PUU(row,col));
    searchMask.at<uchar>(row,col) = 255;

    bool isBorder;

    while(!q.empty() && seg->N < maxSampleSize)
    {
        isBorder = false;
        row = q.front().first;
        col = q.front().second;
        q.pop();

        // Seach the neighbor pixels
        for(unsigned neighbor = 0 ; neighbor < 8 && seg->N < maxSampleSize ; neighbor++)
        {
            unsigned nextRow = row + neighborRow[neighbor],
                     nextCol = col + neighborCol[neighbor];

            // If neighbor exist on the image
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If neighbor wasn't visited
            if(searchMask.at<uchar>(nextRow,nextCol) != 255)
            {
                if(img16bits.at<ushort>(nextRow , nextCol) >= searchThreshold)
                {
                    // Register the visit of pixel
                    searchMask.at<uchar>(nextRow,nextCol) = 255;

                    // Register new querry
                    q.push(PUU(nextRow , nextCol));
                }else isBorder = true;
            }
        }

        if(isBorder)
        {
            // Register first element
            seg->result.at<ushort>(0,seg->N) = row;
            seg->result.at<ushort>(1,seg->N) = col;
            seg->result.at<ushort>(2,seg->N) = img16bits.at<ushort>(row,col);
            seg->N++;

            // Take min max col and row
            if(seg->MRow < row)
                seg->MRow = row;
            if(seg->mRow > row)
                seg->mRow = row;
            if(seg->MCol < col)
                seg->MCol = col;
            if(seg->mCol > col)
                seg->mCol = col;
        }
    }
}

void Segmentation::createSegmentDist(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
    // Realocate memory if necsessary
    if(seg->result.rows < 3 || seg->result.cols < maxSampleSize)
    {
        #ifdef SEGMENTATION_MEMORORY_DEBUG
            cout << "createSegmentDist: Allocating memory for segments!!" << endl;
        #endif
        seg->result = Mat(3, maxSampleSize , CV_16UC1, Scalar(0));
    }

    // Initialize segment
    seg->N = 0;
    seg->MCol = seg->MRow = 0;
    seg->mRow = seg->mCol = 99999;

    // Initialize queue of BFS
    typedef pair<unsigned, unsigned> PUU;
    typedef pair<PUU, unsigned> PUUI;
    queue< PUUI > q;
    int dist=0;

    // Add first element
    q.push(PUUI(PUU(row,col),searchDistance));
    searchMask.at<uchar>(row,col) = 255;

    // Register first element
    seg->result.at<ushort>(0,seg->N) = row;
    seg->result.at<ushort>(1,seg->N) = col;
    seg->result.at<ushort>(2,seg->N) = img16bits.at<ushort>(row,col);
    seg->N++;

    // Take min max col and row
    if(seg->MRow < row)
        seg->MRow = row;
    if(seg->mRow > row)
        seg->mRow = row;
    if(seg->MCol < col)
        seg->MCol = col;
    if(seg->mCol > col)
        seg->mCol = col;

    while(!q.empty())
    {
        row = q.front().first.first;
        col = q.front().first.second;
        dist = q.front().second;
        q.pop();

        // Seach the neighbor pixels
        for(unsigned neighbor = 0 ; neighbor < 8 && seg->N < maxSampleSize ; neighbor++)
        {
            unsigned nextRow = row + neighborRow[neighbor],
                     nextCol = col + neighborCol[neighbor];

            // If neighbor isn't on image
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If we can visit this neighbor pixel
            if(searchMask.at<uchar>(nextRow,nextCol) != 255)
            {
            if(img16bits.at<ushort>(nextRow , nextCol) >= searchThreshold)
            {
                // Register the visit of pixel
                searchMask.at<uchar>(nextRow,nextCol) = 255;

                // Register pixel information
                seg->result.at<ushort>(0,seg->N) = row;
                seg->result.at<ushort>(1,seg->N) = col;
                seg->result.at<ushort>(2,seg->N) = img16bits.at<ushort>(row,col);
                seg->N++;

                // Take min max col and row
                if(seg->MRow < row)
                    seg->MRow = row;
                if(seg->mRow > row)
                    seg->mRow = row;
                if(seg->MCol < col)
                    seg->MCol = col;
                if(seg->mCol > col)
                    seg->mCol = col;

                // Register new querry
                q.push(PUUI(PUU(nextRow , nextCol),dist));
            }else if(dist >0) // If we can jump for then
            {
                // Register the visit of pixel
                searchMask.at<uchar>(nextRow,nextCol) = 255;

                // Register new querry
                q.push(PUUI(PUU(nextRow , nextCol),dist-1));
            }
            }
        }
    }
}

void Segmentation::serachAuto(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
}

void Segmentation::createSegmentRelative(Segment *seg, Mat img16bits, unsigned row, unsigned col, unsigned deacaiment)
{

    unsigned relativeThreshold;
    if(img16bits.at<ushort>(row,col) > deacaiment)
    {        
        relativeThreshold = img16bits.at<ushort>(row,col) - deacaiment;
    }
    if(relativeThreshold < 50) relativeThreshold = 50;

    // Realocate memory if necsessary
    if(seg->result.rows < 3 && seg->result.cols < maxSampleSize)
    {
        #ifdef SEGMENTATION_MEMORORY_DEBUG
            cout << "createSegmentRelative: Allocating memory for segments!!" << endl;
        #endif
        seg->result = Mat(3, maxSampleSize , CV_16UC1, Scalar(0));
    }

    // Initialize segment
    seg->N = 0;
    seg->MCol = seg->MRow = 0;
    seg->mRow = seg->mCol = 99999;

    // Initialize queue of BFS
    typedef pair<unsigned, unsigned> PUU;
    queue< PUU > q;

    // Add first element
    q.push(PUU(row,col));
    searchMask.at<uchar>(row,col) = 255;

    // Register first element
    seg->result.at<ushort>(0,seg->N) = row;
    seg->result.at<ushort>(1,seg->N) = col;
    seg->result.at<ushort>(2,seg->N) = img16bits.at<ushort>(row,col);
    seg->N++;

    // Take min max col and row
    if(seg->MRow < row)
        seg->MRow = row;
    if(seg->mRow > row)
        seg->mRow = row;
    if(seg->MCol < col)
        seg->MCol = col;
    if(seg->mCol > col)
        seg->mCol = col;

    while(!q.empty())
    {
        row = q.front().first;
        col = q.front().second;
        q.pop();

        // Seach the neighbor pixels
        for(unsigned neighbor = 0 ; neighbor < 8 && seg->N < maxSampleSize ; neighbor++)
        {
            unsigned nextRow = row + neighborRow[neighbor],
                     nextCol = col + neighborCol[neighbor];

            // If neighbor exist on the image
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If neighbor wasn't visited
            if(img16bits.at<ushort>(nextRow , nextCol) >= relativeThreshold
               && searchMask.at<uchar>(nextRow,nextCol) != 255)
            {
                // Register the visit of pixel
                searchMask.at<uchar>(nextRow,nextCol) = 255;

                // Register pixel information
                seg->result.at<ushort>(0,seg->N) = row;
                seg->result.at<ushort>(1,seg->N) = col;
                seg->result.at<ushort>(2,seg->N) = img16bits.at<ushort>(row,col);
                seg->N++;

                // Take min max col and row
                if(seg->MRow < row)
                    seg->MRow = row;
                if(seg->mRow > row)
                    seg->mRow = row;
                if(seg->MCol < col)
                    seg->MCol = col;
                if(seg->mCol > col)
                    seg->mCol = col;

                // Register new querry
                q.push(PUU(nextRow , nextCol));
            }
        }
    }
}

void Segmentation::createSegmentRelativePixelCount(Segment *seg, Mat img16bits, unsigned row, unsigned col, unsigned pixelCount)
{

    unsigned m_maxSampleSize = min(pixelCount,maxSampleSize);

    // Realocate memory if necsessary
    if(seg->result.rows < 3 || seg->result.cols < maxSampleSize)
    {
        #ifdef SEGMENTATION_MEMORORY_DEBUG
            cout << "createSegmentRelativePixelCount: Allocating memory for segments!!" << endl;
        #endif
        seg->result = Mat(3, maxSampleSize , CV_16UC1, Scalar(0));
    }

    // Initialize segment
    seg->N = 0;
    seg->MCol = seg->MRow = 0;
    seg->mRow = seg->mCol = 99999;

    // Initialize queue of BFS
    typedef pair<unsigned, unsigned> PUU;
    typedef pair<ushort, PUU> PUPUU;
    priority_queue< PUPUU> q;

    // Add first element
    q.push(PUPUU(img16bits.at<ushort>(row,col),PUU(row,col)));
    searchMask.at<uchar>(row,col) = 255;

    while(!q.empty() && seg->N < m_maxSampleSize)
    {
        row = q.top().second.first;
        col = q.top().second.second;
        q.pop();

        // Register element
        seg->result.at<ushort>(0,seg->N) = row;
        seg->result.at<ushort>(1,seg->N) = col;
        seg->result.at<ushort>(2,seg->N) = img16bits.at<ushort>(row,col);
        seg->N++;

        // Take min max col and row
        if(seg->MRow < row)
            seg->MRow = row;
        if(seg->mRow > row)
            seg->mRow = row;
        if(seg->MCol < col)
            seg->MCol = col;
        if(seg->mCol > col)
            seg->mCol = col;

        // Seach the neighbor pixels
        for(unsigned neighbor = 0 ; neighbor < 8 && seg->N < m_maxSampleSize ; neighbor++)
        {
            unsigned nextRow = row + neighborRow[neighbor],
                     nextCol = col + neighborCol[neighbor];

            // If neighbor exist on the image
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If neighbor wasn't visited
            if(img16bits.at<ushort>(nextRow , nextCol) >= 40 &&
               searchMask.at<uchar>(nextRow,nextCol) != 255)
            {
                // Register the visit of pixel
                searchMask.at<uchar>(nextRow,nextCol) = 255;

                // Register new querry
                q.push(PUPUU(img16bits.at<ushort>(nextRow , nextCol),
                            PUU(nextRow , nextCol))
                       );
            }
        }
    }
}

bool Segmentation::isPeak(Mat &img16bits, unsigned row, unsigned col)
{
    ushort myPixel, neighborPixel;
    if(row <= img16bits.rows ||
       col <= img16bits.cols)
    {
        myPixel = img16bits.at<ushort>(row,col);
    }
    else return false;

    for(unsigned neighbor = 0 ; neighbor < 8 ; neighbor++)
    {
        unsigned nextRow = row + neighborRow[neighbor],
                 nextCol = col + neighborCol[neighbor];

        // If neighbor isn't on image
        if(nextRow >= img16bits.rows ||
           nextCol >= img16bits.cols)
            continue;

        neighborPixel = img16bits.at<ushort>(nextRow,nextCol);
        if(neighborPixel >= myPixel)
            return false;
    }
    return true;
}

Scalar Segmentation::peakColor(Mat &img16bits, unsigned row, unsigned col)
{
    ushort myPixel, neighborPixel;
    if(row <= img16bits.rows ||
       col <= img16bits.cols)
    {
        myPixel = img16bits.at<ushort>(row,col);
    }
    else return Scalar(0,0,0);

    unsigned nP=0,nL=0,nE=0;
    for(unsigned neighbor = 0 ; neighbor < 8 ; neighbor++)
    {
        unsigned nextRow = row + neighborRow[neighbor],
                 nextCol = col + neighborCol[neighbor];

        // If neighbor isn't on image
        if(nextRow >= img16bits.rows ||
           nextCol >= img16bits.cols)
            continue;

        neighborPixel = img16bits.at<ushort>(nextRow,nextCol);
        if(neighborPixel > myPixel)
            nP++;
        else if(neighborPixel < myPixel)
            nL++;
        else nE++;
    }
    if(nL == 8)
    {
        return Scalar(0,0,255); // Red (Direct Peak!)
    }
    if(nL > 0 && nP == 0)
    {
        return Scalar(0,128,255); // Orange ( UP )
    }
    if(nL > nP)
    {
        return Scalar(0,255,255); // Yellow (In transition Up)
    }
    if(nL == nP)
    {
        return Scalar(0,255,0); // Green (In transition)
    }
    if(nE == 8)
    {
        return Scalar(255,255,255); // Withe ( Stable )
    }
    if(nP > 0 && nL == 0)
    {
        return Scalar(255,255,122); // Low Cyan (In transition Down)
    }    
    if(nP > nL)
    {
        return Scalar(211,0,146); // Purple ( Down )
    }
    if(nP == 8)
    {
        return Scalar(255,0,0); // Blue (Hole !)
    }

    return Scalar(0,0,0); // Undefined posible image bords
}

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
}

Segmentation::Segmentation():
    searchDistance(0),
    maxSampleSize(1200),
    minSampleSize(20),
    peakThreshold(210),
    maxPeakSize(200)
{

    cout << "Loading sonar image mask" << endl;
    imgMask = imread("../../GroundTruth/mask.png",CV_LOAD_IMAGE_GRAYSCALE);

}

Segmentation::~Segmentation()
{
    // Delete Segments
    for(unsigned i = 0 ; i < segments.size(); i++)
    {
        delete segments[i];
    }
}

void Segmentation::segment(Mat &img16bits, vector<Segment *> *sg)
{
#ifdef SEGMENTATION_DRWING_DEBUG
    //    Mat result(img16bits.rows , img16bits.cols, CV_8UC3, Scalar(0,0,0));
    Mat result;
    img16bits.convertTo(result,CV_8UC1);
    cvtColor(result,result,CV_GRAY2BGR);

    unsigned maxROISize=0, minROISize=maxSampleSize;

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);
#endif

#ifdef SEGMENTATION_EXECUTION_TIME_DEBUG
    Cronometer cr;
    cr.reset();
#endif

    /* Initialize Mask of Visit */
    resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

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
                createSegment(seg,img16bits,row,col);
//                createSegmentDist(seg,img16bits,row,col);
//                createBorderSegment(seg,img16bits,row,col,searchThreshold);

                if(seg->N < minSampleSize)
                {
                    continue;
                }

                segCount++;

            #ifdef SEGMENTATION_DRWING_DEBUG
                seg->drawSegment(result,colorMap[segCount%6]);

                if(seg->N > maxROISize)
                    maxROISize = seg->N;
                if(seg->N < minROISize)
                    minROISize = seg->N;
            #endif

                // Add seg to answer
                sg->push_back(seg);
            }
        }
    }
#ifdef SEGMENTATION_DRWING_DEBUG
    imshow("Thresholded Image", result);
    cout << "ROI Extraction Result:" << endl
         << "max ROI Size = " << maxROISize << endl
         << "min ROI Size = " << minROISize << endl
         << "number of ROI found " << segCount << endl
         << endl;
#endif

#ifdef SEGMENTATION_EXECUTION_TIME_DEBUG
    cout << "ROI Extraction Executation Time " << cr.read() << endl;
#endif
}

void Segmentation::segmentDouble(Mat &img16bits, vector<Segment *> *sg)
{
    #ifdef SEGMENTATION_DRWING_DEBUG
    Mat result(img16bits.rows , img16bits.cols, CV_8UC3, Scalar(0,0,0));

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);
    #endif

    /* Initialize Mask of Visit */
    resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

    /* Search for high intensity pixels */
    unsigned linJump = 1, colJump = 1, segCount=0,
            secPixelThreshold = pixelThreshold/2, secSearchThreshold = searchThreshold/2;
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
                createBorderSegment(seg,img16bits,row,col,searchThreshold);

                if(seg->N < minSampleSize)
                {
                    continue;
                }

                segCount++;

                #ifdef SEGMENTATION_DRWING_DEBUG
                seg->drawSegment(result,colorMap[segCount%6]);
                #endif
                // Add seg to answer
                sg->push_back(seg);
            }
        }
    }

    for(unsigned row = 0 ; row < img16bits.rows ; row+=linJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(searchMask.at<uchar>(row,col) == 0 &&
               img16bits.at<ushort>(row,col) >= secPixelThreshold)
            {
                // Search the segment on image
                seg = segment(segCount);
        //                createSegment(seg,img16bits,row,col);
        //                createSegmentDist(seg,img16bits,row,col);
                createBorderSegment(seg,img16bits,row,col,secSearchThreshold);

                if(seg->N < minSampleSize)
                {
                    continue;
                }

                segCount++;

                #ifdef SEGMENTATION_DRWING_DEBUG
                seg->drawSegment(result,colorMap[segCount%6]);
                #endif

                // Add seg to answer
                sg->push_back(seg);
            }
        }
    }
    #ifdef SEGMENTATION_DRWING_DEBUG
    imshow("Thresholded Image", result);
    #endif
}

void Segmentation::segmentWithPeaks(Mat &img16bits, vector<Segment *> *sg)
{
    #ifdef SEGMENTATION_DRWING_DEBUG
    Mat result(img16bits.rows , img16bits.cols, CV_8UC3, Scalar(0,0,0));

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);
    #endif

    peaks.clear();
    /* Search for high intensity pixels */
    unsigned linJump = 1, colJump = 1;
    for(unsigned row = 0 ; row < img16bits.rows ; row+=linJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(img16bits.at<ushort>(row,col) >= peakThreshold &&
                 isPeak(img16bits,row,col))
            {
                peaks.push_back(PUPUU(img16bits.at<ushort>(row,col),
                                   PUU(row,col)));
            }
        }
    }
    cout << "nPeak = " << peaks.size() << endl;
    sort(peaks.begin(), peaks.end(), greater<PUPUU>());

    /* Initialize Mask of Visit */
    resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

    Segment * seg;
    unsigned segCount=0;
    for(unsigned i = 0 ; i < peaks.size(); i++)
    {
        unsigned row = peaks[i].second.first,
                 col = peaks[i].second.second;

        if(searchMask.at<uchar>(row,col) == 0)
        {
            // Search the segment on image
            seg = segment(segCount);
            createSegmentRelativePixelCount(seg,img16bits,row,col,maxPeakSize);

            if(seg->N < minSampleSize)
            {
                continue;
            }
            segCount++;

            #ifdef SEGMENTATION_DRWING_DEBUG
                seg->drawSegment(result,colorMap[segCount%6]);
            #endif

            // Add seg to answer
            sg->push_back(seg);
        }
    }

    #ifdef SEGMENTATION_DRWING_DEBUG
    imshow("Peak Image", result);
#endif

}

void Segmentation::segmentWithTR(Mat &img16bits, vector<Segment *> *sg)
{
    // Algorithm parameters
    int sonVerticalPosition =1;

    unsigned nBeams = 20,startBin=20,
            acceptPeakHeight=110; // Minimum height to a peak be acceptable

    float bearing = 130,
          endPeakTax=0.6, // If a peak down low tham resetTax of height, reset a peak.
          takePeakPercent=0.5; // Height percent of acceptable peak used for threshold


    unsigned nBins = img16bits.rows-startBin-11;
    float beamAng,
          cAng = -bearing/2.f,
          endAng = bearing/2.f;

    if(nBeams>1)
        beamAng = bearing/(nBeams-1);
    else beamAng = 2*bearing; // Infinity!

#ifdef SEGMENTATION_DRWING_DEBUG
    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);

//    Mat result(img16bits.rows, img16bits.cols, CV_8UC3, Scalar(0,0,0));

    Mat result;
    img16bits.convertTo(result,CV_8UC1);
    cvtColor(result,result,CV_GRAY2BGR);
#endif

    Point2f sonarPos(img16bits.cols/2.f, img16bits.rows+sonVerticalPosition);

    unsigned beam=nBeams-1;

    /* Initialize Mask of Visit */
    resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

    /* Search for high intensity pixels */
    unsigned segCount=0;
    Segment *seg;

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
                 lastThreshold=99999;

        // For each bin
        for(unsigned bin = nBins-1 ; bin < nBins; bin--)// unsigned overflow!
        {
            beamPos+= beamDir;

            // If pixel was visited
            if(searchMask.at<uchar>(beamPos.y,beamPos.x) == 255)
            {  // Forget all informations about peaks
                minBinI = 99999;
                maxBinI = 0;
                continue;
            }// if not, work with its

            // Save pixel intensity
            binI = img16bits.at<ushort>(beamPos.y,beamPos.x);


            #ifdef SEGMENTATION_DRWING_DEBUG
            // Mark on result img the position of visited pixel
//            result.at<Vec3b>(beamPos.y,beamPos.x)[0] = colorMap[beam%6].val[0];
//            result.at<Vec3b>(beamPos.y,beamPos.x)[1] = colorMap[beam%6].val[1];
//            result.at<Vec3b>(beamPos.y,beamPos.x)[2] = colorMap[beam%6].val[2];
            #endif

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

                        // Compute the peak position on sonar XY image
                        float vecUnit = nBins - bin - maxBinN;
                        Point2f peakPosition(beamPos - beamDir * vecUnit);

                        // Search the segment on image
                        seg = segment(segCount);
//                        createBorderSegment(seg,img16bits,peakPosition.y,peakPosition.x,lastThreshold);
                        searchThreshold = lastThreshold;
                        createSegment(seg,img16bits,peakPosition.y,peakPosition.x);

                        // If segment is greater tham minimum acceptable segment size
                        if(seg->N >= minSampleSize)
                        {
                            segCount++;

                            #ifdef SEGMENTATION_DRWING_DEBUG
                                seg->drawSegment(result,colorMap[segCount%6]);
                            #endif

                            // Add seg to answer
                            sg->push_back(seg);
                        }

                    }

                    // Reset Peak
                    minBinI=binI;
                    minBinN= nBins - bin -1;
                    maxBinI=0;

                }
            }
        }

        beam--;
    }

    #ifdef SEGMENTATION_DRWING_DEBUG
    imshow("TR image result", result);
    #endif
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
                createBorderSegment(seg,img16bits,row,col,searchThreshold);

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

bool Segmentation::createPeakImg(Mat &img16bits)
{
    Mat result(img16bits.rows , img16bits.cols, CV_8UC3, Scalar(0,0,0));

//    GaussianBlur(img16bits,img16bits,Size(5,5),4);

    medianBlur(img16bits,img16bits,5);

    Scalar colorMap[6];
    colorMap[0] = Scalar(255,  0,  0);
    colorMap[1] = Scalar(  0,255,  0);
    colorMap[2] = Scalar(  0,  0,255);
    colorMap[3] = Scalar(255,255,  0);
    colorMap[4] = Scalar(255,  0,255);
    colorMap[5] = Scalar(0  ,255,255);

    typedef pair<unsigned,unsigned> PUU;
    typedef pair<unsigned,PUU> PUPUU;

    peaks.clear();

    /* Search for high intensity pixels */
    unsigned linJump = 1, colJump = 1;
    for(unsigned row = 0 ; row < img16bits.rows ; row+=linJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(imgMask.at<uchar>(row,col) == 0) continue;

            Scalar color(127,127,127);
//            if(img16bits.at<ushort>(row,col) >= peakThreshold)
//            {
//                color = peakColor(img16bits, row, col);
//            }
            color = peakColor(img16bits, row, col);

            if(color.val[0] == 0 &&
               color.val[1] == 0 &&
               color.val[2] == 255 )
            {
                peaks.push_back(PUPUU(img16bits.at<ushort>(row,col),
                                   PUU(row,col)));
            }

            result.at<Vec3b>(row,col)[0] = color.val[0];
            result.at<Vec3b>(row,col)[1] = color.val[1];
            result.at<Vec3b>(row,col)[2] = color.val[2];

        }
    }
    cout << "nPeak = " << peaks.size() << endl;

    imshow("Peak Image", result);
    waitKey();

    result = Scalar(0,0,0);
    sort(peaks.begin(), peaks.end(), greater<PUPUU>());

    /* Initialize Mask of Visit */
    resetMask(img16bits.rows,img16bits.cols);

    vector<Segment *> sg;
    Segment * seg;
    unsigned segCount=0;
    for(unsigned i = 0 ; i < peaks.size(); i++)
    {
        unsigned row = peaks[i].second.first,
                 col = peaks[i].second.second;

        if(searchMask.at<uchar>(row,col) == 0)
        {
            // Search the segment on image
            seg = segment(segCount);
            createSegmentRelativePixelCount(seg,img16bits,row,col,maxPeakSize);

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
    imshow("Peak Image", result);

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

    Drawing::makePlotInfo(mPlot,Size2i(1200,800),
                          0,nBins-1,0,850,plotInfo);

    img16bits.convertTo(mask,CV_8UC1);
    cvtColor(mask,mask,CV_GRAY2BGR);

    unsigned beam=nBeams-1;

    vector<float> intensitys, beams(nBins),xTake, yTake;

    for(unsigned i = 0 ; i < nBins ; i++)
        beams[i] = i;

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
                 peakCount=0;

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

                        xTake.push_back(minBinN);
                        yTake.push_back(lastThreshold);

                        xTake.push_back(nBins - bin -1);
                        yTake.push_back(lastThreshold);

                        Drawing::holdPlot(plotInfo,mPlot,
                                            xTake,yTake,
                                            colorMap[beam%6],1);

                        xTake.clear();
                        yTake.clear();

                        xTake.push_back(maxBinN);
                        yTake.push_back(maxBinI);

                        Drawing::holdPlotCircle(plotInfo,mPlot,
                                            xTake,yTake,
                                            colorMap[peakCount%6],-1);

                        Drawing::holdPlotCircle(plotInfo,mPlot,
                                            xTake,yTake,
                                                Scalar(0,0,0),1);

                        xTake.clear();
                        yTake.clear();

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
                        xTake.push_back(minBinN);
                        yTake.push_back(minBinI);

                        xTake.push_back(nBins - bin -1);
                        yTake.push_back(minBinI);

                        Drawing::holdPlot(plotInfo,mPlot,
                                            xTake,yTake,
                                          Scalar(0,0,255),1);

                        xTake.clear();
                        yTake.clear();

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

        Drawing::holdPlot(plotInfo,mPlot,
                          beams,intensitys,
                          colorMap[beam%6]);

        intensitys.clear();

        beam--;
    }

    imshow("TR image mask", mask);
    imshow("TR plot", mPlot);
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
