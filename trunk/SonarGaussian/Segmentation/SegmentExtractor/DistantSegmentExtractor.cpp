#include "DistantSegmentExtractor.h"

#include <queue>
#include "Segmentation/Segmentation.h"

using namespace std;

class SegDistSearchStruct
{
public:
    unsigned row,
             col;
    int      dist;
    SegDistSearchStruct(unsigned row, unsigned col, int dist):
        row(row),col(col),dist(dist){}
    void get(unsigned &prow, unsigned &pcol, int &pdist)
    {
        prow = row;
        pcol = col;
        pdist = dist;
    }
};

DistantSegmentExtractor::DistantSegmentExtractor():
    maxSampleSize(1200),rhoRecursive(215),searchDistance(3)
{
}

void DistantSegmentExtractor::createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
#ifdef SEGMENTATION_SEARCH_DEBUG
    Mat &imgDebug = m_seg->imgDebug;
    if(imgDebug.rows == 0)
    {
        img16bits.convertTo(imgDebug,CV_8UC1);
        cvtColor(imgDebug,imgDebug,CV_GRAY2BGR);
    }
#endif

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
    queue< SegDistSearchStruct > q, afterBoardQ;
    int dist=0;

    // Add first element
    q.push(SegDistSearchStruct(row,col,searchDistance));
    searchMask->at<uchar>(row,col) = 255u;

    bool isBorder=false;

    while((!q.empty() || !afterBoardQ.empty())
          && seg->N < maxSampleSize)
    {
        if(!q.empty())
        {
            q.front().get(row,col,dist);
            q.pop();
        }else
        {
            afterBoardQ.front().get(row,col,dist);
            afterBoardQ.pop();
        }

        isBorder = false;

        // Seach the neighbor pixels
        for(unsigned neighbor = 0 ; neighbor < 8 && seg->N < maxSampleSize ; neighbor++)
        {
            unsigned nextRow = row + Segmentation::neighborRow[neighbor],
                     nextCol = col + Segmentation::neighborCol[neighbor];

            // If neighbor isn't on image (unsigned comparison)
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If we can visit this neighbor pixel
            if(searchMask->at<uchar>(nextRow,nextCol) != 255)
            {
                if(img16bits.at<ushort>(nextRow , nextCol) >= rhoRecursive)
                {
                    // Register the visit of pixel
                    searchMask->at<uchar>(nextRow,nextCol) = 255;

                    // Register new querry
                    q.push(SegDistSearchStruct(nextRow , nextCol,searchDistance));


                #ifdef SEGMENTATION_SEARCH_DEBUG
//                    imgDebug.at<Vec3b>(nextRow, nextCol) = Vec3b(255,0,0);
                    unsigned debCol = nextCol*3;
                    imgDebug.at<uchar>(nextRow, debCol) = 255;
                    imgDebug.at<uchar>(nextRow, debCol+1) = 0;
                    imgDebug.at<uchar>(nextRow, debCol+2) = 0;
                #endif

                }else if(dist > 0 && img16bits.at<ushort>(nextRow , nextCol) > 5) // If we can jump for then
                {
                    // Register the visit of pixel
                    searchMask->at<uchar>(nextRow,nextCol) = 255;

                    if( dist == searchDistance) // I'm a boarder
                    {
                        isBorder = true;
                    }

                    // Register new querry
                    afterBoardQ.push(SegDistSearchStruct(nextRow , nextCol,dist-1));

                #ifdef SEGMENTATION_SEARCH_DEBUG
//                    imgDebug.at<Vec3b>(nextRow, nextCol) = Vec3b(0,0,255);

                    unsigned debCol = nextCol*3;
                    imgDebug.at<uchar>(nextRow, debCol) = 0;
                    imgDebug.at<uchar>(nextRow, debCol+1) = 0;
                    imgDebug.at<uchar>(nextRow, debCol+2) = 255;
                #endif

                }
            }
        }

        if(isBorder) // I'm a boarder
        {
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

        #ifdef SEGMENTATION_SEARCH_DEBUG
//            imgDebug.at<Vec3b>(row, col) = Vec3b(0,255,0);
            unsigned debCol = col*3;
            imgDebug.at<uchar>(row, debCol) = 0;
            imgDebug.at<uchar>(row, debCol+1) = 255;
            imgDebug.at<uchar>(row, debCol+2) = 0;
        #endif
        }

    #ifdef SEGMENTATION_SEARCH_DEBUG
//        imshow("createSegDist Search Debug", imgDebug);
//        waitKey(1);
    #endif
    }
#ifdef SEGMENTATION_SEARCH_DEBUG
    imshow("createSegDist Search Debug", imgDebug);
    cout << "Creat seg Dist end!!" << endl;
//    waitKey(0);
#endif
}

void DistantSegmentExtractor::load(ConfigLoader &config)
{
    int vi;

    if(config.getInt("General","MaxSampleSize",&vi))
    {
        maxSampleSize = vi;
    }

    if(config.getInt("DistantSegmentExtractor","rhoRecursive",&vi))
    {
        rhoRecursive = vi;
    }

    if(config.getInt("DistantSegmentExtractor","searchDistance",&vi))
    {
        searchDistance = vi;
    }
}

void DistantSegmentExtractor::setThreshold(unsigned threshold)
{
    rhoRecursive = threshold;
}
