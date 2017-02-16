#include "DistantSegmentExtractorV2.h"

#include <queue>
#include <stack>
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

DistantSegmentExtractorV2::DistantSegmentExtractorV2()
{
}

void DistantSegmentExtractorV2::get3Neighbor(int id,
                                             unsigned refRow, unsigned refCol,
                                             unsigned *lastRow, unsigned *lastCol,
                                             unsigned *currentRow, unsigned *currentCol,
                                             unsigned *nextRow, unsigned *nextCol)
{
    int lastId = id-1, nextId = id+1;
    if(lastId<0) lastId = 7; if(nextId>7) nextId = 0;

    *currentRow = refRow + Segmentation::neighborRow[id];
    *currentCol = refCol + Segmentation::neighborCol[id];

    *lastRow = refRow + Segmentation::neighborRow[lastId];
    *lastCol = refCol + Segmentation::neighborCol[lastId];

    *nextRow = refRow + Segmentation::neighborRow[nextId];
    *nextCol = refCol + Segmentation::neighborCol[nextId];

}

void DistantSegmentExtractorV2::createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col)
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
        queue< SegDistSearchStruct > q;

        int dist=0;

        // Add first element
        q.push(SegDistSearchStruct(row,col,searchDistance));
        searchMask->at<uchar>(row,col) = 255u;

        bool isInsideBorder=false;

        while(!q.empty() && seg->N < maxSampleSize)
        {
            q.front().get(row,col,dist);
            q.pop();
            #ifdef SEGMENTATION_SEARCH_DEBUG
//                imgDebug.at<Vec3b>(row, col) = Vec3b(255,0,255); // Magenta
//                imshow("createSegDist Search Debug", imgDebug);
            #endif

            // Seach the neighbor pixels
            for(unsigned neighbor = 0 ; neighbor < 8 && seg->N < maxSampleSize ; neighbor++)
            {
                unsigned nextRow = row + Segmentation::neighborRow[neighbor],
                         nextCol = col + Segmentation::neighborCol[neighbor];

                // If neighbor isn't on image (unsigned comparison)
                if(nextRow >= img16bits.rows ||
                   nextCol >= img16bits.cols)
                    continue;

                if(img16bits.at<ushort>(nextRow , nextCol) >= rhoRecursive)
                {
                    // If can not visited, register a visit
                    if(searchMask->at<uchar>(nextRow,nextCol) != 255)
                    {
                        // Register visit of pixel
                        searchMask->at<uchar>(nextRow,nextCol) = 255;
                        q.push(SegDistSearchStruct(nextRow , nextCol,searchDistance));
                        #ifdef SEGMENTATION_SEARCH_DEBUG
                            imgDebug.at<Vec3b>(nextRow, nextCol) = Vec3b(255,0,0); // Blue
//                            imshow("createSegDist Search Debug", imgDebug);
//                            waitKey();
                        #endif
                    }

                }else if(dist > 0 && img16bits.at<ushort>(nextRow , nextCol) > 5) // If we can jump over then
                {
                    // If can not visited, register a visit
                    if(searchMask->at<uchar>(nextRow,nextCol) != 255)
                    {
                        // Register visit of pixel
                        searchMask->at<uchar>(nextRow,nextCol) = 255;
                        q.push(SegDistSearchStruct(nextRow , nextCol,dist-1));
                        #ifdef SEGMENTATION_SEARCH_DEBUG
                            imgDebug.at<Vec3b>(nextRow, nextCol) = Vec3b(0,0,255); // Red
//                            imshow("createSegDist Search Debug", imgDebug);
//                            waitKey();
                        #endif
                    }

                    if( dist == searchDistance) // I'm inside and my neighbor is outside
                    {
                        // I'm inside board and my neighboard is outside board
                        isInsideBorder = true;
                    }
                }
            }

            if(isInsideBorder) // I'm a boarder
            {
                isInsideBorder = false;

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
                    imgDebug.at<Vec3b>(row, col) = Vec3b(0,255,0);
//                    imshow("createSegDist Search Debug", imgDebug);
//                    waitKey();
                #endif
            }

        #ifdef SEGMENTATION_SEARCH_DEBUG
//            imshow("createSegDist Search Debug", imgDebug);
//            waitKey();
        #endif
        }

    #ifdef SEGMENTATION_SEARCH_DEBUG
        imshow("createSegDist Search Debug", imgDebug);
        cout << "Creat seg Dist end!!" << endl;
//        waitKey(1);
    #endif
}

void DistantSegmentExtractorV2::load(ConfigLoader &config)
{
    int vi;

    if(config.getInt("General","MaxSampleSize",&vi))
    {
        maxSampleSize = vi;
    }

    if(config.getInt("DistantSegmentExtractorV2","rhoRecursive",&vi))
    {
        rhoRecursive = vi;
    }

    if(config.getInt("DistantSegmentExtractorV2","searchDistance",&vi))
    {
        searchDistance = vi;
    }
}

void DistantSegmentExtractorV2::setThreshold(unsigned threshold)
{
    rhoRecursive = threshold;
}
