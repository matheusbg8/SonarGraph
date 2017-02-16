#include "RelativeSegmentExtractor.h"

#include "Segmentation.h"

#include <queue>
using namespace std;


RelativeSegmentExtractor::RelativeSegmentExtractor()
{

}

/**
 * @brief Search for neighbord pixels with intensity inside a
 * range of intensity relative to started pixel intensity
 *
 * @param seg
 * @param img16bits
 * @param row
 * @param col
 */
void RelativeSegmentExtractor::createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
    unsigned relativeThreshold;
    if(img16bits.at<ushort>(row,col) > decaiment)
    {
        relativeThreshold = img16bits.at<ushort>(row,col) - decaiment;
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
    searchMask->at<uchar>(row,col) = 255;

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
            unsigned nextRow = row + Segmentation::neighborRow[neighbor],
                     nextCol = col + Segmentation::neighborCol[neighbor];

            // If neighbor exist on the image
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If neighbor wasn't visited
            if(img16bits.at<ushort>(nextRow , nextCol) >= relativeThreshold
               && searchMask->at<uchar>(nextRow,nextCol) != 255)
            {
                // Register the visit of pixel
                searchMask->at<uchar>(nextRow,nextCol) = 255;

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

void RelativeSegmentExtractor::load(ConfigLoader &config)
{

}

void RelativeSegmentExtractor::setThreshold(unsigned threshold)
{

}
