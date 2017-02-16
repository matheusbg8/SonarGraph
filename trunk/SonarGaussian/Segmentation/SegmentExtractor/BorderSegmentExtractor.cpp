#include "BorderSegmentExtractor.h"
#include "Segmentation/Segmentation.h"

#include<queue>
using namespace std;

BorderSegmentExtractor::BorderSegmentExtractor()
{
}


/**
 * @brief Search for neighbord pixels
 * with intensity more than searchThreshold
 *
 * @param seg
 * @param img16bits
 * @param row
 * @param col
 */
void BorderSegmentExtractor::createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
    // Realocate memory if necsessary
    if(seg->result.rows < 3 || seg->result.cols < maxSampleSize)
    {
        #ifdef SEGMENTATION_MEMORORY_DEBUG
            cout << "createBorderSegment: Allocating memory for segments!!" << endl;
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
            unsigned nextRow = row + Segmentation::neighborRow[neighbor],
                     nextCol = col + Segmentation::neighborCol[neighbor];

            // If neighbor exist on the image
            if(nextRow >= img16bits.rows ||
               nextCol >= img16bits.cols)
                continue;

            // If neighbor wasn't visited
            if(searchMask->at<uchar>(nextRow,nextCol) != 255)
            {
                if(img16bits.at<ushort>(nextRow , nextCol) >= searchThreshold)
                {
                    // Register the visit of pixel
                    searchMask->at<uchar>(nextRow,nextCol) = 255;

                    // Register new querry
                    q.push(PUU(nextRow , nextCol));
                }else isBorder = true;
            }
        }

        if(isBorder)
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
        }
    }
}

void BorderSegmentExtractor::load(ConfigLoader &config)
{

}

void BorderSegmentExtractor::setThreshold(unsigned threshold)
{
    searchThreshold = threshold;
}
