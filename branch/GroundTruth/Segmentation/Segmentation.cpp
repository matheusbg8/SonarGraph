#include "Segmentation.h"
#include <iostream>
#include <queue>

const int Segmentation::neighborRow[8] = { -1, -1, -1,  0,  0,  1, 1, 1};
const int Segmentation::neighborCol[8] = { -1,  0,  1, -1,  1, -1, 0, 1};

void Segmentation::createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
    // Realocate memory if necsessary
    if(seg->result.rows < 3 && seg->result.cols < maxSampleSize)
    {
        cout << "Allocing memory for segments!!" << endl;
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
            if(img16bits.at<ushort>(nextRow , nextCol) >= searchThreshold
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

void Segmentation::createSegmentDist(Segment *seg, Mat img16bits, unsigned row, unsigned col)
{
    // Realocate memory if necsessary
    if(seg->result.rows < 3 && seg->result.cols < maxSampleSize)
    {
        cout << "Allocing memory for segments!!" << endl;
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

Segment *Segmentation::segment(unsigned id)
{
    unsigned segSize = segments.size();
    if(id < segSize)
    {
        return segments[id];
    }
    cout << "Allocing more memory to segments!!" << endl;
    segments.resize((segSize+1)*2);

    for(segSize ; segSize < segments.size() ; segSize++)
    {
        segments[segSize] = new Segment;
    }

    return segments[id];
}

void Segmentation::resetMask(unsigned rows, unsigned cols)
{
    searchMask = Mat(rows, cols, CV_8UC1, Scalar(0));
}

Segmentation::Segmentation():
    searchDistance(0)
{

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

                if(seg->N < minSampleSize)
                {
                    continue;
                }

                segCount++;

                // Add seg to answer
                sg->push_back(seg);
            }
        }
    }
}
