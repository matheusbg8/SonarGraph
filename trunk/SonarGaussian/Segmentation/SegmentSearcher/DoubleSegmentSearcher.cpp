#include "DoubleSegmentSearcher.h"

#include "Segmentation.h"

DoubleSegmentSearcher::DoubleSegmentSearcher():
    rowJump(1),colJump(1),
    firstRhoLinear(200),secondRhoLinear(150),
    minSampleSize(10)
{
}

void DoubleSegmentSearcher::load(ConfigLoader &config)
{
    int vi;

    if(config.getInt("DoubleSegmentSearcher","rowJump",&vi))
    {
        rowJump = vi;
    }

    if(config.getInt("DoubleSegmentSearcher","colJump",&vi))
    {
        colJump = vi;
    }

    if(config.getInt("DoubleSegmentSearcher","firstRhoLinear",&vi))
    {
        firstRhoLinear = vi;
    }

    if(config.getInt("DoubleSegmentSearcher","secondRhoLinear",&vi))
    {
        secondRhoLinear = vi;
    }

    if(config.getInt("DoubleSegmentSearcher","minSampleSize",&vi))
    {
        minSampleSize = vi;
    }
}


void DoubleSegmentSearcher::segment(Mat &img16bits, vector<Segment *> *sg)
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
    m_seg->resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

    unsigned segCount=0;
    Segment *seg;

    /* Search for high intensity pixels */
    for(unsigned row = 0 ; row < img16bits.rows ; row+=rowJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(searchMask->at<uchar>(row,col) == 0 &&
               img16bits.at<ushort>(row,col) >= firstRhoLinear)
            {
                // Search the segment on image
                seg = m_seg->segment(segCount);

                m_extractor->createSegment(seg,img16bits,row,col);

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

    for(unsigned row = 0 ; row < img16bits.rows ; row+=rowJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(searchMask->at<uchar>(row,col) == 0 &&
               img16bits.at<ushort>(row,col) >= secondRhoLinear)
            {
                // Search the segment on image
                seg = m_seg->segment(segCount);

                m_extractor->createSegment(seg,img16bits,row,col);

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
