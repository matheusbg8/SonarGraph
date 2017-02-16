#include "LinearSegmentSearcher.h"

#include "Segmentation/Segmentation.h"

LinearSegmentSearcher::LinearSegmentSearcher():
    rowJump(1), colJump(1),rhoLinear(220),minSampleSize(10)
{
}

void LinearSegmentSearcher::load(ConfigLoader &config)
{
    int vi;

    if(config.getInt("LinearSegmentSearcher","rowJump",&vi))
    {
        rowJump = vi;
    }

    if(config.getInt("LinearSegmentSearcher","colJump",&vi))
    {
        colJump = vi;
    }

    if(config.getInt("LinearSegmentSearcher","rhoLinear",&vi))
    {
        rhoLinear = vi;
    }

    if(config.getInt("LinearSegmentSearcher","minSampleSize",&vi))
    {
        minSampleSize = vi;
    }
}

/**
 * @brief Create the segments of img16bits on sg vector,
 *  you should not delete this vector never!
 *   and this segments will be valid until new call of this method
 *
 * @param img16bits
 * @param sg
 */
void LinearSegmentSearcher::segment(Mat &img16bits, vector<Segment *> *sg)
{
#ifdef SEGMENTATION_DRWING_DEBUG
        Mat result(img16bits.rows , img16bits.cols, CV_8UC3, Scalar(0,0,0));
//    Mat result;
//    img16bits.convertTo(result,CV_8UC1);
//    cvtColor(result,result,CV_GRAY2BGR);

    unsigned maxROISize=0, minROISize=9999999;

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
    m_seg->resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

    /* Search for high intensity pixels */
    unsigned segCount=0;
    Segment *seg;
    for(unsigned row = 0 ; row < img16bits.rows ; row+=rowJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(searchMask->at<uchar>(row,col) == 0 &&
               imgMask->at<uchar>(row,col) != 0 &&
               img16bits.at<ushort>(row,col) >= rhoLinear)
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
