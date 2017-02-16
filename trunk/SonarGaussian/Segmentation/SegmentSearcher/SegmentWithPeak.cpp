#include "SegmentWithPeak.h"

#include "Segmentation.h"

SegmentWithPeak::SegmentWithPeak():
    rowJump(1),colJump(1),rhoLinear(213),minSampleSize(120)
{
}

void SegmentWithPeak::load(ConfigLoader &config)
{
    int vi;

    if(config.getInt("SegmentWithPeak","rowJump",&vi))
    {
        rowJump = vi;
    }

    if(config.getInt("SegmentWithPeak","colJump",&vi))
    {
        colJump = vi;
    }

    if(config.getInt("SegmentWithPeak","rhoLinear",&vi))
    {
        rhoLinear = vi;
    }

    if(config.getInt("SegmentWithPeak","minSampleSize",&vi))
    {
        minSampleSize = vi;
    }
}

Scalar SegmentWithPeak::peakColor(Mat &img16bits, unsigned row, unsigned col)
{
    ushort myPixel, neighborPixel;

    if(row <= img16bits.rows ||
       col <= img16bits.cols)
    {
        myPixel = img16bits.at<ushort>(row,col);
    }
    else return Scalar(0,0,0);

    unsigned nGreater=0,nLower=0,nEqual=0;
    for(unsigned neighbor = 0 ; neighbor < 8 ; neighbor++)
    {
        unsigned nextRow = row + Segmentation::neighborRow[neighbor],
                 nextCol = col + Segmentation::neighborCol[neighbor];

        // If neighbor isn't on image
        if(nextRow >= img16bits.rows ||
           nextCol >= img16bits.cols)
            continue;

        neighborPixel = img16bits.at<ushort>(nextRow,nextCol);

        if(neighborPixel > myPixel)
            nGreater++;
        else if(neighborPixel < myPixel)
            nLower++;
        else nEqual++;
    }

    if(nLower == 8)
    {
        return Scalar(0,0,255); // Red (Direct Peak!)
    }
    if(nLower > 0 && nGreater == 0)
    {
        return Scalar(0,128,255); // Orange ( UP Peak)
    }
    if(nLower > nGreater)
    {
        return Scalar(0,255,255); // Yellow (In transition Up Peak)
    }
    if(nLower == nGreater)
    {
        return Scalar(0,255,0); // Green (In transition)
    }
    if(nEqual == 8)
    {
        return Scalar(255,255,255); // Withe ( Stable plan pixel)
    }
    if(nGreater > 0 && nLower == 0)
    {
        return Scalar(255,255,122); // Low Cyan (In transition Down)
    }
    if(nGreater > nLower)
    {
        return Scalar(211,0,146); // Purple ( Down )
    }
    if(nGreater == 8)
    {
        return Scalar(255,0,0); // Blue (Hole !)
    }

    return Scalar(0,0,0); // Undefined posible image bords
}

bool SegmentWithPeak::isPeak(Mat &img16bits, unsigned row, unsigned col)
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
        unsigned nextRow = row + Segmentation::neighborRow[neighbor],
                 nextCol = col + Segmentation::neighborCol[neighbor];

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

bool SegmentWithPeak::createPeakImg(Mat &img16bits)
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
    for(unsigned row = 0 ; row < img16bits.rows ; row+=rowJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(imgMask->at<uchar>(row,col) == 0)
                continue;

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
    m_seg->resetMask(img16bits.rows,img16bits.cols);

    vector<Segment *> sg;
    Segment * seg;
    unsigned segCount=0;
    for(unsigned i = 0 ; i < peaks.size(); i++)
    {
        unsigned row = peaks[i].second.first,
                 col = peaks[i].second.second;

        if(searchMask->at<uchar>(row,col) == 0)
        {
            // Search the segment on image
            seg = m_seg->segment(segCount);

            m_extractor->createSegment(seg,img16bits,row,col);

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



/**
 * @brief Segmennt seaching peaks, one peak is a pixel how has
 * greater intensity than our neighbor pixels.
 *
 * @param img16bits
 * @param sg
 */
void SegmentWithPeak::segment(Mat &img16bits, vector<Segment *> *sg)
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
    for(unsigned row = 0 ; row < img16bits.rows ; row+=rowJump)
    {
        for(unsigned col = 0 ; col < img16bits.cols ; col+=colJump)
        {
            if(imgMask->at<uchar>(row,col) > 0 &&
               img16bits.at<ushort>(row,col) >= rhoLinear &&
               isPeak(img16bits,row,col))
            {
                peaks.push_back(PUPUU(img16bits.at<ushort>(row,col),
                                   PUU(row,col)));
            }
        }
    }

//    cout << "nPeak = " << peaks.size() << endl;
    sort(peaks.begin(), peaks.end(), greater<PUPUU>());

    /* Initialize Mask of Visit */
    m_seg->resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

    Segment * seg=0x0;
    unsigned segCount=0;
    for(unsigned i = 0 ; i < peaks.size(); i++)
    {
        unsigned row = peaks[i].second.first,
                 col = peaks[i].second.second;

        if(searchMask->at<uchar>(row,col) == 0)
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

    #ifdef SEGMENTATION_DRWING_DEBUG
    imshow("Peak Image", result);
    #endif
}

