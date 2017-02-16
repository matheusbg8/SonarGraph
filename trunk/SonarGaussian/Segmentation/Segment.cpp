#include "Segment.h"
#include <iostream>


/**
 * @brief Draw pixels of segment on image BGR
 *
 * @param bgrImg
 * @param color
 */
void Segment::drawSegment(Mat &bgrImg, const Scalar &color)
{
    for(unsigned i = 0 ; i < N ; i++)
    {
        // Here:
        //   result.at<ushort>(0, k ) - Acess the row of K-th pixel found by floodfill search
        //   result.at<ushort>(1, k ) - Acess the col of K-th pixel found by floodfill search
        //   result.at<ushort>(2, k ) - Acess the pixel intensity of K-th pixel found by floodfill search
        /// @todo - Acessing uchar Mat::data[] is faster than at<type>()

        unsigned pixelRow = result.at<ushort>(0,i),
                pixelCol = result.at<ushort>(1,i),
                pixelColStart = pixelCol*3;

        if(pixelRow < bgrImg.rows &&
           pixelCol < bgrImg.cols)
        {
            bgrImg.at<uchar>(pixelRow, pixelColStart) = color.val[0];
            bgrImg.at<uchar>(pixelRow, pixelColStart+1) = color.val[1];
            bgrImg.at<uchar>(pixelRow, pixelColStart+2) = color.val[2];
        }else
        {
            cout << "Pixel out of image( " <<
                    bgrImg.rows << " , " << bgrImg.cols << " )!!! ( "
                 << pixelRow << " , " << pixelCol << ")" << endl;
        }
    }
}

void Segment::toBinMatrix(Mat &mBin)
{
    mBin = Mat(MRow-mRow+1 , MCol-mCol+1, CV_8UC1,Scalar(0));

    for(unsigned i = 0; i < N;i++)
    {
        mBin.at<uchar>( result.at<ushort>(0,i) -mRow,
                        result.at<ushort>(1,i) -mCol) = 255u;
    }
}

void Segment::toContour(Mat &mBin)
{
    mBin = Mat(N,1, CV_32SC2);

    for(unsigned i = 0; i < N;i++)
    {
        mBin.at<int>( i, 0) = result.at<ushort>(1,i);
        mBin.at<int>( i, 1) = result.at<ushort>(0,i);
    }
}

void Segment::toContour(vector<Point> &contour)
{
    contour.resize(N);
    for(unsigned i = 0; i < N;i++)
    {
        contour[i].x = result.at<ushort>(1,i);
        contour[i].y = result.at<ushort>(0,i);
    }
}

void Segment::toOrderedContours(vector<Mat> &contour)
{
    Mat bin;
    toBinMatrix(bin);
    findContours(bin,contour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,Point(mCol,mRow));
}

void Segment::toOrderedContours(vector<vector<Point> > &contour)
{
    Mat bin;
    toBinMatrix(bin);
    findContours(bin,contour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,Point(mCol,mRow));
}


/**
 * @brief  Draw box of segment on image BGR
 *
 * @param bgrImg
 * @param color
 */
void Segment::drawSegmentBox(Mat &bgrImg, const Scalar &color)
{
    rectangle(bgrImg,Rect(mCol, mRow, MCol - mCol, MRow-mRow),
              color,1,8,0);
}


/**
 * @brief Rotate all pixel degreeAng around cx, cy position
 *
 * @param degreeAng
 * @param cx
 * @param cy
 */
void Segment::rotateSegment(float degreeAng, float cx, float cy)
{
    float angRad = (degreeAng*M_PI)/180.f,
          cosA = cos(angRad),
          sinA = sin(angRad);

    for(unsigned i=0; i < N ; i++)
    {
        float ny = result.at<ushort>(0,i)-cy, // Acess pixel's row
              nx = result.at<ushort>(1,i)-cx; // Acess pixel's col

        result.at<ushort>(0,i) = // Access the row
                roundf(  // It's important! round the values!!
                         nx*sinA + ny*cosA + cy  // Row <-> y
                      );

        result.at<ushort>(1,i) = // Access the col
                roundf(  // It's important! round the values!!
                         nx*cosA - ny*sinA + cx // Col <-> x
                    );
    }
}


/**
 * @brief Copy a segment inside another
 *  and update atributes mRow MRow mCol MCol
 *
 * @param seg - Seg will be copied.
 */
void Segment::merge(Segment *seg)
{

    if(result.cols < N+seg->N)
    {
        Mat newMat(3,N+seg->N,CV_8UC1);

             result(Rect( 0 , 0 ,  N , 3 )).copyTo
            (newMat(Rect( 0 , 0 ,  N , 3 )));

        seg->result(Rect( 0 , 0 ,seg->N, 3 )).copyTo
            (newMat(Rect( N , 0 ,seg->N, 3 )));
        result = newMat;
    }else
    {
        seg->result(Rect( 0 , 0 ,seg->N, 3 )).copyTo
            (result(Rect( N , 0 ,seg->N, 3 )));
    }

    N +=seg->N;

    mRow = std::min(mRow,seg->mRow);
    MRow = std::max(MRow,seg->MRow);
    mCol = std::min(mCol,seg->mCol);
    MCol = std::max(MCol,seg->MCol);
}
