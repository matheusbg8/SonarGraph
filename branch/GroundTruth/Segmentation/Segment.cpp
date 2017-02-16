#include "Segment.h"
#include <iostream>

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

void Segment::drawSegmentBox(Mat &bgrImg, const Scalar &color)
{
    rectangle(bgrImg,Rect(mCol, mRow, MCol - mCol, MRow-mRow),
              color,2,8,0);
}

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
