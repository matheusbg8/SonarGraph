#include "GenericImageProcessing.h"
#include "DirOperations.h"

GenericImageProcessing::GenericImageProcessing(const string &path):
    dataPath(path)
{

}

bool GenericImageProcessing::loadFrames()
{
    FILE *f_img_names = fopen((dataPath + "Frames_shortDataset.txt").c_str(),"r");
    char img_file_name[300];

    if(f_img_names == 0x0)
    {
        cout << "Image list file Frames.txt fileName not found!!" << endl;
        return false;
    }

    while( fscanf(f_img_names,"%s", img_file_name) != -1)
    {
        imgsName.push_back(img_file_name);
        cout << "Image " << img_file_name << " registred!" << endl;
    }

    fclose(f_img_names);
}

unsigned int GenericImageProcessing::computeImageDiff(Mat &img1, Mat &img2)
{
//    Mat dst;
//    absdiff(img1,img2,dst);
//    return sum( dst )[0];

//        result(img1.rows, img1.cols, CV_16UC1);

//        ushort max=0u, min=65000u;
    unsigned int diff=0;
    for(unsigned i = 0 ; i < img1.rows; i++)
    {
        for(unsigned j = 0 ; j < img1.rows; j++)
        {

            ushort &p1 = img1.at<ushort>(i,j),
                   &p2 = img2.at<ushort>(i,j),
//                       &r = result.at<ushort>(i,j);
                   r;

            if(p1> p2)
                r = p1-p2;
            else r = p2-p1;

//                if(r!= 0) diff ++;
            diff += r;

//                if(r > max) max = r;
//                if(r < min) min = r;
        }
    }

//        Mat result8Bit;
//        result.convertTo(result8Bit,CV_8UC1, 255.f/(max-min),-min*255.f/(max-min) );
//        applyColorMap(result8Bit,result8Bit,COLORMAP_JET);
//        imshow("subResult", result8Bit);
//        waitKey();

    return diff;
}

void GenericImageProcessing::generateAllImgDiff()
{
    Mat imgDiff(imgsName.size(), imgsName.size(),CV_32SC1);

    unsigned min=9999999999, max=0;
    for(unsigned i = 0; i < imgsName.size() ; i++)
    {
        cout << "Processing image " << i << " / " << imgsName.size() << endl;
        Mat img1 = imread(imgsName[i],CV_LOAD_IMAGE_ANYDEPTH);
        for(unsigned j = i+1; j < imgsName.size() ; j++)
        {
            Mat img2 = imread(imgsName[j],CV_LOAD_IMAGE_ANYDEPTH);
            int r = computeImageDiff(img1,img2);

//            cout << r << " , " << min << " , " << max << endl;

            if(r <0) cout << "Houston, we have a overflow problem!" << endl;

            imgDiff.at<int>(i,j) = r;
            imgDiff.at<int>(j,i) = r;

            if(r < min) min = r;
            if(r > max) max = r;
        }
    }

    Mat finalImg;
    imgDiff.convertTo(finalImg,CV_8UC1, 255.f/(max-min), -(float)min*(255.f/(max-min)));

    for(unsigned i = 0; i < imgsName.size() ; i++)
    {
        finalImg.at<uchar>(i,i) = 0;
    }
    imwrite("BitDiffImage.png", finalImg);

    applyColorMap(finalImg,finalImg,COLORMAP_JET);
    imwrite("BitDiffImage_color.png", finalImg);

    imshow("BitDiffImage_color", finalImg);
    cout << "max " << max << " , min " << min << endl;
    waitKey();

}
