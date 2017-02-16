#include "GroundTruth.h"

int main( int argc, char** argv )
{
    GroundTruth gt;
//    gt.loadEmptyFrames("imgs.txt");
    gt.loadGroundTruth("../../GroundTruth/Yatcht_05_12_2014.txt");
    gt.start();
}

//int main( int argc, char** argv )
//{
//    const int dims = 1;
//    int size[1] = {5};
//    SparseMat sparse_mat(dims, size, CV_8U);
//    uchar *e;
//    for(int i = 0; i < 500000; i++)
//    {
//        int idx;
//        idx = rand()%2000;
//        e = (uchar*) sparse_mat.ptr(idx,true);
//  //      cout << idx << " -> " << (int) (*e) << endl;
//        (*e)++;
//    }
//}
