#include "WindowTool/Frame.h"
#include <iostream>
using namespace std;

Frame::Frame(const string& fileName,unsigned frameNumber):
    fileName(fileName),
    frameNumber(frameNumber)
{
}

Frame::~Frame()
{

}



ostream &operator <<(ostream &os, const Frame &fr)
{
    os << "Frame " << ((int) fr.frameNumber)
       << " name " << fr.fileName
       << std::endl;
    return os;
}
