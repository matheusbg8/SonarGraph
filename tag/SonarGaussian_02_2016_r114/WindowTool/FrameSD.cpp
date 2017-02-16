#include "FrameSD.h"

FrameSD::FrameSD(const string &fileName, unsigned frameNumber):
    Frame(fileName,frameNumber),
    sd(0x0)
{
}

FrameSD::~FrameSD()
{

}
