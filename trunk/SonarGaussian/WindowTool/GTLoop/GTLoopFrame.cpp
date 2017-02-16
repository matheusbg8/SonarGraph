#include "GTLoopFrame.h"

GTLoopFrame::GTLoopFrame(const string &fileName, unsigned frameNumber, double timeStamp,
                         const Point2f &p, const Point2f &c, float heading):
    Frame(fileName,frameNumber),
    p(p),c(c),heading(heading)
{

}
