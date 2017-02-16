#ifndef FRAMESD_H
#define FRAMESD_H

#include "WindowTool/Frame.h"
#include "Sonar/SonarDescritor.h"

/**
 * @brief This class is a Frame with
 * Sonar Description used by WFTpopologicalMatch
 * WindowTool.
 *
 */
class FrameSD : public Frame
{
public:
    FrameSD(const string& fileName,unsigned frameNumber);
    ~FrameSD();

    SonarDescritor *sd;

};

#endif // FRAMESD_H
