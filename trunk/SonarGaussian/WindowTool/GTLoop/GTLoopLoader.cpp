#include "GTLoopLoader.h"
#include <cstdio>
#include <iostream>
#include <cmath>

#include "CSVReader/CSVReader2.h"

using namespace std;

GTLoopLoader::GTLoopLoader():
    sonarRange(30.f)
{
}

bool GTLoopLoader::loadPositionFromCSV(const string &positionCSV)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(positionCSV.c_str(),"r");
    if(f == 0x0)
    {
        cout << "GTLoopLoader:: TF position file "
             << positionCSV
             << " can't be loaded"
             << endl;
        return false;
    }

    csvPosition.push_back(CSVTFPosition());
    CSVTFPosition *ref = &(csvPosition.back());

    //          timeStamp, x, y, z, rot
    while(fscanf(f,"%lg,%g,%g,%g,%g",
                  &ref->timeStamp,&ref->x, &ref->y, &ref->z,
                  &ref->rot) != -1)
    {
        csvPosition.push_back(CSVTFPosition());
        ref = &(csvPosition.back());
    }
    totalPathLenght=0.f;
    csvPosition.resize(csvPosition.size()-1);
    for(unsigned i = 1 ; i < csvPosition.size();i++)
    {
        float dx = csvPosition[i].x - csvPosition[i-1].x,
              dy = csvPosition[i].y - csvPosition[i-1].y;
        totalPathLenght += sqrt(dx*dx+dy*dy);
    }

    fclose(f);

//    cout.precision(15);
//    for(unsigned i = 0; i < csvPosition.size(); i++)
//    {
//        ref = &(csvPosition[i]);
//        cout << ref->timeStamp << " , "
//             << ref->x << " , "
//             << ref->y << " , "
//             << ref->z << " , "
//             << ref->rot << endl;
//    }

}

bool GTLoopLoader::loadFramesFromCSV(const string &frameInfo)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(frameInfo.c_str(),"r");
    if(f == 0x0)
    {
        cout << "GTLoopLoader:: frame information file "
             << frameInfo
             << " can't be loaded"
             << endl;
        return false;
    }

    csvFrames.push_back(CSVFrame());
    CSVFrame *ref = &(csvFrames.back());

    //          id, timeStamp
    while(fscanf(f,"%u,%lg",
                  &ref->id, &ref->timeStamp) != -1)
    {

        csvFrames.push_back(CSVFrame());
        ref = &(csvFrames.back());
    }

    fclose(f);

//    cout.precision(15);
//    for(unsigned i = 0; i < csvFrames.size(); i++)
//    {
//        ref = &(csvFrames[i]);
//        cout << ref->id << " , "
//             << ref->timeStamp << endl;
//    }
}

bool GTLoopLoader::loadHeadingFromCSV(const string &headingCSV)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(headingCSV.c_str(),"r");
    if(f == 0x0)
    {
        cout << "GTLoopLoader:: frame information file "
             << headingCSV
             << " can't be loaded"
             << endl;
        return false;
    }

    csvHeading.push_back(CSVHeading());
    CSVHeading *ref = &(csvHeading.back());

    //          id, timeStamp
    while(fscanf(f,"%lg,%g",
                  &ref->timeStamp, &ref->heading) != -1)
    {

        csvHeading.push_back(CSVHeading());
        ref = &(csvHeading.back());
    }

    fclose(f);
//    cout.precision(15);
//    for(unsigned i = 0; i < csvHeading.size(); i++)
//    {
//        ref = &(csvHeading[i]);
//        cout << ref->timeStamp << " , "
//             << ref->heading << endl;
    //    }
}

bool GTLoopLoader::loadFrameDescriptionFromCSV(const string &frameDescriptionCSV)
{
    CSVReader2 csv;

    if(!csv.open(frameDescriptionCSV.c_str(),3,',', CSV_INT, CSV_INT, CSV_INT))
    {
        return false;
    }

    csv.ignoreLine();

    CSVFrameDescriptionInfo current;
    while(csv.read(&current.frameId,&current.vertexCount,&current.edgeCount))
    {
        csvFrameDescription.push_back(current);
    }
}


/**
 * @brief
 *      Compute GTFLoop frames with position and heading.
 * @param prefix - File name prefix
 * @param frames - Frames that will bee filled
 */
void GTLoopLoader::syncFramesData(const string &prefix, vector<GTLoopFrame> &frames)
{
    unsigned headingId=0, positionId=0, framesId=0;
    double currentTime, nextTime, frameTime;

    frames.reserve(csvFrames.size()+1);

    // Sort all msgs by time

    while(framesId < csvFrames.size())
    {
        // Take next frame
        CSVFrame &fr = csvFrames[framesId++];

        frameTime = fr.timeStamp;

        // Find closest Heading by timestamp
        currentTime = csvHeading[headingId].timeStamp;
        // While has next msgs
        while(headingId+1 < csvHeading.size())
        {
            nextTime = csvHeading[headingId+1].timeStamp;

            if(abs(nextTime - frameTime) < abs(currentTime - frameTime))
            {
                headingId++;
                currentTime = csvHeading[headingId].timeStamp;
            }else
                break; // current headingId is the closest
        }

        CSVHeading &heading = csvHeading[headingId];

        // Find closest Position by timestamp
        currentTime = csvPosition[positionId].timeStamp;
        // While has next msgs
        while(positionId+1 < csvPosition.size())
        {
            nextTime = csvPosition[positionId+1].timeStamp;
            if(abs(nextTime - frameTime) < abs(currentTime - frameTime))
            {
                positionId++;
                currentTime = csvPosition[positionId].timeStamp;
            }else
                break; // current positionId is the closest
        }
        CSVTFPosition &position = csvPosition[positionId];

        char str[200];
        sprintf(str, "%s%05d.png",prefix.c_str(),framesId);

        double radAng = heading.heading*M_PI/180.0;
        Point2f p(position.x, position.y),
                direction( sin(radAng),cos(radAng) );

        frames.push_back(GTLoopFrame(str,framesId,frameTime,
                                     p,
                                     p + direction*sonarRange*0.5,
                                     heading.heading));

    }

//    cout << "Frames:" << endl;
//    for(unsigned i = 0 ; i < frames.size(); i++)
//    {
//        cout << frames[i].fileName << ", "
//             << frames[i].p << ", "
//             << frames[i].c << ", "
//             << frames[i].heading << endl;
//    }
}
