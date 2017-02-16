#ifndef SVMFRAME_H
#define SVMFRAME_H

#include <vector>
#include "Sonar/Gaussian.h"

using namespace std;

class SVMObject
{
public:
    int label, objectId;
    vector<double> data;
    SVMObject(){ label = objectId = -1; }
};

class SVMFrame
{
public:
    SVMFrame();

    vector<SVMObject> vms;
    int frameId,info;

    void allocateObjecId(unsigned objId);

    void setData(int objId, vector<double> &data);
    void setLabel(int objId, int label);

    bool hasVm(unsigned vmId);

    bool hasData(unsigned objId);
    bool hasLabel(unsigned objId);

    SVMObject & getVm(unsigned objId);

    void clear();

};

#endif // SVMFRAME_H
