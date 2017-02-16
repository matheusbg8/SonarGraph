#include "SVMFrame.h"
#include <iostream>
using namespace std;

SVMFrame::SVMFrame():
    frameId(-1),info(-1)
{
}

void SVMFrame::allocateObjecId(unsigned objId)
{
    if(objId >= vms.size())
        vms.resize((objId+1)*2);
}

void SVMFrame::setData(int objId, vector<double> &data)
{
    allocateObjecId(objId);

    SVMObject &vm = vms[objId];
    vm.objectId = objId;
    vm.data = data;
}

void SVMFrame::setLabel(int objId, int label)
{
    allocateObjecId(objId);

    SVMObject &vm = vms[objId];
    vm.objectId = objId;
    vm.label = label;
}

bool SVMFrame::hasVm(unsigned vmId)
{
    if(vmId < vms.size())
        return true;
    return false;
}

bool SVMFrame::hasData(unsigned objId)
{
    if(objId < vms.size() && vms[objId].data.size() > 0)
        return true;
    return false;
}

bool SVMFrame::hasLabel(unsigned objId)
{
    if(objId < vms.size() && vms[objId].label != -1)
        return true;
    return false;
}

SVMObject & SVMFrame::getVm(unsigned objId)
{
    return vms[objId];
}

void SVMFrame::clear()
{
    vms.clear();
}
