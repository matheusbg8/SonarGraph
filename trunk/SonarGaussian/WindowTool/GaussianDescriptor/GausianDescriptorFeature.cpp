#include "GausianDescriptorFeature.h"

GausianDescriptorFeature::GausianDescriptorFeature():
    _WFGD(0x0)
{
}

void GausianDescriptorFeature::setWFGaussianDescriptor(WFGaussianDescriptor *WFGD)
{
    _WFGD = WFGD;
}

void GausianDescriptorFeature::newGaussian(int gId, int frameId)
{

}

void GausianDescriptorFeature::leftGaussianSelected(int gId, int frameId)
{

}

void GausianDescriptorFeature::rightGaussianSeleced(int gId, int frameId)
{

}

void GausianDescriptorFeature::cleanedGaussians(int frameId)
{

}
