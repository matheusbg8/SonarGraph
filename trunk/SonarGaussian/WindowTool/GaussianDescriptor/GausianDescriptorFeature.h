#ifndef GAUSIANDESCRIPTORFEATURE_H
#define GAUSIANDESCRIPTORFEATURE_H

#include "Sonar/Gaussian.h"

class WFGaussianDescriptor;

class GausianDescriptorFeature
{
//protected:
public:
    WFGaussianDescriptor *_WFGD;

public:
    GausianDescriptorFeature();
    void setWFGaussianDescriptor(WFGaussianDescriptor *WFGD);

    virtual void newGaussian(int gId, int frameId);

    virtual void leftGaussianSelected(int gId, int frameId);
    virtual void rightGaussianSeleced(int gId, int frameId);

    virtual void cleanedGaussians(int frameId);

};

#endif // GAUSIANDESCRIPTORFEATURE_H
