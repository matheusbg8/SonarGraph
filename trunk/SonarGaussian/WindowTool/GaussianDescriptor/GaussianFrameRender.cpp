#include "GaussianFrameRender.h"

#include <iostream>
using namespace std;

GaussianFrameRender::GaussianFrameRender()
{
}

void GaussianFrameRender::setGassians(vector<Gaussian> &gs)
{
    rgs.clear();
    for(unsigned i = 0 ; i < gs.size() ; i++)
    {
        rgs.push_back(GaussianRender(gs[i],i));
    }
}

GaussianRender &GaussianFrameRender::getGaussianRender(unsigned id)
{
    if(id >= rgs.size())
        cout << "GaussianRender::getGaussianRender Invalid ID!!!" << endl;

    return rgs[id];
}

void GaussianFrameRender::render(Mat &img)
{
    for(unsigned i = 0 ; i < rgs.size() ; i++)
    {
        if(rgs[i].isVisible())
            rgs[i].render(img);
    }
}
