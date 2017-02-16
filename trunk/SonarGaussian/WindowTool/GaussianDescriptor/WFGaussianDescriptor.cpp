#include "WFGaussianDescriptor.h"
#include "Drawing/Drawing.h"

WFGaussianDescriptor::WFGaussianDescriptor(ConfigLoader &config):
    _gdf(0x0),
    hightlightLeftGaussian(-1),
    hightlightRightGaussian(-1)
{
    removeGaussianSelections();
    loadConfigFile(config);    
}

void WFGaussianDescriptor::loadConfigFile(ConfigLoader &config)
{
    sonar.loadConfig(config);

}

int WFGaussianDescriptor::selectGaussianOnFrame(int x, int y, int frameID, float precision)
{
    int gaussianID = -1;
    float minDist=FLT_MAX;
    vector<Gaussian> &gaussians = frames[frameID].gaussians;

    for(unsigned g = 0 ; g < gaussians.size(); g++)
    {
        float dx = gaussians[g].x - x,
              dy = gaussians[g].y - y,
              dist = sqrt(dx*dx + dy*dy);

        if(minDist > dist )
        {
            gaussianID = g;
            minDist = dist;
        }
    }

    if(minDist > precision)
    {
        gaussianID = -1;
    }

    return gaussianID;
}

void WFGaussianDescriptor::removeGaussianSelections()
{
    leftSelecGaussian = rightSelecGaussian = -1;
}

void WFGaussianDescriptor::setGDF(GausianDescriptorFeature *gdf)
{
    _gdf = gdf;
    _gdf->_WFGD = this;
}

void WFGaussianDescriptor::_processImage(Mat &img16Bits, int frameId)
{
    // Risize gaussian frames
    if(frameId >= frames.size())
    {
        frames.resize((frameId+1)*2);
    }

    // Take a reference to gaussians of the frame
    vector<Gaussian> &gs = frames[frameId].gaussians;

    // If this frame is shown in screen
    if(wt->isCurrentLeftFrame(frameId)) // than remove selections
    {
        leftSelecGaussian=hightlightLeftGaussian=-1;
    }else if(wt->isCurrentRightFrame(frameId))
    {
        rightSelecGaussian=hightlightRightGaussian=-1;
    }

    // If there are no gaussians
    if(gs.size() == 0)
    {   // Create them
        sonar.createGaussian(img16Bits,gs,false);
    }
}

void WFGaussianDescriptor::start()
{

}

void WFGaussianDescriptor::keyPress(char c)
{
    switch(c)
    {
    case 's':
        save("test.txt");
    break;
    case 'l':
        load("test.txt");
    break;
    case 'u': // Start segmentation calib gui
        if(wt->isFrameSelected())
        {
            frames[wt->selectedFrameId()].clear();
            _gdf->cleanedGaussians(wt->selectedFrameId());

            sonar.segmentationCalibUI(wt->selectedFrameImg());
            _processImage(wt->selectedFrameImg(),wt->selectedFrameId());
        }
    break;
    }
}

void WFGaussianDescriptor::mouseEvent(int event, int x, int y)
{
    if(event == CV_EVENT_LBUTTONUP)
    {
        if(x < wt->windowSize.width*0.5f) // Left Frame
        {
            x = (x-wt->el.val[2])/wt->el.val[0];
            y = (y-wt->el.val[3])/wt->el.val[1];

            leftSelecGaussian = selectGaussianOnFrame(x,y,wt->currentLeftFrame);

            if(_gdf != 0x0) _gdf->leftGaussianSelected(leftSelecGaussian,wt->currentLeftFrame);

            cout << "Left transform:"
                 << " dx " << wt->el.val[2]
                 << " dy " << wt->el.val[3]
                 << " ex " << wt->el.val[0]
                 << " ey " << wt->el.val[1]
                 << " mx " << x
                 << " my " << y
                 << " left gaussian " << leftSelecGaussian
                 << endl;


        }else // Right Frame
        {
            x = (x-wt->er.val[2])/wt->er.val[0];
            y = (y-wt->er.val[3])/wt->er.val[1];

            rightSelecGaussian = selectGaussianOnFrame(x,y,wt->currentRightFrame);

            if(_gdf != 0x0) _gdf->rightGaussianSeleced(rightSelecGaussian,wt->currentRightFrame);

            cout << "Right transform:"
                 << " dx " << wt->er.val[2]
                 << " dy " << wt->er.val[3]
                 << " ex " << wt->er.val[0]
                 << " ey " << wt->er.val[1]
                 << " mx " << x
                 << " my " << y
                 << " right gaussian " << rightSelecGaussian
                 << endl;
        }
    }
}

void WFGaussianDescriptor::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    vector<Gaussian> &lgs = frames[leftId].gaussians,
                     &rgs = frames[rightId].gaussians;

    Drawing::drawGaussians(leftImg,lgs,Scalar(.0,.0,255.0),Scalar(0.0,255.0,0.0),true,false,false);
    Drawing::drawGaussians(rightImg,rgs,Scalar(.0,.0,255.0),Scalar(0.0,255.0,0.0),true,false,false);


    if(hightlightLeftGaussian >= 0)
    {
        Drawing::drawGaussian(leftImg,lgs[hightlightLeftGaussian],
                              Scalar(255,0,0),Scalar(0,0,255),hightlightLeftGaussian,true,false);
    }
    if(hightlightRightGaussian >= 0)
    {
        Drawing::drawGaussian(rightImg,rgs[hightlightRightGaussian],
                              Scalar(255,0,0),Scalar(0,0,255),hightlightRightGaussian,true,false);
    }


    if(leftSelecGaussian >= 0)
    {
        Drawing::drawGaussian(leftImg,lgs[leftSelecGaussian],
                              Scalar(0,255,0),Scalar(0,0,255),leftSelecGaussian,true,false);
    }

    if(rightSelecGaussian >= 0)
    {
        Drawing::drawGaussian(rightImg,rgs[rightSelecGaussian],
                              Scalar(0,255,0),Scalar(0,0,255),rightSelecGaussian,true,false);
    }

}

void WFGaussianDescriptor::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    _processImage(leftImg,leftId);
    _processImage(rightImg,rightId);
}

bool WFGaussianDescriptor::save(const char *fileName)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(fileName, "w");
    if(f == 0x0)
    {
        cout << "File " << fileName << " not found" << endl;
        return false;
    }

    unsigned frameId=0, g, u, i, nMatch;

    fprintf(f,"%u\n", frames.size());
    for(frameId = 0 ; frameId < frames.size() ; frameId++)
    {
        GaussianFrame &fr = frames[frameId];

        // Frame description
        // frameID NumberOfGaussians
        fprintf(f,"%u, %u\n", frameId, fr.gaussians.size());

        // Gaussians Description
        vector<Gaussian> &gs = fr.gaussians;
        if(gs.size() == 0) continue;

        for(g = 0 ; g < gs.size(); g++)
        {
            Gaussian &ga = gs[g];
            // ID X, Y, Z , DX, DY, DZ, Ang, N
            fprintf(f,"%d,%g,%g,%g,%g,%g,%g,%g,%u,",
                    g, ga.x, ga.y , ga.intensity,
                    ga.dx  , ga.dy, ga.di,
                    ga.ang , ga.N );

            // Shape information
            // Premiter, area, convexHUll area - area
            fprintf(f,"%lg,%lg,%lg,",
                    ga.perimeter,
                    ga.area, ga.convexHullArea);

            // Hu moments
            fprintf(f,"%lg,%lg,%lg,%lg,%lg,%lg,%lg\n",
                    ga.hu[0],ga.hu[1],ga.hu[2],ga.hu[3],
                    ga.hu[4],ga.hu[5],ga.hu[6]);
        }
    }
    fclose(f);

    return true;
}

bool WFGaussianDescriptor::load(const char *fileName)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
    {
        cout << "File " << fileName << " not found" << endl;
        return false;
    }

    unsigned frameId, gId,frameCount,gaussiansCount;

    // Number of Frames on this GroundTrutuh
    fscanf(f,"%u", &frameCount);
    frames.resize(frameCount);
    for(frameId = 0 ; frameId < frames.size() ; frameId++)
    {
        GaussianFrame &fr = frames[frameId];

        // Frame description
        // frameID NumberOfGaussians
        fscanf(f,"%*u, %u",&gaussiansCount);

        if(gaussiansCount == 0) continue;

        // Gaussians Description
        vector<Gaussian> &gs = fr.gaussians;
        gs.resize(gaussiansCount);
        for(gId = 0 ; gId < gaussiansCount; gId++)
        {
            Gaussian &ga = gs[gId];
            // ID X, Y, Z , DX, DY, DZ, Ang, N
            fscanf(f,"%*d,%g,%g,%g,%g,%g,%g,%g,%u,",
                    &ga.x, &ga.y , &ga.intensity,
                    &ga.dx , &ga.dy, &ga.di,
                    &ga.ang , &ga.N );

            // Shape information
            // Premiter, area, convexHUll area - area
            fscanf(f,"%lg,%lg,%lg,",
                    &ga.perimeter,
                    &ga.area, &ga.convexHullArea);

            // Hu moments
            fscanf(f,"%lg,%lg,%lg,%lg,%lg,%lg,%lg\n",
                   &ga.hu[0],&ga.hu[1],&ga.hu[2],&ga.hu[3],
                   &ga.hu[4],&ga.hu[5],&ga.hu[6]);
        }
    }
    fclose(f);
    return true;
}

Gaussian &WFGaussianDescriptor::getGaussian(unsigned frameId, unsigned gaussianId)
{
    if(frameId < frames.size())
    {
        vector<Gaussian> &gs = frames[frameId].gaussians;
        if(gaussianId < gs.size())
        {
            return gs[gaussianId];
        }
        else
        {
            cout << "WFGaussianDescriptor: getGaussian gaussian Id problem" << endl;
            return gs[gaussianId];
        }
    }
    cout << "WFGaussianDescriptor: getGaussian frame Id problem" << endl;
    return frames[frameId].gaussians[gaussianId];
}

int WFGaussianDescriptor::getLeftSelectedGaussian()
{
    return leftSelecGaussian;
}

int WFGaussianDescriptor::getRighSelectedGaussian()
{
    return rightSelecGaussian;
}
