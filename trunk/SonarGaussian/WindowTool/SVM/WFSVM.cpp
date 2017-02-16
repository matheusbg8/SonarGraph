#include "WFSVM.h"

#include "WindowTool/GaussianDescriptor/WFGaussianDescriptor.h"
#include "Drawing/Drawing.h"


// http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
// http://opencvexamples.blogspot.com/2013/09/find-contour.html


WFSVM::WFSVM():
    gamma(41.55),
    C(28.45),
//    traningDataPercent(0.65f),
    traningDataPercent(1.f),
    kernel_type(CvSVM::RBF),
    normalizeData(true),autoTrain(false),trainingDataReady(false),
    lastSelecFrameId(-1),
    lastSelectedObjectId(-1),
    showFrameInfo(true),
    svmTrained(false),
    trainResultWindowSize(600,600),
    workFileName("SVM_dataset")
{

}

Scalar WFSVM::getClassColor(float label)
{
    if( (label - (int) label) != 0)
    {
        cout << "Asked class " << label << endl;
    }

    if(label >= 0.5 && label < 1.5)
    {
        return Scalar(0.f,255.f,0.f);  // Green Pole
    }else if(label >= 1.5 && label < 2.5)
    {
        return Scalar(0.f,69.f,255.f); // Orange Wharf
    }else if(label >= 2.5 && label < 3.5)
    {
        return Scalar(0.f,0.f,255.f); // Red BoatHull

    }else if(label >= 3.5 && label < 4.5)
    {
        return Scalar(255.f,255.f,0.f); // Cyan Stone
    }else if(label >= 4.5 && label < 5.5)
    {
        return Scalar(0.f,255.f,255.f); // Yellow Fish
    }else if(label >= 5.5 && label < 6.5)
    {
        return Scalar(255.f,0.f,0.f); // Blue Swimmer
    }
}



void WFSVM::doTraining()
{
    // Set up training data
    Mat trainingLabelsMat, trainingDataMat;
    Mat validationLabelsMat, validationDataMat;

    makeTraningData(trainingLabelsMat,trainingDataMat,
                    validationLabelsMat,validationDataMat);

//    cout << "Labels: " << trainingLabelsMat << endl;
//    cout << "Data: " << trainingDataMat << endl;

    cout << "MinVal - MaxVal:" << endl;
    // For each class take min and max value
    for(unsigned i = 0 ; i < minVal.size() ; i++)
    {
        cout << minVal[i] << " - " << maxVal[i] << endl;
    }

    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = kernel_type;
    params.gamma = gamma;
    params.C = C;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    if(autoTrain)
    {
//        SVM.train_auto(dataMat, labelsMat, Mat(), Mat(), params,3);
        int k = 10;
        SVM.train_auto(trainingDataMat, trainingLabelsMat, Mat(), Mat(), params,k,
                       CvParamGrid(0.1,60.0,1.1),  // C parameters
                       CvParamGrid(0.1,60.0,1.1)  // Gamma parameters
                       );
//    train_auto(data, labels, Mat(), Mat(), params, int k_fold=10, CvParamGrid Cgrid=CvSVM::get_default_grid(CvSVM::C), CvParamGrid gammaGrid=CvSVM::get_default_grid(CvSVM::GAMMA), CvParamGrid pGrid=CvSVM::get_default_grid(CvSVM::P), CvParamGrid nuGrid=CvSVM::get_default_grid(CvSVM::NU), CvParamGrid coeffGrid=CvSVM::get_default_grid(CvSVM::COEF), CvParamGrid degreeGrid=CvSVM::get_default_grid(CvSVM::DEGREE), bool balanced=false)

        CvSVMParams resParams = SVM.get_params();
//        C = resParams.C;
//        gamma = resParams.gamma;
        cout << "AutoTrain:" << endl
             << "k = " << k << endl
             << "Gamma = " << resParams.gamma << endl
             << "C = " << resParams.C << endl;
    }else
    {
        SVM.train(trainingDataMat, trainingLabelsMat, Mat(), Mat(), params);
    }

    svmTrained = true;

    showTrainResults(SVM);

    vector<pair<unsigned , unsigned> > results;

    cout << "\nHit statistics:" << endl;

    cout << "Validation " << computeHitPercentage(validationLabelsMat,validationDataMat,results)
         << endl;
    for(unsigned i = 0; i < results.size(); i++)
    {
        if(allSVMByClass[i].size()>0)
        {
            cout << classesName[i]
                 << " " << (float) results[i].first/ ((float)results[i].first +  results[i].second)
                 << "%" << endl;
        }
    }
    cout << "\nAll data "
         << computeHitPercentage(results)
         << "%" << endl;
    for(unsigned i = 0; i < results.size(); i++)
    {
        if(allSVMByClass[i].size()>0)
        {
            cout << classesName[i]
                 << " " << (float) results[i].first/ ((float)results[i].first +  results[i].second)
                 << "% of hit" << endl;
        }
    }
}


void WFSVM::showTrainResults(CvSVM &SVM)
{
    if(minVal.size() > 2) return;

    Mat imResult(trainResultWindowSize.height, trainResultWindowSize.width,CV_8UC3,Scalar(0,0,0));

    for(unsigned i = 0; i < trainResultWindowSize.height;i++)
    {
        for(unsigned j = 0 ; j < trainResultWindowSize.width ; j++)
        {
            Mat sampleMat(1,minVal.size(),CV_32FC1,Scalar(0));

            if(normalizeData)
            {
                sampleMat.at<float>(0,0) = (float) j/ (float) (trainResultWindowSize.width);
                sampleMat.at<float>(0,1) = (float) i/ (float) (trainResultWindowSize.height);
            }

            for(unsigned k = 2; k < minVal.size();k++)
            {
                sampleMat.at<float>(0,k) = 0.5f;
            }

            float response = SVM.predict(sampleMat);

            Scalar color = getClassColor(response);
            imResult.at<Vec3b>(i,j)  = Vec3b(color.val[0],color.val[1],color.val[2]);
        }
    }

    // Draw VMs
    for(unsigned frameId = 0; frameId < frames.size() ;frameId++)
    {
        SVMFrame &fr = frames[frameId];
        vector<SVMObject> &vms = fr.vms;

        for(unsigned vmId = 0 ; vmId < vms.size() ; vmId++)
        {
            if(fr.hasData(vmId))
            {
                SVMObject &vm = vms[vmId];

                // Normalized center
                Point2f center( (vm.data[0]-minVal[0])/(maxVal[0]-minVal[0]) ,
                                (vm.data[1]-minVal[1])/(maxVal[1]-minVal[1]) );

                // Center on screen
                Point2f screenCenter(center.x*trainResultWindowSize.width,
                                     center.y*trainResultWindowSize.height);

                if(fr.hasLabel(vmId))
                {
                    circle(imResult,screenCenter, 5,
                           getClassColor(vm.label),-1);
                    // Draw black circle contour
                    circle(imResult,screenCenter, 5,
                           Scalar(0.f,0.f,0.f),1);
                }else
                {
                    // Unknow label is black circle
//                    circle(imResult,screenCenter, 5,
//                           Scalar(255.f,0.f,255.f),-1);
                }
            }
        }
    }

    imshow("Train Results", imResult);
}

void WFSVM::takeMinVal(vector<double> &min, vector<double> &data)
{
    if(data.size() > min.size())
        min.resize(data.size(), 999999.9);

    for(unsigned i = 0 ; i < data.size();i++)
        min[i] = std::min(min[i],data[i]);
}

void WFSVM::takeMaxVal(vector<double> &max, vector<double> &data)
{
    if(data.size() > max.size())
        max.resize(data.size(),-999999.9);
    for(unsigned i = 0 ; i < data.size();i++)
        max[i] = std::max(max[i],data[i]);
}

void WFSVM::makeTraningData(Mat &traningLabels, Mat &trainingData,
                            Mat &validationLabels, Mat &validationData)
{
    vector<GaussianFrame> &gFrs = _WFGD->frames;
    unsigned vmCount=0, labeledFrames=0;
    vector<double> data;
    bool frameHasLabel=false;

    // Reset min max values used to normalize after
    minVal.clear(); maxVal.clear();
    // Update min max value of every dimension and compute the
    // amount of vector with label (only vector with label are included
    // in min max computation)

    allSVMByClass.clear();

    // Computing support vector machine vectors
    for(unsigned frameId = 0 ; frameId < frames.size(); frameId++)
    {
        vector<Gaussian> &gs = gFrs[frameId].gaussians;
        SVMFrame &SVMfr = frames[frameId];

        frameHasLabel=false;

        for(unsigned i = 0 ; i < gs.size(); i++)
        {
            computeVector(data,gs[i]);
            SVMfr.setData(i,data);

//            takeMaxVal(maxVal,data);
//            takeMinVal(minVal,data);

            if(SVMfr.hasLabel(i))
            {
                vmCount++;
                frameHasLabel=true;
                takeMaxVal(maxVal,data);
                takeMinVal(minVal,data);

                SVMObject &svnObj = SVMfr.getVm(i);

                if(svnObj.label >= allSVMByClass.size())
                {
                    allSVMByClass.resize((svnObj.label+1)*2);
                }
                allSVMByClass[svnObj.label].push_back( svnObj );
            }
        }

        if(frameHasLabel)
            labeledFrames++;
    }

    cout << "Training information:" << endl
         << "* Vector dimension = " << minVal.size() << endl
         << "* Amount of labeled VMs = " << vmCount << endl
         << "* Amount of labeled frames = " << labeledFrames << endl;

    vector<int> trainingClassSize(allSVMByClass.size(),0);
    int trainingSize = 0,
        validationSize = 0;

    // Counting data
    for(unsigned i = 0 ; i < allSVMByClass.size(); i++)
    {
        if(allSVMByClass[i].size() == 0) continue;
        trainingSize+= trainingClassSize[i] = allSVMByClass[i].size()*traningDataPercent;
        validationSize+= allSVMByClass[i].size()-trainingClassSize[i];
    }

    traningLabels = Mat(trainingSize, 1, CV_32FC1, Scalar(0.f));
    trainingData = Mat(trainingSize, minVal.size() , CV_32FC1, Scalar(0.f));

    validationLabels = Mat(validationSize, 1, CV_32FC1, Scalar(0.f));
    validationData = Mat(validationSize, minVal.size() , CV_32FC1, Scalar(0.f));

    int trainingCount = 0, validationCount = 0;

    // Making training and validation vectors
    for(unsigned labelId = 0 ; labelId < allSVMByClass.size(); labelId++)
    {
        for(unsigned svmId = 0 ; svmId < allSVMByClass[labelId].size(); svmId++)
        {
            SVMObject &vm = allSVMByClass[labelId][svmId];

            if(svmId < trainingClassSize[labelId])
            {
                traningLabels.at<float>(trainingCount,0) = vm.label;
                vector<double> &vmData = vm.data;
                for(unsigned k = 0 ; k < vmData.size() ; k++)
                {
                    trainingData.at<float>(trainingCount,k) = (float) ((vmData[k] - minVal[k]) / (maxVal[k]-minVal[k]));
                }
                trainingCount++;
            }else
            {
                validationLabels.at<float>(validationCount,0) = vm.label;
                vector<double> &vmData = vm.data;
                for(unsigned k = 0 ; k < vmData.size() ; k++)
                {
                    validationData.at<float>(validationCount,k) = (float) ((vmData[k] - minVal[k]) / (maxVal[k]-minVal[k]));
                }
                validationCount++;
            }
        }
    }

    cout << "Class count:" << endl;
    for(unsigned i =0 ; i < allSVMByClass.size() ; i++)
    {
        if(allSVMByClass[i].size() > 0)
            cout << classesName[i] << " = " << allSVMByClass[i].size()
                 << " T " << (int)(allSVMByClass[i].size()*traningDataPercent)
                 << " V " << (allSVMByClass[i].size() - (int)(allSVMByClass[i].size()*traningDataPercent))
                 <<endl;
    }
    trainingDataReady = true;
}

double WFSVM::computeHitPercentage(Mat &labels, Mat &data, vector<pair<unsigned, unsigned> > &results)
{
    if(!svmTrained || labels.rows == 0) return 0.0;

    unsigned hit =0,wrong=0;
    results.clear();
    results.resize(allSVMByClass.size(),pair<unsigned, unsigned>(0,0));

    Mat resultsMat;
    SVM.predict(data, resultsMat);

    for(unsigned i = 0; i < data.rows; i++)
    {
        if(std::fabs(resultsMat.at<float>(i,0) - labels.at<float>(i,0)) < 0.5f ) // Correct class?
        {
            results[(int)labels.at<float>(i,0)].first++;
            hit++;
        }else
        {
            results[(int)labels.at<float>(i,0)].second++;
            wrong++;
        }
    }
    return hit / (double) (hit+wrong);
}

double WFSVM::computeHitPercentage(vector<pair<unsigned, unsigned> > &results)
{
    if(!svmTrained) return 0.0;

    unsigned hit =0,wrong=0;
    results.clear();
    results.resize(allSVMByClass.size(),pair<unsigned, unsigned>(0,0));

    // Draw VMs
    for(unsigned frameId = 0; frameId < frames.size() ;frameId++)
    {
        SVMFrame &fr = frames[frameId];
        vector<SVMObject> &vms = fr.vms;

        for(unsigned vmId = 0 ; vmId < vms.size() ; vmId++)
        {
            if(fr.hasData(vmId) && fr.hasLabel(vmId))
            {
                SVMObject &vm = vms[vmId];
                vector<double> &vmData = vm.data;
                Mat sampleMat(1, vmData.size(), CV_32FC1, Scalar(0.f));

                for(unsigned i = 0 ; i < vmData.size() ; i++)
                    sampleMat.at<float>(0,i) = (float) (vmData[i]-minVal[i])/(maxVal[i]-minVal[i]);

                float response = SVM.predict(sampleMat);

                if(std::fabs(response - vm.label) < 0.5f ) // Correct class?
                {
                    results[vm.label].first++;
                    hit++;
                }else
                {
                    results[vm.label].second++;
                    wrong++;
                }
            }
        }
    }

    return hit / (double) (hit+wrong);
}

void WFSVM::computeVector(vector<double> &v, Gaussian &g)
{
    v.clear();

    // Only axes
//    v.resize(2);
//    v[0] = g.dx;
//    v[1] = g.dy;

    // We trusth
//    v.resize(4);
//    v[0] = g.dx;  // Width
//    v[1] = g.dy;  // Height
//    v[2] = g.dx / g.dy; // Inertia Ratio - circularty (circle = 1 line = 0)
//    v[3] = g.N; // Inertia Ratio - circularty (circle = 1 line = 0)

    // Hu moments
//    v.resize(7);
//    v[0] = g.hu[0];
//    v[1] = g.hu[1];
//    v[2] = g.hu[2];
//    v[3] = g.hu[3];
//    v[4] = g.hu[4];
//    v[5] = g.hu[5];
//    v[6] = g.hu[6];

    // Areas
//    v.resize(2);
//    v[0] = g.area / g.convexHullArea; //Convexity
//    v[1] = g.dx / g.dy; // Inertia Ratio - circularty (circle = 1 line = 0)

    // Areas
//    v.resize(2);
//    v[0] = g.area / g.convexHullArea; //Convexity
//    v[1] = g.dx / g.dy; // Inertia Ratio - circularty (circle = 1 line = 0)

    // Everything good
    v.resize(10);
    v[0] = g.dx;  // Width
    v[1] = g.dy;  // Height
    v[2] = g.dx / g.dy; // Inertia Ratio - circularty (circle = 1 line = 0)
    v[3] = g.di; // Std Intensity
    v[4] = g.intensity; // Mean Intensity
    v[5] = g.area;  // Area
    v[6] = g.convexHullArea;  // Hull Area
    v[7] = g.area / g.convexHullArea; //Convexity
    v[8] = g.perimeter;  // Perimeter
    v[9] = g.N;  // Pixel Count

//    v.resize(2);
//    v[0] = g.intensity;
//    v[1] = g.di;

}


void WFSVM::renderFrame(Mat &img, int frameId)
{
    if(frameId >= frames.size()) return;

    SVMFrame &fr = frames[frameId];
    vector<SVMObject> &vms = fr.vms;
    vector<Gaussian> &gs = _WFGD->frames[frameId].gaussians;

    Point2f gtLabelPos(20.f,-10.f), // Ground Truth label position
            svmLabelPos(20.f,-25.f); // SVM label position

    unsigned minIdRange = std::min(gs.size(),vms.size());
    for(unsigned gId = 0 ; gId < minIdRange; gId++)
    {
        SVMObject &vm = fr.getVm(gId);

        // If Vm has an label
        if(fr.hasLabel(gId))
        {
            Drawing::drawGaussianText(img,gs[gId],Scalar(0.f,255.f,255.f),
                                      classesName[vm.label],gtLabelPos);
        }

        if(svmTrained && fr.hasData(gId))
        {
            vector<double> &vmData = vm.data;
            Mat sampleMat(1, vmData.size(), CV_32FC1, Scalar(0.f));

            for(unsigned i = 0 ; i < vmData.size() ; i++)
                sampleMat.at<float>(0,i) = (float) (vmData[i]-minVal[i])/(maxVal[i]-minVal[i]);

            float response = SVM.predict(sampleMat);

            Scalar color;
            if(!fr.hasLabel(gId))
            {
                color = Scalar(255.f,0.f,255.f); // Magenta (unknow)
            }
            else if(std::fabs(response - vm.label) < 0.5f ) // Correct class?
            {
                color = Scalar(0.f,255.f,0.f); // Green (correct)
            }else
            {
                color = Scalar(0.f,0.f,255.f); // Red (wrong)
            }

            int clasId = (int) round(response);
            Drawing::drawGaussianText(img,gs[gId],color,
                                      classesName[clasId],svmLabelPos);
        }
    }
}

void WFSVM::_renderFrameTogether(Mat &screen, const Scalar_<float> &e, unsigned frameId)
{
    if(showFrameInfo)
    {
        Point2f txtPosition(e.val[2] + 10.f,e.val[3] + screen.rows-10);
        Scalar txtColor(255.f,255.f,255.f);
        switch(frames[frameId].info)
        {
            case -1:// Unknow
                putText(screen,"Unknow Frame",
                       txtPosition,
                       FONT_HERSHEY_COMPLEX,0.5,
                       txtColor,2);
            break;
            case 0: // To use
                putText(screen,"To cross-validation",
                       txtPosition,
                       FONT_HERSHEY_COMPLEX,0.5,
                       txtColor,2);
            break;
            case 1: // To train
                putText(screen,"To train frame",
                       txtPosition,
                       FONT_HERSHEY_COMPLEX,0.5,
                       txtColor,2);
            break;
            case 2: // No used
                putText(screen,"No used frame",
                       txtPosition,
                       FONT_HERSHEY_COMPLEX,0.5,
                       txtColor,2);
            break;
        }
    }
}

void WFSVM::alocateFrame(unsigned frameId)
{
    if(frameId >= frames.size())
        frames.resize((frameId+1)*2);

    frames[frameId].frameId = frameId;

}

void WFSVM::showSVMFeatures(unsigned frameId, unsigned svmId)
{
    if(!trainingDataReady)
    {
        cout << "WFSVM::showSVMFeatures - Warnning traning data not ready yet!!" << endl;
        return;
    }

    vector<double> &data = frames[frameId].vms[svmId].data;
    vector<double> normData(data.size());
    vector<string> labels(10);
    labels[0] = "Widht";
    labels[1] = "Height";
    labels[2] = "Inertia Ratio";
    labels[3] = "Std Intensity";
    labels[4] = "Mean Intensity";
    labels[5] = "Area";
    labels[6] = "Hull Area";
    labels[7] = "Convexity";
    labels[8] = "Perimeter";
    labels[9] = "Pixel Count";

    for(unsigned i= 0 ; i < data.size();i++)
    {
        normData[i] = (data[i] - minVal[i]) / (maxVal[i]-minVal[i]);
    }

    cout << "Size " << data.size() << endl;
    Mat rImg;
    Drawing::polarPlot(rImg, normData, Size(600,600),labels);
    imshow("SVM Feature View", rImg);
    imwrite("SVMFeature.png", rImg);
}

char WFSVM::classesName[10][200] =
{
    "0",
    "Pole",
    "wharf",
    "BoatHull",
    "Stone",
    "Fish",
    "Swimmer",
    "7","8","9"
};


void WFSVM::start()
{
    CvParamGrid gC = CvSVM::get_default_grid(CvSVM::C);
    CvParamGrid gGamma = CvSVM::get_default_grid(CvSVM::GAMMA);

    cout << "Default C grid parameters: max = " << gC.max_val
         << " min = " << gC.min_val
         << " step = " << gC.step
         << endl;

    cout << "Default Gammagrid parameters: max = " << gGamma.max_val
         << " min = " << gGamma.min_val
         << " step = " << gGamma.step
         << endl;

}

void WFSVM::selectedFrame(int frameId)
{
    lastSelecFrameId = frameId;
}

void WFSVM::keyPress(char c)
{
    int num = c- '0';

    if(num > 0 && num < 10 && lastSelectedObjectId >= 0)
    {
        cout << "Gaussian " << lastSelectedObjectId << " from frame " << lastSelecFrameId
             << " is class " << classesName[num] << endl;
        frames[lastSelecFrameId].setLabel(lastSelectedObjectId,num);
    }

    switch(c)
    {
        case '[':
            if(lastSelecFrameId>=0)
                frames[lastSelecFrameId].clear();
        break;
        case ';':
            autoSave();
//            saveGt("../../GroundTruth/SVM/LARS2016");
        break;
        case 'h':
            if(lastSelecFrameId>=0)
                frames[lastSelecFrameId].info = 0;
        break;
        case 'j':
            if(lastSelecFrameId>=0)
                frames[lastSelecFrameId].info = 1;
        break;
        case 'k':
            if(lastSelecFrameId>=0)
                frames[lastSelecFrameId].info = 2;
        break;
        case 't':
            doTraining();
        break;
        case 'y':
            autoTrain = !autoTrain;
            cout << "Autotrain = " << autoTrain << endl;
        break;
        case 's':
            if(lastSelecFrameId>=0)
                searchSimilar();
        break;
    }
}

void WFSVM::mouseEvent(int event, int x, int y)
{

}

void WFSVM::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    renderFrame(leftImg,leftId);
    renderFrame(rightImg,rightId);
}

void WFSVM::renderFrameTogether(Mat &screen,
                                const Scalar_<float> &el, unsigned leftFrameId,
                                const Scalar_<float> &er, unsigned rightFrameId)
{
    _renderFrameTogether(screen,el,leftFrameId);
    _renderFrameTogether(screen,er,rightFrameId);
}

void WFSVM::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    alocateFrame(leftId);
    alocateFrame(rightId);

}

double WFSVM::euclidianDist(vector<double> v1, vector<double> v2)
{
    if(v1.size() != v2.size())
        return 0.0;

    double sum=0.0, diff=0.0;
    unsigned nv = v1.size();

    for(unsigned i =0 ; i < nv ; i++)
    {
        diff = v1[i] - v2[i];
        sum += diff*diff;
    }
    return sqrt(sum);
}

void WFSVM::searchSimilar()
{
    // Take frames and gaussians information
    int leftFrame = wt->getLeftFrameId(),
        righFrame = wt->getRighFrameId(),
        leftGaussian = _WFGD->getLeftSelectedGaussian(),
        righGaussian = _WFGD->getRighSelectedGaussian();

    if(lastSelectedObjectId == leftGaussian && lastSelecFrameId == leftFrame)
    {
        SVMFrame &leftFr = frames[leftFrame];
        if(!leftFr.hasVm(leftGaussian))
            return ;

        SVMObject &srcVm = leftFr.getVm(leftGaussian);

        int minId =-1;
        double minScore = 99999.99, score;


        SVMFrame &rightFr = frames[righFrame];
        unsigned nVms = rightFr.vms.size();
        for(unsigned i = 0 ; i < nVms; i++)
        {
            if(rightFr.hasVm(i))
            {
                score = euclidianDist(rightFr.getVm(i).data,srcVm.data);
                if(score < minScore)
                {
                    minScore = score;
                    minId = i;
                }
                cout << "ID " << i << " = " << score << endl;
            }
        }
        cout << "Winner ID " << minId << " = " << minScore << endl;

        _WFGD->hightlightRightGaussian = minId;
    }
}

void WFSVM::newGaussian(int gId, int frameId)
{

}

void WFSVM::leftGaussianSelected(int gId, int frameId)
{
    lastSelecFrameId = frameId;
    lastSelectedObjectId = gId;

    if(gId >= 0)
        showSVMFeatures(frameId,gId);
}

void WFSVM::rightGaussianSeleced(int gId, int frameId)
{
    lastSelecFrameId = frameId;
    lastSelectedObjectId = gId;

    if(gId == -1)
        return ;

    Gaussian &g = _WFGD->frames[frameId].gaussians[gId];

    cout << "Hu Moments of gaussian " << gId << ":" << endl;
    for(unsigned i =0  ; i < 7 ; i++)
    {
        cout << "hu[" << i << "] = "
             << g.hu[i] << endl;
    }
}

void WFSVM::cleanedGaussians(int frameId)
{
    frames[frameId].clear();
}

bool WFSVM::save(const char *fileName)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(fileName, "w");
    if(f == 0x0)
    {
        cout << "WFSVM: The file " << fileName << " can not be saved. May be not permission?" << endl;
        return false;
    }

    unsigned frameId=0, vmId;

    // NUmber of frames
    fprintf(f,"%u\n", frames.size());
    for(frameId = 0 ; frameId < frames.size() ; frameId++)
    {
        SVMFrame &fr = frames[frameId];

        // VM Descriptions
        vector<SVMObject> &vms = fr.vms;

        unsigned maxVMCount = std::min(_WFGD->frames[frameId].gaussians.size(), vms.size());

        // Frame description
        // frameID, frameInfo, NumberOfGaussians
        fprintf(f,"%u,%d,%u\n", frameId, fr.info, maxVMCount);
        for(vmId = 0 ; vmId < maxVMCount; vmId++)
        {
            SVMObject &vm = vms[vmId];

            // classId, ObjectId, vectorSize
            fprintf(f,"%u,%d,%d\n",
                vmId, vm.label, vm.objectId);
        }
    }
    fclose(f);

    return true;
}

bool WFSVM::load(const char *fileName)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
    {
        cout << "WFSVM: File " << fileName << " not found to load" << endl;
        return false;
    }

    unsigned frameId=0, vmId=0,frameCount=0,vmCount=0;

    // Number of Frames on this GroundTrutuh
    fscanf(f,"%u", &frameCount);
    frames.resize(frameCount);
    for(frameId = 0 ; frameId < frames.size() ; frameId++)
    {
        SVMFrame &fr = frames[frameId];
        fr.frameId = frameId;

        // Frame description
        // FrameId, FrameInfo, vms count in this frame
        fscanf(f,"%*u,%d,%u",&fr.info,&vmCount);

        if(vmCount == 0) continue;

        // VM Description
        vector<SVMObject> &vms = fr.vms;
        vms.resize(vmCount);
        for(vmId = 0 ; vmId < vmCount; vmId++)
        {
            SVMObject &vm = vms[vmId];

            fscanf(f,"%*u,%d,%d\n",
                    &vm.label, &vm.objectId);
        }
    }
    fclose(f);
    return true;
}

bool WFSVM::loadGt(const char *fileName)
{
    string sufix;

    sufix = "_wt.csv";
    if(!wt->load((fileName + sufix).c_str()))
        return false;

    sufix = "_gd.csv";
    if(!_WFGD->load((fileName + sufix).c_str()))
        return false;

    sufix = "_svm.csv";
    if(!load((fileName + sufix).c_str()))
        return false;
}

bool WFSVM::saveGt(const char *fileName)
{
    string sufix;

    sufix = "_wt.csv";

    if(!wt->save((fileName + sufix).c_str()))
        return false;

    sufix = "_gd.csv";
    if(!_WFGD->save((fileName + sufix).c_str()))
        return false;

    sufix = "_svm.csv";
    if(!save((fileName + sufix).c_str()))
        return false;
}

void WFSVM::setWorkFile(const char *fileName)
{
    workFileName = fileName;
}

void WFSVM::autoSave()
{
    saveGt(workFileName.c_str());
}

bool WFSVM::autoLoad()
{
    if(!loadGt(workFileName.c_str()))
    {
       cout << "Loading default sonar images" << endl;
       wt->loadEmptyFrames("../../Datasets/yacht_05_12_img16bits_especifico.txt");
    }
}


