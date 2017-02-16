#include "TestSVMFeature.h"

TestSVMFeature::TestSVMFeature():
    currentClass(-1.0),
    kernel_type(CvSVM::RBF),
//    gamma(8.85),
//    C(9.2),
    gamma(41.55),
    C(28.45),
    normalizeData(true),autoTrain(false)
{
}

Scalar TestSVMFeature::getClassColor(float label)
{
    if( (label - (int) label) != 0)
    {
        cout << "Asked class " << label << endl;
    }

    if(label >= -1.0 && label <= -0.4)
    {
        return Scalar(0.f,255.f,0.f);
    }else if(label > -0.4 && label <= 0.4)
    {
        return Scalar(255.f,0.f,0.f);

    }else if(label > 0.4 && label <= 1.4)
    {
        return Scalar(0.f,0.f,255.f);

    }else if(label > 1.4 && label <= 2.4)
    {
        return Scalar(255.f,255.f,0.f);
    }else if(label > 2.4 && label <= 3.4)
    {
        return Scalar(0.f,255.f,255.f);
    }else if(label > 3.4 && label <= 4.4)
    {
        return Scalar(255.f,0.f,255.f);
    }
}

void TestSVMFeature::autoSetMaxMinValues()
{
    Size winSize = _SIW->windowSize;
    maxV1 = winSize.width;
    maxV2 = winSize.height;
    minV1 = 0;
    minV2 = 0;
}

void TestSVMFeature::train()
{
    Mat labels(vs.size(),1,CV_32FC1,Scalar(0)),
        data(vs.size(),2,CV_32FC1,Scalar(0));

    for(unsigned i = 0; i < vs.size() ; i++)
    {
        labels.at<float>(i,0) = vs[i].label;

        if(normalizeData)
        {
            data.at<float>(i,0) = (vs[i].x - minV1)/(maxV1-minV1);
            data.at<float>(i,1) = (vs[i].y - minV2)/(maxV2-minV2);
        }else
        {
            data.at<float>(i,0) = vs[i].x;
            data.at<float>(i,1) = vs[i].y;
        }
    }

    cout << "Labels: " << labels << endl;
    cout << "Data: " << data << endl;

    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = kernel_type;
    params.gamma = gamma;
    params.C = C;
    params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    if(autoTrain)
    {
        SVM.train_auto(data, labels, Mat(), Mat(), params,3);
//    train_auto(data, labels, Mat(), Mat(), params, int k_fold=10, CvParamGrid Cgrid=CvSVM::get_default_grid(CvSVM::C), CvParamGrid gammaGrid=CvSVM::get_default_grid(CvSVM::GAMMA), CvParamGrid pGrid=CvSVM::get_default_grid(CvSVM::P), CvParamGrid nuGrid=CvSVM::get_default_grid(CvSVM::NU), CvParamGrid coeffGrid=CvSVM::get_default_grid(CvSVM::COEF), CvParamGrid degreeGrid=CvSVM::get_default_grid(CvSVM::DEGREE), bool balanced=false)

        CvSVMParams resParams = SVM.get_params();
        C = resParams.C;
        gamma = resParams.gamma;
        cout << "AutoTrain:" << endl
             << "Gamma = " << gamma << endl
             << "C = " << C << endl;
    }else
    {
        SVM.train(data, labels, Mat(), Mat(), params);
    }

    showTrainResults(SVM);
}

void TestSVMFeature::showTrainResults(CvSVM &SVM)
{
    Size windowSize = _SIW->windowSize;
    Mat imResult(windowSize.height, windowSize.width,CV_8UC3,Scalar(0,0,0));

    for(unsigned i = 0; i < windowSize.height;i++)
    {
        for(unsigned j = 0 ; j < windowSize.width ; j++)
        {
            Mat sampleMat;
            if(normalizeData)
                sampleMat = (Mat_<float>(1,2) << (j-minV1)/(maxV1-minV1),(i-minV2)/(maxV2-minV2));
            else
                sampleMat = (Mat_<float>(1,2) << j,i);

            float response = SVM.predict(sampleMat);

            Scalar color = getClassColor(response);
            imResult.at<Vec3b>(i,j)  = Vec3b(color.val[0],color.val[1],color.val[2]);
        }
    }
    render(imResult);
    imshow("Train Results", imResult);
}

void TestSVMFeature::reset()
{
    vs.clear();
}

void TestSVMFeature::mouseClick(int x, int y)
{
    vs.push_back(TestSVMData(currentClass,x,y));
}

void TestSVMFeature::mouseDrag(int x, int y)
{

}

void TestSVMFeature::render(Mat &imgBgr)
{
    for(unsigned i = 0 ; i < vs.size();i++)
    {
        circle(imgBgr,
               Point2f(vs[i].x,vs[i].y),
               3,
               getClassColor(vs[i].label),
               -1);
        circle(imgBgr,
               Point2f(vs[i].x,vs[i].y),
               3,
               Scalar(0.f,0.f,0.f),
               1);
    }
}

void TestSVMFeature::keyPress(char c)
{
    switch(c)
    {
    case '1':
        currentClass = -1.0;
    break;
    case '2':
        currentClass = 0.0;
    break;
    case '3':
        currentClass = 1.0;
    break;
    case '4':
        currentClass = 2.0;
    break;
    case '5':
        currentClass = 3.0;
    break;
    case '6':
        currentClass = 4.0;
    break;
    case 'r':
        reset();
    break;
    case 't':
        train();
    break;
    case 'y':
        autoTrain = !autoTrain;
        cout << "Autotrain = " << autoTrain << endl;
    break;
    case 'w':
        gamma+=0.005;
        cout << "Gamma = " << gamma << endl;
    break;
    case 's':
        gamma-=0.005;
        cout << "Gamma = " << gamma << endl;
    break;
    case 'e':
        C+=0.1;
        cout << "C = " << C << endl;
    break;
    case 'd':
        C-=0.1;
        cout << "C = " << C << endl;
    break;
    case 'q':
        kernel_type = CvSVM::LINEAR;
        cout << "Using linear kernel" << endl;
    break;
    case 'a':
        kernel_type = CvSVM::RBF;
        cout << "Using Radial (RBF) Kernel" << endl;
    break;
    case 'n':
        normalizeData = !normalizeData;
        cout << "Normalize = " << normalizeData << endl;
    break;
    }
}

void TestSVMFeature::start()
{
    autoSetMaxMinValues();
}
