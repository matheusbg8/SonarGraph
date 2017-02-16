#include "ThetaRhoSegmentSearcher.h"

#include "SegmentExtractor/SegmentExtractor.h"
#include "Segmentation.h"

#include "Drawing/Chart.h"


ThetaRhoSegmentSearcher::ThetaRhoSegmentSearcher():
    nBeams(720),startBin(20),Hmin(110),bearing(130.f),
    PiEnd(0.6f), PiRecursive(0.98),sonVerticalPosition(1),
    minSampleSize(10)
{

}

void ThetaRhoSegmentSearcher::load(ConfigLoader &config)
{
    int vi;
    float vf;

    if(config.getInt("General","MinSampleSize",&vi))
    {
        minSampleSize = vi;
    }

    if(config.getInt("ThetaRhoSegmentSearcher","sonVerticalPosition",&vi))
    {
        sonVerticalPosition = vi;
    }

    if(config.getInt("ThetaRhoSegmentSearcher","minSampleSize",&vi))
    {
        minSampleSize = vi;
    }

    if(config.getInt("ThetaRhoSegmentSearcher","nBeams",&vi))
    {
        nBeams = vi;
    }

    if(config.getInt("ThetaRhoSegmentSearcher","startBin",&vi))
    {
        startBin = vi;
    }

    if(config.getInt("ThetaRhoSegmentSearcher","Hmin",&vi))
    {
        Hmin = vi;
    }

    if(config.getFloat("ThetaRhoSegmentSearcher","bearing",&vf))
    {
        bearing = vf;
    }

    if(config.getFloat("ThetaRhoSegmentSearcher","PiEnd",&vf))
    {
        PiEnd = vf;
    }

    if(config.getFloat("ThetaRhoSegmentSearcher","PiRecursive",&vf))
    {
        PiRecursive = vf;
    }
}

void ThetaRhoSegmentSearcher::calibUI(Mat &img16bits)
{
    // Algorithm parameters
    int sonVerticalPosition =1;
    float beamAng=0.f;

    char str[300];

    unsigned nBins = img16bits.rows-startBin-11;


    Point2f sonarPos(img16bits.cols/2.f, img16bits.rows+sonVerticalPosition);

    Mat mPlot,mask;

    Chart chart(img16bits.cols,img16bits.rows);
    vector<ushort> intensitys;
    Segment seg;
    unsigned beam=0;
    while (true)
    {
         m_seg->resetMask(img16bits.rows,img16bits.cols);

        float radAng = beamAng*M_PI/180.f;

        Point2f beamPos(sonarPos.x - startBin*sin(radAng),
                        sonarPos.y - startBin*cos(radAng));

        Point2f beamDir(-sin(radAng),-cos(radAng));

        unsigned minBinI=99999, maxBinI=0, minBinN, maxBinN,
                 binI,
                 peakHeight,
                 lastThreshold=99999,
                 peakCount=0,
             accepetedPlot = chart.newLabel(Chart::PLOT_LINE,Drawing::color[beam%Drawing::nColor],1),
          rejectedPeakPlot = chart.newLabel(Chart::PLOT_LINE,Scalar(0,0,255),1);

        img16bits.convertTo(mask,CV_8UC1);
        cvtColor(mask,mask,CV_GRAY2BGR);

        // For each bin
        for(unsigned bin = nBins-1 ; bin < nBins; bin--)// unsigned overflow!
        {
            beamPos+= beamDir;

            binI = img16bits.at<ushort>(beamPos.y,beamPos.x);

            intensitys.push_back(binI);

            mask.at<Vec3b>(beamPos.y,beamPos.x)[0] = Drawing::color[beam%Drawing::nColor].val[0];
            mask.at<Vec3b>(beamPos.y,beamPos.x)[1] = Drawing::color[beam%Drawing::nColor].val[1];
            mask.at<Vec3b>(beamPos.y,beamPos.x)[2] = Drawing::color[beam%Drawing::nColor].val[2];

            if(binI < minBinI)
            {
                minBinI = binI;
                minBinN = nBins - bin -1;
                maxBinI = 0;
            }else
            if(binI > maxBinI)
            {
                 maxBinI = binI;
                 maxBinN = nBins - bin -1;
            }else
            if(maxBinI > minBinI)
            {
                peakHeight = maxBinI - minBinI;

                if( (binI-minBinI) < peakHeight*PiEnd)
                {
                    if(peakHeight > Hmin)
                    {
                        // Accept Peak!
                        lastThreshold = minBinI+peakHeight*PiRecursive;

                        chart.addPoint(accepetedPlot,minBinN,lastThreshold);
                        chart.addPoint(accepetedPlot,nBins - bin -1,lastThreshold);

                        chart.newLabel(Chart::PLOT_CIRCLE,Drawing::color[peakCount%Drawing::nColor],-1);
                        chart.addPoint(maxBinN,maxBinI); // PLot a circle the top

                        chart.newLabel(Chart::PLOT_CIRCLE,Scalar(0,0,0),1);
                        chart.addPoint(maxBinN,maxBinI); // PLot a circle the top

                        Point2f peakPosition;
                        float vecUnit = nBins - bin - maxBinN;
                        peakPosition = beamPos - beamDir * vecUnit;

                        m_extractor->setThreshold(lastThreshold);
                        m_extractor->createSegment(&seg,img16bits,
                                                   peakPosition.y,peakPosition.x);


                        seg.drawSegment(mask,Drawing::color[peakCount%Drawing::nColor]);

                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[0] = 0;
                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[1] = 0;
                        mask.at<Vec3b>(peakPosition.y,peakPosition.x)[2] = 255;

                        circle(mask,peakPosition,7,Drawing::color[peakCount%Drawing::nColor],-1);
                        circle(mask,peakPosition,7,Scalar(0,0,0),1);


                        peakCount++;
                    }else
                    {
                        chart.addPoint(rejectedPeakPlot,minBinN,minBinI);
                        chart.addPoint(rejectedPeakPlot,nBins - bin -1,minBinI);
                    }

                    // Reset Peak (not acceptable)
                    minBinI=binI;
                    minBinN= nBins - bin -1;
                    maxBinI=0;
                }
            }
        }

        chart.newLabel(Chart::PLOT_CONTINUOS_LINE,Drawing::color[beam%Drawing::nColor]);
        for(unsigned i = 0 ; i < nBins ; i++)
            chart.addPoint(float(i),float(intensitys[i]));

        intensitys.clear();

        imshow("TR image mask", mask);
        Drawing::plot(chart,mPlot);
        imshow("TR plot", mPlot);
        chart.clear();

        char c = waitKey();
        switch(c)
        {
        case 'a':
            if(beamAng<65.f)
                beamAng+=1.f;
        break;
        case 'd':
            if(beamAng>-65.f)
                beamAng-=1.f;
        break;
        case 'w':
            Hmin+=5;
        break;
        case 's':
            if(Hmin>5)
                Hmin-=5;
        break;

        case 'r':
            PiEnd+=0.01;
            cout << "PiEnd = " << PiEnd << endl;
        break;
        case 'f':
            PiEnd-=0.01;
            cout << "PiEnd = " << PiEnd << endl;
        break;

        case 't':
            PiRecursive+=0.01;
            cout << "PiRecursive = " << PiRecursive << endl;

        break;
        case 'g':
            PiRecursive-=0.01;
            cout << "PiRecursive = " << PiRecursive << endl;
        break;
        case 'b':
            setlocale(LC_ALL, "C"); // USE POINT AS DECIMAL SEPARATOR!
            sprintf(str,"TRPeak_PiR%.2f_PiF%.2f_Hb%u.png",
                    PiRecursive,PiEnd,Hmin);

            cout << "Saving img " << str << endl;
            imwrite(str,mPlot);

            cout << "Saving img " << str << endl;
            sprintf(str,"TRImg_PiR%.2f_PiF%.2f_Hb%u.png",
                    PiRecursive,PiEnd,Hmin);
            imwrite(str,mask);

        break;

        case 10: case 13: case 27: // keys Enter or ESC
            destroyWindow("TR image mask");
            destroyWindow("TR plot");
            return;
        break;
        }
        cout << "bearign " << beamAng << " Hmin " << Hmin << endl;
    }
}


void ThetaRhoSegmentSearcher::segment(Mat &img16bits, vector<Segment *> *sg)
{
    unsigned nBins = img16bits.rows-startBin;
    float beamAng,
          cAng = -bearing/2.f;

    if(nBeams>1)
        beamAng = bearing/(nBeams-1);
    else beamAng = 2*bearing; // Infinity!

#ifdef SEGMENTATION_DRWING_DEBUG
    Mat result(img16bits.rows, img16bits.cols, CV_8UC3, Scalar(0,0,0));

//    Mat result;
//    img16bits.convertTo(result,CV_8UC1);
//    cvtColor(result,result,CV_GRAY2BGR);
#endif

    Point2f sonarPos(img16bits.cols/2.f, img16bits.rows+sonVerticalPosition);

    unsigned beam=nBeams-1;

    /* Initialize Mask of Visit */
    m_seg->resetMask(img16bits.rows,img16bits.cols);
    sg->clear();

    /* Search for high intensity pixels */
    unsigned segCount=0;
    Segment *seg=0x0;

    // For each beam
    for(unsigned i = 0; i < nBeams; i++, cAng+=beamAng)
    {
        float radAng = cAng*M_PI/180.f;

        Point2f beamPos(sonarPos.x - startBin*sin(radAng),
                        sonarPos.y - startBin*cos(radAng));

        Point2f beamDir(-sin(radAng),-cos(radAng));

        unsigned minBinI=99999, maxBinI=0, minBinN, maxBinN,
                 binI,
                 peakHeight,
                 lastThreshold=99999;

        // For each bin
        for(unsigned bin = nBins-1 ; bin < nBins; bin--)
        // Unsigned overflow!
        {
            beamPos+= beamDir;

            // If pixel was visited
            if(searchMask->at<uchar>(beamPos.y,beamPos.x) == 255)
            {  // Forget all informations about peaks
                minBinI = 99999;
                maxBinI = 0;
                continue;
            }// if not, work with its

            // Save pixel intensity
            binI = img16bits.at<ushort>(beamPos.y,beamPos.x);


            #ifdef SEGMENTATION_DRWING_DEBUG
            // Mark on result img the position of visited pixel
            result.at<Vec3b>(beamPos.y,beamPos.x)[0] = Drawing::color[beam%Drawing::nColor].val[0];
            result.at<Vec3b>(beamPos.y,beamPos.x)[1] = Drawing::color[beam%Drawing::nColor].val[1];
            result.at<Vec3b>(beamPos.y,beamPos.x)[2] = Drawing::color[beam%Drawing::nColor].val[2];
            #endif

            if(binI < minBinI)
            {
                minBinI = binI;
                minBinN = nBins - bin -1;
                maxBinI = 0;
            }else
            if(binI > maxBinI)
            {
                 maxBinI = binI;
                 maxBinN = nBins - bin -1;
            }else
            if(maxBinI > minBinI)
            {
                peakHeight = maxBinI - minBinI;

                if( (binI-minBinI) < peakHeight*PiEnd)
                {
                    if(peakHeight > Hmin)
                    {
                        // Accept Peak!
                        lastThreshold = minBinI+peakHeight*PiRecursive;

                        // Compute the peak position on sonar XY image
                        float vecUnit = nBins - bin - maxBinN;
                        Point2f peakPosition(beamPos - beamDir * vecUnit);

                        // Search the segment on image
                        seg = m_seg->segment(segCount);

                        m_extractor->setThreshold(lastThreshold);
                        m_extractor->createSegment(seg,img16bits,
                                                   peakPosition.y,peakPosition.x);

                        // If segment is greater tham minimum acceptable segment size
                        if(seg->N >= minSampleSize)
                        {
                            segCount++;

                            #ifdef SEGMENTATION_DRWING_DEBUG
                                seg->drawSegment(result,Drawing::color[segCount%Drawing::nColor]);
                            #endif

                            // Add seg to answer
                            sg->push_back(seg);
                        }

                    }

                    // Reset Peak
                    minBinI=binI;
                    minBinN= nBins - bin -1;
                    maxBinI=0;

                }
            }
//            else
//            {
//                cout << "Caiu nesse caso!" << endl;
//            }
        }

        beam--;
    }

    #ifdef SEGMENTATION_DRWING_DEBUG
    imshow("TR image result", result);
    #endif
}
