#include "SegmentExtractor.h"


#include "Segmentation.h"

SegmentExtractor::SegmentExtractor():
    searchMask(0x0),m_seg(0x0)
{

}

SegmentExtractor::~SegmentExtractor()
{

}

void SegmentExtractor::setSegmentation(Segmentation *seg)
{
    m_seg = seg;
    searchMask = &(m_seg->searchMask);
}

void SegmentExtractor::setThreshold(unsigned threshold)
{

}

void SegmentExtractor::calibUI(Mat &img16bits)
{

}

