#include "SegmentSearcher.h"
#include "Segmentation.h"

SegmentSearcher::SegmentSearcher():
    searchMask(0x0),imgMask(0x0),m_seg(0x0),m_extractor(0x0)
{

}

SegmentSearcher::~SegmentSearcher()
{

}

void SegmentSearcher::setExtractor(SegmentExtractor *extractor)
{
    m_extractor = extractor;
}

void SegmentSearcher::setSegmentation(Segmentation *seg)
{
    m_seg = seg;
    searchMask = &(m_seg->searchMask);
    imgMask = &(m_seg->imgMask);
}

void SegmentSearcher::calibUI(Mat &img16bits)
{

}
