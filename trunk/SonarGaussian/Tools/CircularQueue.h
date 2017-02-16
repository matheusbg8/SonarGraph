#ifndef CIRCULARQUEUE_H
#define CIRCULARQUEUE_H

class CircularQueue
{
private:
    int *m_el;
    unsigned m_beg, m_end, m_sz, m_msz;

public:
    CircularQueue(unsigned maxSize);
    ~CircularQueue();

    unsigned size();
    void clear();
    bool empty();
    void ResizeAndClear(unsigned newSize);
    unsigned maxSize();

    bool push(int v);
    bool pop();
    int front();
    int back();

};


// ======= Test Section =========

//#include "Tools/CircularQueue.h"
//#include <iostream>
//using namespace std;

//int main()
//{
//    CircularQueue q;

//    q.ResizeAndClear(20);

//    for(unsigned i  = 0 ; q.push(i) ; i++)
//    {
//        cout << q.front()
//             << " , "
//             << q.back()
//             << " , "
//             << q.size()
//             << endl;
//    }

//    while(q.pop())
//    {
//        cout << q.front()
//             << " , "
//             << q.back()
//             << " , "
//             << q.size()
//             << endl;
//    }
//    return 0;
//}

#endif // CIRCULARQUEUE_H
