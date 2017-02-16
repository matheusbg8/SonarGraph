#ifndef CSVREADER2_H
#define CSVREADER2_H

#include <vector>
#include <cstdarg>
#include <cstdio>
#include <string>

#include <CSVReader/CSVReader.h>

using namespace std;

class CSVReader2
{
protected:
    FILE *csvFile;
    vector<CSV_TYPE> types;
    vector<void*> adress;
    string csvFileName;
    char spliter;
public:
    CSVReader2();
    ~CSVReader2();

    bool open(const char *fileName, unsigned nCols, const char spliter, ...);
    void close();

    void ignoreLine();

    bool read(void *field1,...);
};

#endif // CSVREADER2_H
