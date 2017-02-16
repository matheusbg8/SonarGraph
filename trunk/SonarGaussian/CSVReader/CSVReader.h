#ifndef CSVREADER_H
#define CSVREADER_H

#include <vector>
#include <cstdarg>
#include <cstdio>

enum CSV_TYPE
{
    CSV_STRING,
    CSV_INT, // int_32
    CSV_LLU, // uint_64
    CSV_FLOAT,
    CSV_DOUBLE,
    CSV_INVALID,
    CSV_IGNORE
};

enum CSV_MSG
{
    CSV_CURRENT_MSG,
    CSV_SPECIFIC_MSG
};

class CSVData;

class CSVReader
{
protected:

    static unsigned _fieldCount(FILE *file, const char spliter);
    static bool _isCommentLine(FILE *file);

    CSVReader(){};
public:

    static CSVData* read(const char *fileName, const char spliter = ',', ...);
};

#endif // CSVREADER_H
