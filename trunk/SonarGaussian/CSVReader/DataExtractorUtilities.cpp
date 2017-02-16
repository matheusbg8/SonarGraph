#include <CSVReader/DataExtractorUtilities.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>


void endLine(FILE *source)
{
    char c = 0;
    while( (c=getc(source)) &&
          c != 0 && c!= -1 && c != '\n');
}


void removeCharacter(char *str, char caracter)
{
    char *p=str , *c=str;
    while(*p != 0)
    {
        if(*p != caracter)
        {
            *c = *p;
            c++;
        }
        p++;
    }
    *c = 0;
}

void removeQuotationMarks(char *str)
{
    char *p = str, *a1=0 , *a2=0;
    while(*p != 0)
    {
        if(*p == 34)
        {
            if(a1 != 0)
                a2 = p;
            else a1 = p;
        }
        p++;
    }
    if(a2 != 0x0)
    {
        while(*(a2+1) != 0)
        {
            *a2 = *(a2+1);
            a2++;
        }
        *a2 = 0;
    }
    if(a1!=0x0)
    {
        while(*(a1+1) != 0)
        {
            *a1 = *(a1+1);
            a1++;
        }
        *a1 = 0;
    }
}


int extractField(FILE *source, char *field, char split)
{
    char c=0, *p = field;
    while((c = getc(source)) &&
          c != '\n' && c != split && c != -1)
    {
        *p = c;
        p++;
    }
    *p = '\0';
    if(c == split || c == '\n')
        return p - field;
    else return -1;
}

int extractField(FILE *source, char *field, char cStart, char cEnd)
{
    char c=0, *p = field;
    int count=0;

    while((c = getc(source)) > 0)
        if(c == cStart){ count++; break;}

    while((c = getc(source)) > 0)
    {
        if(c == cEnd)
            count--;
        else if(c == cStart)
            count++;

        if(count > 0)
        {
            *p = c;
            p++;
        }else break;
    }
    *p = '\0';

    if(c == cEnd)
        return p- field;
    else return -1;
}

int extractField(FILE *source, char *field, const char* cStart, const char * cEnd)
{
    char c=0, *p = field;
    const char *pStart = cStart, *pEnd = cEnd;
    int count=0; // Fild mark count

    // Find start field
    while((c = getc(source)) > 0)
    {
        if(c == *pStart)
        {
            pStart++;
            if(*pStart == '\0')
            {
                count++;
                pStart = cStart;
                break;
            }
        }else pStart = cStart;
    }

    // Extract until end mark
    if(count > 0)
    while((c = getc(source)) > 0)
    {
        // Do end mark is?
        if(c == *pEnd)
        {
            pEnd++;
            if(*pEnd == '\0')
            {
                count--;
                pEnd = cEnd;
            }
        }else pEnd = cEnd;


        // Do start mark is?
        if(c == *pStart && count > 0)
        {
            pStart++;
            if(*pStart == '\0' && pEnd == cEnd)
            {
                count++;
                pStart = cStart;
            }
        }else pStart = cStart;

        *p = c;
        p++;
        if(count <= 0) break;
    }

    // If file characters end without field characters
    if(c == 0 && (  p - field) == 0 )
    {
        return -1;
    }

    // Put end string mark without end field mark
    p-=strlen(cEnd);
    *p = '\0';
    return p- field;
}


int sExtractField(char **source, char *field, char split)
{
    char c, *p = field;
    while( (c = **source) &&
          c != '\n' && c != split && c != -1)
    {
        *p = c;
        p++;
        (*source)++;
    }
    (*source)++;
    *p = '\0';
    if(c == split || c == '\n')
        return  p- field;
    else return -1;
}

int sExtractField(char **source, char *field, char cStart, char cEnd)
{
    char c=0, *p = field;
    int count=0;

    while( (c = **source) > 0)
    {
        (*source)++;
        if(c == cStart)
        { count++; break;}
    }

    while((c = **source) > 0)
    {
        (*source)++;
        if(c == cEnd)
            count--;
        else if(c == cStart)
            count++;

        if(count > 0)
        {
            *p = c;
            p++;
        }else break;
    }
    *p = '\0';

    if(c == cEnd)
        return p-field;
    else return -1;
}

int sExtractField(char **source, char *field, const char* cStart, const char * cEnd)
{
    char c=0, *p = field;
    const char *pStart = cStart, *pEnd = cEnd;
    int count=0; // Fild mark count

    // Find start field
    while( (c = **source) > 0)
    {
        (*source)++;
        if(c == *pStart)
        {
            pStart++;
            if(*pStart == '\0')
            {
                count++;
                pStart = cStart;
                break;
            }
        }else pStart = cStart;
    }

    // Extract until end mark
    if(count > 0)
    while((c = **source) > 0)
    {
        (*source)++;
        // Do end mark is?
        if(c == *pEnd)
        {
            pEnd++;
            if(*pEnd == '\0')
            {
                count--;
                pEnd = cEnd;
            }
        }else pEnd = cEnd;


        // Do start mark is?
        if(c == *pStart && count > 0)
        {
            pStart++;
            if(*pStart == '\0' && pEnd == cEnd)
            {
                count++;
                pStart = cStart;
            }
        }else pStart = cStart;

        *p = c;
        p++;
        if(count <= 0) break;
    }

    // If file characters end without field characters
    if(c == 0 && (  p - field) == 0 )
    {
        return -1;
    }

    // Put end string mark without end field mark
    p-=strlen(cEnd);
    *p = '\0';
    return p- field;
}

void readLine(FILE *source)
{
    fscanf(source, "%*[^\n]\n", NULL);
}

void readHeader(FILE *source)
{
    int ID;
    char label[100], str[100];

    fscanf(source,"%*s %*s %*s ");

    extractField(source,str,'|');
    // While str dif statusId do ...
    while(strcmp(str,"statusId") != 0)
    {
        ID = atoi(str);
        extractField(source,label,'|');

        extractField(source,str,'|');
    }
    fscanf(source, "%*s ");
}

int extractCordinate(FILE *source, double x, double y, double z)
{
    // [xValue, yValue,  zValue]

    char value[50];
    extractField(source, value,'[', ',');
    x = atof(value);
    extractField(source, value,',');
    y = atof(value);
    extractField(source, value,']');
    z = atof(value);
}

int extractCordinate(char *source, double *x, double *y, double *z)
{
    // [xValue, yValue,  zValue]

    char value[50];
    sExtractField(&source, value,'[', ',');
    *x = atof(value);
    sExtractField(&source, value,',');
    *y = atof(value);
    sExtractField(&source, value,']');
    *z = atof(value);
}

void extractGPSMsg(char *str, char *latitude, char *longitude, char *altitude, char *time, char *accuracy)
{
    // {""mResults"":[0.0,0.0],""mProvider"":""fused"",""mDistance"":0.0,""mTime"":1419803565788,""mAltitude"":418.0164870463307,""mLongitude"":-53.4792245,""mLon2"":0.0,""mLon1"":0.0,""mLatitude"":-30.5006009,""mLat1"":0.0,""mLat2"":0.0,""mInitialBearing"":0.0,""mHasSpeed"":false,""mHasBearing"":false,""mHasAltitude"":true,""mHasAccuracy"":true,""mAccuracy"":24.0,""mSpeed"":0.0,""mBearing"":0.0}

    char varName[200];

    while(sExtractField(&str,varName,"\"\"","\"\"")!= -1)
    {
        if(strcmp(varName,"mTime") == 0)
        {
            sExtractField(&str,time,':',',');
        }else if(strcmp(varName,"mAltitude") == 0)
        {
            sExtractField(&str,altitude,':',',');
        }else if(strcmp(varName,"mLongitude") == 0)
        {
            sExtractField(&str,longitude,':',',');
        }else if(strcmp(varName,"mLatitude") == 0)
        {
            sExtractField(&str,latitude,':',',');
        }else if(strcmp(varName,"mAccuracy") == 0)
        {
            sExtractField(&str,accuracy,':',',');
        }
    }
}


void extractGPSMsgSensorLOG(char *str, char *latitude, char *longitude, char *altitude, char *time, char *accuracy)
{
    // {"mResults":[0.0,0.0],"mProvider":"fused","mDistance":0.0,"mTime":1444678398513,"mAltitude":260.83234827740046,"mLongitude":-53.089211,"mLon2":0.0,"mLon1":0.0,"mLatitude":-30.9438022,"mLat1":0.0,"mLat2":0.0,"mInitialBearing":0.0,"mHasSpeed":true,"mHasBearing":false,"mHasAltitude":true,"mHasAccuracy":true,"mAccuracy":64.0,"mSpeed":0.0,"mBearing":0.0}
    char varName[200];

    while(sExtractField(&str,varName,"\"","\"")!= -1)
    {
        if(strcmp(varName,"mTime") == 0)
        {
            sExtractField(&str,time,':',',');
        }else if(strcmp(varName,"mAltitude") == 0)
        {
            sExtractField(&str,altitude,':',',');
        }else if(strcmp(varName,"mLongitude") == 0)
        {
            sExtractField(&str,longitude,':',',');
        }else if(strcmp(varName,"mLatitude") == 0)
        {
            sExtractField(&str,latitude,':',',');
        }else if(strcmp(varName,"mAccuracy") == 0)
        {
            sExtractField(&str,accuracy,':',',');
        }
    }
}

int extractDataCSV(FILE *source,int *msgID, int *labelID,
                   char *sensorName,double *accuracy,
                   char*sensorValue, char *timeStamp)
{
//    "_id", "statusId", "sensorName", "accuracy", "value", "timestamp"
//    "2034975", "4", "K3G Gyroscope Sensor", "0", "[-0.0,-0.0,-0.0]", "1480698996949"

    // Remove header
    readLine(source);

    // ID
    extractField(source,sensorValue,'\"','\"');
    *msgID = atoi(sensorValue);
    // statusID
    extractField(source,sensorValue,'\"','\"');
    *labelID = atoi(sensorValue);
    // SensorName
    extractField(source,sensorName,'\"','\"');
    // Accuracy
    extractField(source,sensorValue,'\"','\"');
    *accuracy = atof(sensorValue);
    // Value
    extractField(source,sensorValue,'\"','\"');
    // Time Stamp
    extractField(source,timeStamp,'\"','\"');
}

int extractData(FILE *source,int *msgID, int *labelID, char *sensorName,double *accuracy, char*sensorValue, char *timeStamp)
{
    readHeader(source);

    *labelID=0;
    *msgID=0;
    *accuracy =0.0;

    extractField(source,sensorValue,'\"','\"');
    *msgID = atoi(sensorValue);

    extractField(source,sensorValue,",\"","\",");
    *labelID = atoi(sensorValue);

    extractField(source,sensorName,"\"","\",");

    extractField(source,sensorValue,"\"","\",");
    sscanf(sensorValue, "%lf", accuracy);

    extractField(source,sensorValue,"\"","\",");

    return extractField(source,timeStamp,'\"','\"');
}


int extractDataSensorLog(FILE *source, int *labelID, char *sensorName, char *sensorValue, char *timeStamp)
{
    *labelID=0;
    *sensorName = '\0';
    *timeStamp = '\0';

    extractField(source,sensorValue,'|');
    *labelID = atoi(sensorValue);

    extractField(source,sensorName,'|');

    extractField(source,sensorValue,'|');

    return extractField(source,timeStamp,'|');
}


