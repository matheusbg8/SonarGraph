#ifndef CONFIGLOADER_H
#define CONFIGLOADER_H

#include <string>
#include <map>
#include <cstdio>
#include <iostream>
using namespace std;

#include "Automaton.h"

class ConfigLoader
{
private:
    map<string, map<string,string> > configs;

    Automaton extractTag,
              extractValue;

    static const string empty;
public:
    string name;

    ConfigLoader(const char *fileName=0x0);

    bool load(const char *fileName);

    bool has(const char *tag, const char *key);

    const string &getString(const char *tag, const char *key, bool *ok=0x0);
    bool getString(const char *tag, const char *key, string* str);

    float getFloat(const char *tag, const char *key);
    bool getFloat(const char *tag, const char *key, float *value);

    int getInt(const char *tag, const char *key);
    bool getInt(const char *tag, const char *key, int *value);

    void printConfigs();
};

/*
// Some tests
#include "Sonar/SonarConfig/ConfigLoader.h"

int main(int argc, char* argv[])
{
    SonarConfig sg("../SonarGaussian/Configs.txt");

    sg.printConfigs();
    cout << sg.getFloat("GraphCreator","DistToLink")
         << endl
         << sg.getInt("LinearSegmentation","RhoRecursive")
         << endl;

    return 0;
}
*/

#endif // CONFIGLOADER_H
