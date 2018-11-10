
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace std;

extern "C" {

char StandbyInput(char *inputbuf)
{
    string sInput;

    getline(cin,sInput,'\n');
    strcpy(inputbuf, sInput.c_str());
    return false;
}

}

