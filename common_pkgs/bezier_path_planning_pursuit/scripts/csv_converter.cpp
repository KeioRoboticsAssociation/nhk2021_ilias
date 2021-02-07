// csv.converter.cpp

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

#define LINE_NUMBER 26

void csv_converter(string infilename);
void create_yobi(string infilename);
string replace_str(string replacedStr, string from, string to);

void csv_converter(string infilename)
{
    // red or blue
    bool redflag = true;
    if (infilename.find("red") == string::npos)
    {
        redflag = false;
        if (infilename.find("blue") == string::npos)
        {
            cerr << "csv filename error" << endl;
            exit(1);
        }
    }

    string s;
    int count = 0;
    ifstream infile;
    ofstream outfile;

    // check readable
    infile.open(infilename);
    if (!infile)
    {
        cerr << "can not open file" << endl;
        exit(1);
    }
    while (getline(infile, s))
    {
        count++;
    }
    if (count < 1)
    {
        cerr << "csv data error" << std::endl;
        exit(1);
    }
    infile.close();

    // outfile name
    string outfilename;
    if (redflag)
        outfilename = replace_str(infilename, "red", "blue");
    else
        outfilename = replace_str(infilename, "blue", "red");

    // yobi file
    create_yobi(infilename);
    create_yobi(outfilename);

    // read file
    count = 0;
    infile.open(infilename);
    outfile.open(outfilename);
    while (getline(infile, s))
    {
        count++;
        if (count == 1)
        {
            outfile << s << endl;
            continue;
        }
        // read data
        stringstream ss_data{s};
        string data_s[3];
        float data_f[3] = {0};
        bool notanumber = false;
        for (int i = 0; i < 3; i++)
        {
            stringstream ss;
            getline(ss_data, data_s[i], ',');
            if (data_s[i] == "-")
            {
                notanumber = true;
                break;
            }
            ss << data_s[i];
            ss >> data_f[i];
        }

        // edit data
        stringstream ssw;
        data_f[0] *= -1;
        if (notanumber)
        {
            ssw << data_f[0] << "," << data_f[1] << ",-";
            outfile << ssw.str() << endl;
            continue;
        }
        else if (2575 - 1000 < abs(data_f[0]) && abs(data_f[0]) < 5075)
        { // kicking zone
            data_f[2] *= -1;
            if (redflag)
            {
                if (data_f[2] > 180 + 180)
                    data_f[2] -= 360;
                else if (data_f[2] < -180 + 180)
                    data_f[2] += 360;
            }
            else
            {
                if (data_f[2] > 180)
                    data_f[2] -= 360;
                else if (data_f[2] < -180)
                    data_f[2] += 360;
            }
        }
        else
        { // receive or try zone
            if (data_f[1] > 1000)
            { // not start zone
                if (redflag)
                    data_f[2] += 180;
                else
                {
                    if (data_f[2] > 0)
                        data_f[2] -= 180;
                    else
                        data_f[2] += 180;
                }
            }
        }
        ssw << data_f[0] << "," << data_f[1] << "," << data_f[2];
        outfile << ssw.str() << endl;
    }
    infile.close();
    outfile.close();
}

void create_yobi(string infilename)
{
    string s;
    int count = 0;
    ifstream infile;
    ofstream outfile;

    infile.open(infilename);
    outfile.open(replace_str(infilename, "/", "_yobi/"));
    if (!infile)
    {
        cerr << "can not open file" << endl;
        return;
    }

    while (getline(infile, s))
    {
        outfile << s << endl;
    }
    infile.close();
    outfile.close();
}

string replace_str(string replacedStr, string from, string to)
{
    const unsigned int pos = replacedStr.find(from);
    const int len = from.length();

    if (pos == std::string::npos || from.empty())
    {
        return replacedStr;
    }

    return replacedStr.replace(pos, len, to);
}