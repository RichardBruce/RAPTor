#include "simd.h"
#include <iostream>
#include <vector>
#include "x86timer.hpp"
#include <fstream>

using namespace std;

int main(void)
{
    x86timer t;

    vector<vfp_t> data,results; // vector data
    vector<float> datas,resultss; // scalar data

#define N 10000
    for (int i=0;i<N;i++)
	{
    	data.push_back(vfp_t(i,i+1,i+2,i+3));
    	results.push_back(vfp_t(0,0,0,0));
	}

    for (int i=0;i<N;i++)
    {
        datas.push_back(i);
        datas.push_back(i+1);
        datas.push_back(i+2);
        datas.push_back(i+3);
        resultss.push_back(0);
        resultss.push_back(0);
        resultss.push_back(0);
        resultss.push_back(0);
    }

    unsigned long tim=0; // vector time 
    unsigned long tim2=0; // scalar time

    // Number of repeats to get error down
    for (int r=0;r<1000;r++)
    {
        t.start();
        for (int i=0;i<data.size();i++)
        {
            //
            //	Vector benchmark code here
            //
            results[i]=approx_inverse_sqrt(data[i]) * data[i] + data[i];
        }
        tim += t.stop();

        t.start();
        for (int i=0;i<datas.size();i++)
    	{
        	//
        	//	Scalar benchmark code here
        	//
        	resultss[i]=1.0 / sqrt(datas[i]) * datas[i] + datas[i];
    	}

        tim2 += t.stop();
    } // end repeats

    ofstream file("data.dat",ios::out);
    ofstream file2("datas.dat",ios::out);

    for (int i=0;i<results.size();i++)
	    file << results[i][0] << " " << results[i][1] << " " << results[i][2] << " " << results[i][3] << endl;

    int count=0;
    for (int i=0;i<resultss.size();i++)
	{
	    file2 << resultss[i] << " ";
	    count++;
	    if (count == 4)
		{
		    count=0;
		    file2 << endl;
		}
	}

    cout << "Vector path faster by factor: " << double(tim2)/double(tim) << endl;
}
