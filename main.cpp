#include <iostream>
#include <iomanip>
#include <fstream>
#include "SensorFusion.h"
#include "IIR_I.h"
#include <string.h>
#define sample_time 0.001
using namespace std;

int main (int argc, char** argv)
{
    
    
    float time_all =0.0, time_start = 0.0;
    char buf[128];
    FILE *pp;
    if( (pp = _popen("adb shell mpu_iio -r", "r")) == NULL )
    {
         printf("popen() error!/n");
		 return 1;
    }
	FILE* calibrationFile = fopen("D://aiglass//aiglass_program//sensor-fusion//self_calibration.txt", "r");
	FILE* pfile = fopen("D://aiglass//aiglass_program//sensor-fusion//Euler.txt", "w");
	if (calibrationFile == NULL||pfile==NULL)
	{
		printf("Open Clibration Failed!!\n");
		return 1;
	}
	else
	{
		printf("Calibration File is existed\n");
	}
    char flag[128];
    vector <float> gyro;
    vector <float> accel;
    float x, y, z;
    unsigned long long int time_stamp = 0;
	/////////////////////////////////////////////////////////////////////////////////////
	/////////////bb and aa is the coefficient of IIR high-pass filter///////////////////
	/////////////b and a is the coefficient of IIR low-pass filter//////////////////////
	////////////IIR filter : order = 2;filtCutOff = 0.4;samplePeriod=0.001;////////////
	double bb[3] = {0.998224425025158,-1.996448850050315,0.998224425025158};
	double aa[3] = {1.000000000000000,-1.996445697381339,0.996452002719291};
	double b[3] = { 0.00000157633448805150,0.00000315266897610300,0.00000157633448805150};
	double a[3] = {1.000000000000000,-1.996445697381339,0.996452002719291};//  
	int order=2;
	SensorFusion sf(calibrationFile);
	IIR_I filterpx,filterpy,filterpz,filterppx,filterppy,filterppz,filterpppx,filterpppy,filterpppz; 
	filterpx.setPara(b, order, a, order);
	filterpy.setPara(b, order, a, order);
	filterpz.setPara(b, order, a, order);
	filterppx.setPara(bb,order, aa, order);
	filterppy.setPara(bb, order, aa, order);
	filterppz.setPara(bb, order, aa, order);
	filterpppx.setPara(bb,order, aa, order);
	filterpppy.setPara(bb, order, aa, order);
	filterpppz.setPara(bb, order, aa, order);
	////////////inite some variable///////////////////////////////////////////////////////////
    double dis[3]={0.0,0.0,0.0};
	double vel[3]={0.0,0.0,0.0};
	double velx=0.0,vely=0.0,velz=0.0,posx=0.0,posy=0.0,posz=0.0,velxx=0.0,velyy=0.0,velzz=0.0;
	////////////do main "while" work/////////////////////////////////////////////////////////////////
	while(fgets(buf, sizeof(buf), pp))
    {
        sscanf (buf,"%s %f %f %f %llu\n",&flag, &x, &y, &z, &time_stamp);
        printf("%s", buf);
        if(strcmp(flag, "Accel:") == 0)
        {
            accel.clear();
            accel.push_back(x);
            accel.push_back(y);
            accel.push_back(z);
        }
		else if(strcmp(flag, "Gyro:") == 0 && accel.size()==3)
		{
			gyro.clear();
			gyro.push_back(x);
			gyro.push_back(y);
			gyro.push_back(z);

			sf.SensorPretreatment(gyro);
			sf.handlemessage(accel, gyro, 0.001);

			////low-pass filter////
			velx=filterpx.filter(sf.StateTransformPos[0],velx);
			vely=filterpy.filter(sf.StateTransformPos[1],vely);
			velz=filterpz.filter(sf.StateTransformPos[2],velz);
			/////////////////////////////
			vel[0]+=velx*0.001;
			vel[1]+=vely*0.001;
			vel[2]+=velz*0.001;
			//////high-pass filter//////
			velxx=filterppx.filter(vel[0],velxx);
			velyy=filterppy.filter(vel[1],velyy);
			velzz=filterppz.filter(vel[2],velzz);
			dis[0]+=velxx*0.001; 
			dis[1]+=velyy*0.001;
			dis[2]+=velzz*0.001;
			//////if you needed you can use pos high-pass filter,but this can make some risks of wrong/////////////
			//////the pos high-pass filter need to research in the furter or use other way to process pos data/////
			posx=filterpppx.filter(dis[0],posx);
			posy=filterpppy.filter(dis[1],posy);
			posz=filterpppz.filter(dis[2],posz);
			//cout<<vel[0]<<" "<<vel[1]<<" "<<vel[2]<<endl;
			//fprintf(pfile,"%f %f %f\n",sf.Euler[0]*57.3,sf.Euler[1]*57.3,sf.Euler[2]*57.3);
			fprintf(pfile,"%lf %lf %lf\n",dis[0],dis[1],dis[2]);
		}
    }

    _pclose(pp);
	fclose(pfile);
	return 1;
}