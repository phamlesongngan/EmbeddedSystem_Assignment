#include    "_9DOF.hpp"

#include <cstring>
#include "mraa.hpp"
#include <iostream>
#include <unistd.h>
#include <cmath>
#include "SFE_LSM9DS0.h"

using namespace std;

// ********************** global variables ***********************
LSM9DS0 *imu;
uint16_t imuResult;

bool newAccelData   = false;
bool newMagData     = false;
bool newGyroData    = false;
bool overflow       = false;

short   ntime = 0;                  // count to 10 to make 100ms interval


int*    a = NULL;                   // saving acce values
int     n = 0;                      // number of elements in array a


void    init_9DOF()
{    
    imu = new LSM9DS0(0x6B, 0x1D);

    imuResult = imu->begin();
    cout<<hex<<"Chip ID: 0x"<<imuResult<<dec<<" (should be 0x49d4)"<<endl;

    a = new int[2000];
}

void    capture()
{
    while ((newGyroData & newAccelData & newMagData) != true)
    {
        if (newAccelData != true)
        {
            newAccelData = imu->newXData();
        }
        if (newGyroData != true)
        {
            newGyroData = imu->newGData();
        }
        if (newMagData != true)
        {
            newMagData = imu->newMData();
        } 
    }

    newAccelData = false;
    newMagData   = false;
    newGyroData  = false;

    // Of course, we may care if an overflow occurred; we can check that
    //    easily enough from an internal register on the part. There are functions
    //    to check for overflow per device.
    overflow =  imu->xDataOverflow() | 
                imu->gDataOverflow() | 
                imu->mDataOverflow();

    if (overflow)
    {
        cout<<"WARNING: DATA OVERFLOW!!!"<<endl;
    }

    // Calling these functions causes the data to be read from the IMU into
    //    10 16-bit signed integer public variables, as seen below. There is no
    //    automated check on whether the data is new; you need to do that
    //    manually as above. Also, there's no check on overflow, so you may miss
    //    a sample and not know it.
    imu->readAccel();
    imu->readMag();
    imu->readGyro();
    imu->readTemp();

    // Print the "real" values in more human comprehensible units.

    float acceX = imu->ax;
    float acceY = imu->ay;
    float acceZ = imu->az;
    // float gyroX = imu->calcGyro(imu->gx);
    // float gyroY = imu->calcGyro(imu->gy);
    // float gyroZ = imu->calcGyro(imu->gz);

    //cout<<"-------------------------------------"<<endl;
    // cout<<"Gyro x: "  << gyroX << endl;
    // cout<<"Gyro y: "  << gyroY << endl;
    // cout<<"Gyro z: "  << gyroZ << endl;

    // cout<<"Accel x: " << acceX <<" g" << endl;
    // cout<<"Accel y: " << acceY <<" g" << endl;
    // cout<<"Accel z: " << acceZ <<" g" << endl;

    // cout<<"Mag x: "<<imu->calcMag(imu->mx)<<" Gauss"<<endl;
    // cout<<"Mag y: "<<imu->calcMag(imu->my)<<" Gauss"<<endl;
    // cout<<"Mag z: "<<imu->calcMag(imu->mz)<<" Gauss"<<endl;
    
    cout << "----------- acce = " << sqrt(acceX * acceX + acceY * acceY + acceZ * acceZ) << "\n";

    if (++ntime == 10)
    {
        ntime = 0;
        a[n++] = sqrt(acceX * acceX + acceY * acceY + acceZ * acceZ);
    }
}



float   max_of_interval(int* a, int left, int right)
{
    if (left < 0)
        left = 0;
    if (right >= n)
        right = n - 1;

    float max = a[left];
    for (int i = left + 1; i <= right; i++)
        if (max < a[i])
            max = a[i];
    
    return max;
}

/*
---------------------------------------------------
input: 
    a    :  array of raw accelerometer values
    n    :  number of elements in array a
output:
    total:  number of steps recognized
---------------------------------------------------
init:
    p     : empty array
    step  = 0
    delta = 5

    for i in range(delta, n):
        if a[i] == max_of_interval(a[i - delta], a[i + delta]):
            p[i] = 1
        i++

    k = 0
    D = 0
    for j in range(0, n):
        if p[j] == 1:
            if k == 0:
                D = j - k - 1
                if D > 2:
                    step++
        j++
    if j == n:
        D = n - k
        if D > 2:
            step++
    
    total = step

    return total

*/
float   calculate()
{
    int*    local_maxima = new int[n];

    // set up array local_maxima
    for (int i = 0 + DELTA; i < n - DELTA; i++)
        if (a[i] == max_of_interval(a, i - DELTA, i + DELTA))
            local_maxima[i] = 1;
        else
            local_maxima[i] = 0;

    int k    = 0;
    int D    = 0;
    int step = 0;
    int j    = 0;
    for (; j < n; j++)
    {
        if (local_maxima[j] == 1){
            if (k != 0)
            {
                D = j - k - 1;
                cout << "D = " << D << " and j = " << j << " and k = " << k << "\n";
                if (D > 2)
                    step++;
            }
            k = j;
        }
    }
    if (j == n)
    {
        D = n - k;
        cout << "D = " << D << "\n";
        if (D > 2)
            step++;
    }

    // delete everything
    delete[] local_maxima;
    delete[] a;
    n = 0;

    return step;
}