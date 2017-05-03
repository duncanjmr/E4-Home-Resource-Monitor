/*
    E4 Main Design Project
    Home Resource Monitor
    Duncan Rocha, Rebecca Wroblewski, Matthew Calligaro, Isaac Zinda
*/
#ifndef HRM_h
#define HRM_h

#include "Arduino.h"

#define DHTLIB_OK				0
#define DHTLIB_ERROR_CHECKSUM	-1
#define DHTLIB_ERROR_TIMEOUT	-2


class HRM
{
  public:
    HRM(void);

    // Functions
    void windflowTempUpdate(int analogPinForRV, int analogPinForTMP);
    void windspeedUpdate(int analogPinForRV, int analogPinForTMP);
    int tempUpdate(int pin);
    void sonarUpdate(void);
    void initializeSonar(int Trig, int Echo, int Max_Dist);
    void PIRupdate(int S);


    // Variables:
    float temp;
    float windowDist;
    float windspeed;
    float windflowTemp;
    int humidity;
    bool motion;

  private:
    //float temp;
    float zeroWindAdjustment;
};


#endif