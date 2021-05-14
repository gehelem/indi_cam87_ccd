/*
    Cam87 DIY CCD Camera Library
    See :
    http://www.astroclub.kiev.ua/forum/index.php?topic=28929.0
    http://www.cloudynights.com/topic/497530-diy-astro-ccd-16-bit-color-6mpx-camera/
    http://astroccd.org/

    Copyright (C) 2017 - Gilles Le Maréchal - Gilmanov Rim - Sergiy Vakulenko - Michael "Toups"

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>

*/
#ifndef __LIBCAM87_H__
#define __LIBCAM87_H__

#define BRA 200
#define BRB 20
#define CAM87_LATENCYA  2
#define CAM87_LATENCYB  2
#define CAM87_TIMERA    4000
#define CAM87_TIMERB    100

#ifdef __cplusplus
extern "C" {
#endif
void readframe ( void );
bool cameraConnect ( void );
bool cameraDisconnect ( void );
bool cameraSetGain ( int val );
int  cameraGetGain ( void);
bool cameraSetOffset ( int val );
int  cameraStartExposure ( int bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light );
bool cameraStopExposure ( void );
bool CameraSetTemp ( float temp );
float CameraGetTemp ( void );
float CameraGetTempDHT ( void );
float CameraGetHum ( void );
bool CameraCoolingOn ( void );
bool CameraCoolingOff ( void );
bool cameraGetImageReady ( void );
uint16_t cameraGetImageXY ( int i, int j );
char *cameraGetImage(void);
bool cameraSetBaudrate ( int val );
bool cameraSetBaudrateA ( int val );
bool cameraSetBaudrateB ( int val );
bool cameraSetLibftdiTimerAR ( int tt );
bool cameraSetLibftdiTimerAW ( int tt );
bool cameraSetLibftdiTimerBR ( int tt );
bool cameraSetLibftdiTimerBW ( int tt );
bool cameraSetLibftdiLatA ( int ll );
bool cameraSetLibftdiLatB ( int ll );
bool cameraSetCoolingStartingPowerPercentage(int val);
bool cameraSetCoolingMaximumPowerPercentage(int val);
float cameraGetSetTemp (void);
bool cameraSetReadingTime(int val);
bool cameraSetCoolerDuringReading(bool val);
uint16_t cameraGetCoolerOn(void);
float cameraGetCoolerPower(void);
int  cameraGetCameraState(void);
int cameraGetError(void);
int cameraGetFirmwareVersion(void);
int cameraGetLLDriverVersion (void);
bool cameraSetBiasBeforeExposure(bool val);
int cameraGetCoolingStartingPowerPercentage (void);
int cameraGetCoolingMaximumPowerPercentage (void);
bool cameraSetPIDproportionalGain(float val);
double cameraGetPIDproportionalGain (void);



#ifdef __cplusplus
}
#endif

#endif
