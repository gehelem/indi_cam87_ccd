/*
   INDI Developers Manual
   Tutorial #3

   "Cam87 Driver"

   We develop a Cam87 driver.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simpleccd.cpp
    \brief Construct a basic INDI CCD device that simulates exposure & temperature settings. It also generates a random pattern and uploads it as a FITS file.
    \author Jasem Mutlaq

    \example simpleccd.cpp
    A Cam87 device that can capture images and control temperature. It returns a FITS image to the client. To build drivers for complex CCDs, please
    refer to the INDI Generic CCD driver template in INDI SVN (under 3rdparty).
*/

#include <sys/time.h>
#include <memory>
#include <cmath>
#include <unistd.h>
#include "cam87_ccd.h"
#include "libcam87.h"


//#define POLLMS           300        /* Polling interval 500 ms */
#define MAX_CCD_TEMP      45		/* Max CCD temperature */
#define MIN_CCD_TEMP	 -55		/* Min CCD temperature */
#define TEMP_THRESHOLD  0.25		/* Differential temperature threshold (C)*/
#define TEMPERATURE_UPDATE_FREQ 10      /* Update every 40 POLLMS ~ 20 second */

/* Macro shortcut to CCD temperature value */
#define currentCCDTemperature   TemperatureN[0].value
#define LIBFTDI_TAB     "LibFTDI"
#define COOLER_TAB     "Cooler"

std::unique_ptr<Cam87CCD> simpleCCD ( new Cam87CCD() );

void ISGetProperties ( const char *dev )
{
    fprintf ( stderr,"*** ISGetProperties\n" );
    simpleCCD->ISGetProperties ( dev );
}

void ISNewSwitch ( const char *dev, const char *name, ISState *states, char *names[], int num )
{
    fprintf ( stderr,"*** ISNewSwitch\n" );
    simpleCCD->ISNewSwitch ( dev, name, states, names, num );

}

void ISNewText (	const char *dev, const char *name, char *texts[], char *names[], int num )
{
    fprintf ( stderr,"*** ISNewText\n" );
    simpleCCD->ISNewText ( dev, name, texts, names, num );
}

void ISNewNumber ( const char *dev, const char *name, double values[], char *names[], int num )
{
    fprintf ( stderr,"*** ISNewNumber\n" );
    simpleCCD->ISNewNumber ( dev, name, values, names, num );
}

void ISNewBLOB ( const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n )
{
    fprintf ( stderr,"*** ISNewBLOB\n" );
    INDI_UNUSED ( dev );
    INDI_UNUSED ( name );
    INDI_UNUSED ( sizes );
    INDI_UNUSED ( blobsizes );
    INDI_UNUSED ( blobs );
    INDI_UNUSED ( formats );
    INDI_UNUSED ( names );
    INDI_UNUSED ( n );
}

void ISSnoopDevice ( XMLEle *root )
{
    fprintf ( stderr,"*** ISSnoopDevice\n" );
    simpleCCD->ISSnoopDevice ( root );
}

Cam87CCD::Cam87CCD()
{
    fprintf ( stderr,"*** new cam87 driver instance \n" );

    InExposure = false;
}

/*******************************************************************************
** Client is asking us to set a new number
*******************************************************************************/
bool Cam87CCD::ISNewNumber ( const char *dev, const char *name,
                             double values[], char *names[], int n )
{
    fprintf ( stderr,"DRV newnumber %s\n",name );
    if ( !strcmp ( dev, getDeviceName() ) )
    {
        if ( !strcmp ( name, GainNP.name ) )
        {
            IUUpdateNumber ( &GainNP, values, names, n );
            GainNP.s = IPS_OK;
            IDSetNumber ( &GainNP, NULL );
            cameraSetGain ( GainN[0].value );
            IDMessage ( getDeviceName(), "Cam87 set gain = %d", ( int ) GainN[0].value );
            return true;
        }

        if ( !strcmp ( name, OffsetNP.name ) )
        {
            IUUpdateNumber ( &OffsetNP, values, names, n );
            OffsetNP.s = IPS_OK;
            IDSetNumber ( &OffsetNP, NULL );
            cameraSetOffset ( OffsetN[0].value );
            IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set offset = %d", ( int ) OffsetN[0].value );
            return true;
        }

        /*if ( !strcmp ( name, BaudrateNP.name ) )
          {
            IUUpdateNumber ( &BaudrateNP, values, names, n );
            BaudrateNP.s = IPS_OK;
            IDSetNumber ( &BaudrateNP, NULL );
            cameraSetBaudrate ( BaudrateN[0].value );
            //IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set baudrate = %d", ( int ) BaudrateN[0].value );
            return true;
          }*/

        if ( !strcmp ( name, BaudrateANP.name ) )
        {
            IUUpdateNumber ( &BaudrateANP, values, names, n );
            BaudrateANP.s = IPS_OK;
            IDSetNumber ( &BaudrateANP, NULL );
            cameraSetBaudrateA ( BaudrateAN[0].value );
            IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set baudrate A = %d", ( int ) BaudrateAN[0].value );
            return true;
        }

        if ( !strcmp ( name, BaudrateBNP.name ) )
        {
            IUUpdateNumber ( &BaudrateBNP, values, names, n );
            BaudrateBNP.s = IPS_OK;
            IDSetNumber ( &BaudrateBNP, NULL );
            cameraSetBaudrateB ( BaudrateBN[0].value );
            IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set baudrate B = %d", ( int ) BaudrateBN[0].value );
            return true;
        }
        if ( !strcmp ( name, LibftditimerANP.name ) )
        {
            IUUpdateNumber ( &LibftditimerANP, values, names, n );
            LibftditimerANP.s = IPS_OK;
            IDSetNumber ( &LibftditimerANP, NULL );
            cameraSetLibftdiTimerAR ( LibftditimerAN[0].value );
            cameraSetLibftdiTimerAW ( LibftditimerAN[0].value );
            IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set libftdi timerA = %d", ( int ) LibftditimerAN[0].value );
            return true;
        }

        if ( !strcmp ( name, LibftdilatencyANP.name ) )
        {
            IUUpdateNumber ( &LibftdilatencyANP, values, names, n );
            LibftdilatencyANP.s = IPS_OK;
            IDSetNumber ( &LibftdilatencyANP, NULL );
            cameraSetLibftdiLatA ( LibftdilatencyAN[0].value );
            IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set libftdi latencyA = %d", ( int ) LibftdilatencyAN[0].value );
            return true;
        }
        if ( !strcmp ( name, LibftditimerBNP.name ) )
        {
            IUUpdateNumber ( &LibftditimerBNP, values, names, n );
            LibftditimerBNP.s = IPS_OK;
            IDSetNumber ( &LibftditimerBNP, NULL );
            cameraSetLibftdiTimerBR ( LibftditimerBN[0].value );
            cameraSetLibftdiTimerBW ( LibftditimerBN[0].value );
            IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set libftdi timerB = %d", ( int ) LibftditimerBN[0].value );
            return true;
        }

        if ( !strcmp ( name, LibftdilatencyBNP.name ) )
        {
            IUUpdateNumber ( &LibftdilatencyBNP, values, names, n );
            LibftdilatencyBNP.s = IPS_OK;
            IDSetNumber ( &LibftdilatencyBNP, NULL );
            cameraSetLibftdiLatA ( LibftdilatencyBN[0].value );
            IDMessage ( INDI::CCD::getDeviceName(), "Cam87 set libftdi latencyA = %d", ( int ) LibftdilatencyBN[0].value );
            return true;
        }

    }

    // If we didn't process anything above, let the parent handle it.
    return INDI::CCD::ISNewNumber ( dev,name,values,names,n );
}

/*******************************************************************************
** Client is asking us to set a new switch
*******************************************************************************/

bool Cam87CCD::ISNewSwitch ( const char *dev, const char *name,
                             ISState *states, char *names[], int n )
{
    fprintf ( stderr,"DRV newswitch %s\n",name );
    INDI::CCD::ISNewSwitch(dev,name,states,names,n);
    if ( strcmp ( dev,getDeviceName() ) ==0 )
    {


        /* Cooler */
        if ( !strcmp ( name, CoolerSP.name ) )
        {
            if ( IUUpdateSwitch ( &CoolerSP, states, names, n ) < 0 ) return false;

            if ( CoolerS[0].s == ISS_ON )
            {
                //cameraSetCoolingStartingPowerPercentage ( 60 );
                //cameraSetCoolingMaximumPowerPercentage ( 100 );
                //cameraSetPIDproportionalGain ( 0.3 );
                //cameraSetCoolerDuringReading(true);
                CameraCoolingOn();
            }
            else
                CameraCoolingOff();

            return true;
        }


    }

    return INDI::CCD::ISNewSwitch ( dev, name, states, names,  n );
}


/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool Cam87CCD::Connect()
{

    fprintf ( stderr,"DRV Connect\n" );

    // Let's set a timer that checks teleCCDs status every POLLMS milliseconds.
    SetTimer(getCurrentPollingPeriod());

    //cameraSetBaudrate(80);
    if ( cameraConnect() )
    {
        //cameraSetBaudrateA ( BRA );
        cameraSetBaudrateB ( BRB );
        usleep ( 200*1000 );
        LOG_INFO ( "Cam87 connected successfully" );
        cameraSetOffset ( -20 );
        cameraSetGain ( 0 );
        ;
        //cameraSetReadingTime ( 10 );
        return true;
    }
    else
    {
        LOG_INFO ( "Cam87 connection error" );
        return false;

    }

}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool Cam87CCD::Disconnect()
{

    cameraDisconnect();
    return true;
    IDMessage ( INDI::CCD::getDeviceName(), "Cam87 disconnected successfully!\n" );
}


/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char * Cam87CCD::getDefaultName()
{
    return "Cam87";
}

/**************************************************************************************
** INDI is asking us to init our properties.
***************************************************************************************/
bool Cam87CCD::initProperties()
{
    // Must init parent properties first!
    INDI::CCD::initProperties();

    /*const short minGain = 0;
    const short maxGain = 63;
    const short minOffset = -127;
    const short maxOffset = 127;
    const short minBaudrate = 80;
    const short maxBaudrate = 240;*/

    /* Add Gain number property (gs) */
    IUFillNumber ( GainN, "GAIN", "Gain", "%g", 0, 63, 1, 0 );
    IUFillNumberVector ( &GainNP, GainN, 1, INDI::CCD::getDeviceName(),"GAIN",
                         "Gain", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE );

    /* Add Offset number property (gs) */
    IUFillNumber ( OffsetN, "OFFSET", "Offset", "%g", -255, 255, 10, 0 );
    IUFillNumberVector ( &OffsetNP, OffsetN, 1, INDI::CCD::getDeviceName(),"OFFSET",
                         "Offset", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE );

    /* Add Baudrate number property (gs) */
//  IUFillNumber ( BaudrateN, "BAUDRATE", "Baudrate", "%g", 5, 150, 5, 20 );
//  IUFillNumberVector ( &BaudrateNP, BaudrateN, 1, INDI::CCD::getDeviceName(),"BAUDRATE",
//                       "Baudrate", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

    /* Add Baudrate A number property (gs) */
    IUFillNumber ( BaudrateAN, "BAUDRATEA", "BaudrateA", "%g", 5, 150, 5, 20 );
    IUFillNumberVector ( &BaudrateANP, BaudrateAN, 1, INDI::CCD::getDeviceName(),"BAUDRATEA",
                         "BaudrateA", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

    /* Add Baudrate B number property (gs) */
    IUFillNumber ( BaudrateBN, "BAUDRATEB", "BaudrateB", "%g", 5, 150, 5, 5 );
    IUFillNumberVector ( &BaudrateBNP, BaudrateBN, 1, INDI::CCD::getDeviceName(),"BAUDRATEB",
                         "BaudrateB", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

    /* Add Latency number property (gs) */
    IUFillNumber ( LibftdilatencyAN, "LATENCYA", "LatencyA", "%g", 1, 50, 1, CAM87_LATENCYA );
    IUFillNumberVector ( &LibftdilatencyANP, LibftdilatencyAN, 1, INDI::CCD::getDeviceName(),"LATENCYA",
                         "LatencyA", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

    /* Add timers number property (gs) */
    IUFillNumber ( LibftditimerAN, "TIMERA", "TimerA", "%g", 10, 20000, 10, CAM87_TIMERA );
    IUFillNumberVector ( &LibftditimerANP, LibftditimerAN, 1, INDI::CCD::getDeviceName(),"TIMERA",
                         "TimerA", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

    /* Add Latency number property (gs) */
    IUFillNumber ( LibftdilatencyBN, "LATENCYB", "LatencyB", "%g", 1, 50, 1, CAM87_LATENCYB );
    IUFillNumberVector ( &LibftdilatencyBNP, LibftdilatencyBN, 1, INDI::CCD::getDeviceName(),"LATENCYB",
                         "LatencyB", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

    /* Add timers number property (gs) */
    IUFillNumber ( LibftditimerBN, "TIMERB", "TimerB", "%g",  10, 20000, 10, CAM87_TIMERB );
    IUFillNumberVector ( &LibftditimerBNP, LibftditimerBN, 1, INDI::CCD::getDeviceName(),"TIMERB",
                         "TimerB", LIBFTDI_TAB, IP_RW, 0, IPS_IDLE );

    IUFillSwitch ( &CoolerS[0], "COOLER_ON", "ON", ISS_OFF );
    IUFillSwitch ( &CoolerS[1], "COOLER_OFF", "OFF", ISS_ON );
    IUFillSwitchVector ( &CoolerSP, CoolerS, 2, INDI::CCD::getDeviceName(), "CCD_COOLER", "Cooler", COOLER_TAB, IP_WO, ISR_1OFMANY, 0, IPS_IDLE );

    IUFillNumber ( &CoolerN[0], "CCD_COOLER_VALUE", "Cooling Power (%)", "%+06.2f", 0., 1., .2, 0.0 );
    IUFillNumberVector ( &CoolerNP, CoolerN, 1, INDI::CCD::getDeviceName(), "CCD_COOLER_POWER", "Cooling Power", COOLER_TAB, IP_RO, 60, IPS_IDLE );





    // We set the CCD capabilities
    uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_BAYER | CCD_HAS_COOLER;
    //uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_COOLER;
    SetCCDCapability ( cap );
    //IUSaveText ( &BayerT[2], "RGGB" );
    //IUSaveText ( &BayerT[2], "BGGR" );
    //IUSaveText ( &BayerT[2], "GBRG" );
    IUSaveText ( &BayerT[2], "GRBG" );
    // Add Debug, Simulator, and Configuration controls
    addAuxControls();

    return true;

}

/********************************************************************************************
** INDI is asking us to update the properties because there is a change in CONNECTION status
** This fucntion is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool Cam87CCD::updateProperties()
{
    //cameraSetBaudrate(80);
    // Call parent update properties first
    INDI::CCD::updateProperties();

    if ( isConnected() )
    {
        // Let's get parameters now from CCD
        setupParams();

        // Start the timer
        setCurrentPollingPeriod(300);
        SetTimer(getCurrentPollingPeriod());
        defineProperty ( &GainNP );
        defineProperty ( &OffsetNP );
        //defineNumber ( &BaudrateNP );
        defineProperty ( &BaudrateANP );
        defineProperty ( &BaudrateBNP );
        defineProperty ( &LibftditimerANP );
        defineProperty ( &LibftdilatencyANP );
        defineProperty ( &LibftditimerBNP );
        defineProperty ( &LibftdilatencyBNP );
        defineProperty ( &CoolerSP );
        defineProperty ( &CoolerNP );
    }
    else
    {
        deleteProperty ( GainNP.name );
        deleteProperty ( OffsetNP.name );
        //deleteProperty ( BaudrateNP.name );
        deleteProperty ( BaudrateANP.name );
        deleteProperty ( BaudrateBNP.name );
        deleteProperty ( LibftditimerANP.name );
        deleteProperty ( LibftdilatencyANP.name );
        deleteProperty ( LibftditimerBNP.name );
        deleteProperty ( LibftdilatencyBNP.name );
        deleteProperty ( CoolerSP.name );
        deleteProperty ( CoolerNP.name );

    }

    return true;
}

/**************************************************************************************
** Setting up CCD parameters
***************************************************************************************/
void Cam87CCD::setupParams()
{

    fprintf ( stderr,"DRV setupParams\n" );



    // Our CCD is an 8 bit CCD, 1280x1024 resolution, with 5.4um square pixels.
    SetCCDParams ( 3000, 2000, 16, 7.8, 7.8 );

    // Let's calculate how much memory we need for the primary CCD buffer
    int nbuf;
    nbuf=PrimaryCCD.getXRes() *PrimaryCCD.getYRes() * PrimaryCCD.getBPP() /8;
    nbuf+=512;                      //  leave a little extra at the end
    PrimaryCCD.setFrameBufferSize ( nbuf );

    //bool coolerOn=(cameraGetCoolerPower()>0);
    bool coolerOn=(cameraGetCoolerOn() !=0);

    CoolerS[0].s = coolerOn ? ISS_ON : ISS_OFF;
    CoolerS[1].s = coolerOn ? ISS_OFF : ISS_ON;
    CoolerSP.s = IPS_OK;
    IDSetSwitch(&CoolerSP, NULL);

    double temperature;
    temperature = CameraGetTemp();
    DEBUGF(INDI::Logger::DBG_SESSION, "The CCD Temperature is %f.", temperature);

    TemperatureN[0].value = temperature;			/* CCD chip temperatre (degrees C) */
    IDSetNumber(&TemperatureNP, NULL);


}

/**************************************************************************************
** Client is asking us to start an exposure
***************************************************************************************/
bool Cam87CCD::StartExposure ( float duration )
{


    ExposureRequest=duration;
    LOGF_INFO ( "Start exposure %g",duration );
    // Since we have only have one CCD with one chip, we set the exposure duration of the primary CCD
    PrimaryCCD.setExposureDuration ( duration );
    //cameraStartExposure(1,0,0,3000,2000, duration,true);
    //int r = cameraStartExposure ( PrimaryCCD.getBinX(),PrimaryCCD.getSubX(),PrimaryCCD.getSubY(),PrimaryCCD.getSubW(),PrimaryCCD.getSubH(), duration, true );    //int r = cameraStartExposure(1,0,0,3000,2000, 0.4, true);
    int r = cameraStartExposure(1,0,0,3000,2000, duration, true);
    gettimeofday ( &ExpStart,NULL );


    InExposure=true;

    // We're done
    return true;
}

/**************************************************************************************
** Client is asking us to abort an exposure
***************************************************************************************/
bool Cam87CCD::AbortExposure()
{
    InExposure = false;
    cameraStopExposure;
    return true;
}

/**************************************************************************************
** Client is asking us to set a new temperature
***************************************************************************************/
int Cam87CCD::SetTemperature ( double temperature )
{
    fprintf ( stderr,"DRV SetTemperature\n" );
    TemperatureRequest = temperature;

    // If less than 0.1 of a degree, let's just return OK
    if (fabs(temperature - TemperatureN[0].value) < 0.1)
        return 1;

    CameraCoolingOn();
    CameraSetTemp ( (float) temperature );

    // 0 means it will take a while to change the temperature
    return 0;
}

/**************************************************************************************
** How much longer until exposure is done?
***************************************************************************************/
float Cam87CCD::CalcTimeLeft()
{
    fprintf ( stderr,"DRV CalcTimeLeft\n" );

    double timesince;
    double timeleft;
    struct timeval now;
    gettimeofday ( &now,NULL );

    timesince= ( double ) ( now.tv_sec * 1000.0 + now.tv_usec/1000 ) - ( double ) ( ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec/1000 );
    timesince=timesince/1000;

    timeleft=ExposureRequest-timesince;
    return timeleft;
}

/**************************************************************************************
** Main device loop. We check for exposure and temperature progress here
***************************************************************************************/
void Cam87CCD::TimerHit()
{
    long timeleft;
    if ( isConnected() == false )
        return;  //  No need to reset timer if we are not connected anymore

    if ( InExposure )
    {
        timeleft=CalcTimeLeft();

        // Less than a 0.1 second away from exposure completion
        // This is an over simplified timing method, check CCDSimulator and simpleCCD for better timing checks
        if ( timeleft < 0.1 )
        {
            /* We're done exposing */
            IDMessage ( INDI::CCD::getDeviceName(), "Exposure done, downloading image..." );

            // Set exposure left to zero
            PrimaryCCD.setExposureLeft ( 0 );

            // We're no longer exposing...
            InExposure = false;

            /* grab and save image */
            grabImage();

        }
        else
            // Just update time left in client
            PrimaryCCD.setExposureLeft ( timeleft );

    }

    // TemperatureNP is defined in INDI::CCD
    //if ((TemperatureUpdateCounter++ > TEMPERATURE_UPDATE_FREQ) && !InExposure)
    if ( ( TemperatureUpdateCounter++ > TEMPERATURE_UPDATE_FREQ ) )
    {
        TemperatureUpdateCounter = 0;
        currentCCDTemperature = CameraGetTemp();
        float coolerpower = 0; //cameraGetCoolerPower();
        float settemp = 0;  //cameraGetSetTemp();
        uint16_t cooleron = 0;//cameraGetCoolerOn();
        //double c = log ( HUM / 100 ) + a * tempDHT / ( b + tempDHT );
        //dewpoint = b * c / ( a - c );

        LOGF_INFO (  "%s ccdtemp=%.2f°C settemp=%.2f°C CoolerPower=%.2f CoolerOn %i\n" ,INDI::CCD::getDeviceName(), currentCCDTemperature,settemp,coolerpower,cooleron);
        TemperatureN[0].value =currentCCDTemperature;

    }

    switch ( TemperatureNP.s )
    {
    case IPS_IDLE:
    case IPS_OK:
        //currentCCDTemperature = CameraGetTemp();
        if ( fabs ( currentCCDTemperature - TemperatureN[0].value ) >= TEMP_THRESHOLD )
        {
            TemperatureN[0].value = currentCCDTemperature;
            IDSetNumber ( &TemperatureNP, NULL );
        }
        break;

    case IPS_BUSY:
        //TemperatureN[0].value = CameraGetTemp();
        if (fabs(TemperatureN[0].value - TemperatureRequest) <= TEMP_THRESHOLD)
            TemperatureNP.s = IPS_OK;
        IDSetNumber(&TemperatureNP, NULL);

        break;

    case IPS_ALERT:
        break;
    }

    setCurrentPollingPeriod ( 300 );
    SetTimer(getCurrentPollingPeriod());
    return;
}

/**************************************************************************************
** Create a random image and return it to client
***************************************************************************************/
void Cam87CCD::grabImage()
{
    // Let's get a pointer to the frame buffer
    uint8_t * image = PrimaryCCD.getFrameBuffer();
    int width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX()   * ( PrimaryCCD.getBPP() / 8 );
    int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();
    //int width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX() * (PrimaryCCD.getBPP() / 8);
    //int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY() * (PrimaryCCD.getBPP() / 8);
    usleep(100*1000);
    LOGF_INFO( "grabimage width=%d height=%d BPP=%d\n", width/2, height, PrimaryCCD.getBPP() );
    readframe();

    // Fill buffer with random pattern
    //while ( !cameraGetImageReady() ) LOG_INFO("waiting\n"); // waiting image

    if ( PrimaryCCD.getBinX() ==1 )
    {
        for ( int j=PrimaryCCD.getSubY(); j < height + PrimaryCCD.getSubY(); j++ )
            for ( int i=PrimaryCCD.getSubX(); i < ( PrimaryCCD.getSubX() +width ) /2; i++ )
            {
                uint16_t pix = cameraGetImageXY ( i,j );
                uint8_t hibyte = ( pix & 0xff00 ) >> 8;
                uint8_t lobyte = ( pix & 0x00ff );
                image[2*i+  j*width] = lobyte;
                image[2*i+1+j*width] = hibyte;
            };
    }
    else
    {
        for ( int j=0; j < height ; j++ )
            for ( int i=0; i < width/2; i++ )
            {
                uint16_t pix = cameraGetImageXY ( 2*i,2*j );
                uint8_t hibyte = ( pix & 0xff00 ) >> 8;
                uint8_t lobyte = ( pix & 0xff );
                image[2*i+  j*width] = lobyte;
                image[2*i+1+j*width] = hibyte;
            };
    };


    IDMessage ( INDI::CCD::getDeviceName(), "Download complete.\n" );

    // Let INDI::CCD know we're done filling the image buffer
    ExposureComplete ( &PrimaryCCD );
}
void Cam87CCD::activateCooler(bool enable)
{
    fprintf ( stderr,"DRV activateCooler\n" );

    if (enable)
    {
        CameraCoolingOn();
        CoolerS[0].s = ISS_ON;
        CoolerS[1].s = ISS_OFF;
        CoolerSP.s = IPS_OK;
        DEBUG(INDI::Logger::DBG_SESSION, "Cooler ON");
        IDSetSwitch(&CoolerSP, NULL);
        CoolerNP.s = IPS_BUSY;
    }
    else
    {
        CameraCoolingOff();
        CoolerS[0].s = ISS_OFF;
        CoolerS[1].s = ISS_ON;
        CoolerSP.s = IPS_IDLE;
        DEBUG(INDI::Logger::DBG_SESSION, "Cooler is OFF.");
        IDSetSwitch(&CoolerSP, NULL);
    }
}

