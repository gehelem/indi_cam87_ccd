 /*
    Cam87 DIY CCD Camera Library
    See :
    http://www.astroclub.kiev.ua/forum/index.php?topic=28929.0
    http://www.cloudynights.com/topic/497530-diy-astro-ccd-16-bit-color-6mpx-camera/
    http://astroccd.org/

    Copyright (C) 2017 - Gilles Le Mar�chal - Gilmanov Rim - Sergiy Vakulenko - Michael "Toups"

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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ftdi.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include "libcam87.h"
#include "config.h"



const int   CameraWidth  = 3000;  //image width
const int   CameraHeight = 2000;  //image height
const uint8_t portfirst  = 0x11;  //Initial value on the output port BDBUS
const int  xccd   = 1500;
const int  yccd   = 1000;

bool isConnected  = false;    //variable-flag indicates the status of the connection with the camera
int  adress;        //pointer to the current address in the output buffer FT2232HL
int  mBin;        //Binning
bool imageReady  = false;        //variable-flag displays readiness for reading frame
int  mCameraState = 0;     //Variable-state camera  0 - ready 1 - longexp 2 - read
int  ExposureTimer;      //exposure timer
//      co: posl;                           // Variable for the second stream (image reading)
static uint16_t bufim[3000][2000];       // Buffer array image for operations
//static uint16_t bufim2[30000][20000];
int  mYn,mdeltY;       //start reading and the number of the rows
//int  mXn,mdeltX;      //start reading and number of the columns
uint8_t zatv;
int kolbyte;
int eexp;
uint8_t siin[4];
uint16_t siout;
double durat;
int ftdi_result;
static uint8_t FT_In_Buffer[260000000];
int  FT_Current_Baud;
bool FT_OP_flag;
bool FT_flag;
struct ftdi_context *CAM8A, *CAM8B;
int32_t   dwBytesRead  = 0;
double spusb=20000;
double ms1;
double coolerPowerCache;
const int TRUE_INV_PROT = 0xaa55;
const int FALSE_INV_PROT = 0x55aa;
const int HIGH_MASK_PROT = 0xaa00;
int cameraState = 0;
int cameraIdle = 0;
int cameraWaiting = 1;
int cameraExposing = 2;
int cameraReading = 3;
int cameraDownload = 4;
int cameraError = 5;
bool errorWriteFlag = false;
bool errorReadFlag = false;
pthread_t te;
//cached values
double sensorTempCache = 0;
double targetTempCache = 0;
bool coolerOnCache = false;
double coolerPowerCache  = 0;
int firmwareVersionCache = 0;
double tempDHTCache  = -128.0;
double humidityDHTCache =  -1;
int CoolingStartingPowerPercentageCache = -1;
int CoolingMaximumPowerPercentageCache = 101;
double KpCache  = 0.0;


// timer counter
// timer can only count <1000s
// for longer exposures repeat the timer run as needed
int exposure_time_left;
int eposure_time_rollover = 900000; // 900 seconds (999 seconds is max)
int exposure_time_loop_counter;

// used when 0s exposure image is taken to clear the sensor before real exposure
bool sensorClear;

int softwareLLDriverVersion = 91;

/*A little explanation with FT2232LH.
�Always use this technique:
��1. First, the buffer is filled and the initial bytes (required pulse sequence BDBUS output port).
This pointer is incremented adress.
��2. Next, the whole array is passed to the output of the command: n: = Write_USB_Device_Buffer (FT_CAM8B, adress);
Wonderful chip FT2232HL honestly without delay all transfers to your port BDBUS. Transfer 1 byte at the same time takes 65 ns.
Time working out the next command n: = Write_USB_Device_Buffer (FT_CAM8B, adress) depends on the workload of the OSes and is not controlled
us. Therefore, the critical sequence of pulses need to fill the whole, rather than pass on the queue.
Fortunately Programnyj driver buffer it allows (in the program up to 24 MB!) To do this, change the text D2XX.pas, I called him MyD2XX.pas*/

/*int Qbuf(void)
{
 Get_USB_Device_QueueStatus(CAM8A);
 return FT_Q_Bytes;
}*/

int ftdi_read_data_modified ( struct ftdi_context *ftdi, unsigned char *buf, int size )
{
    fprintf(stderr,"--ftdi_read_data_modified\n");
    const int uSECPERSEC = 1000000;
    const int uSECPERMILLISEC = 1000;

    int offset;
    int result;
// Sleep interval, 1 microsecond
    struct timespec tm;
    tm.tv_sec = 0;
    tm.tv_nsec = 1000L;
// Read timeout
    struct timeval startTime;
    struct timeval timeout;
    struct timeval now;

    gettimeofday ( &startTime, NULL );
// usb_read_timeout in milliseconds
// Calculate read timeout time of day
    timeout.tv_sec = startTime.tv_sec + ftdi->usb_read_timeout / uSECPERMILLISEC;
    timeout.tv_usec = startTime.tv_usec + ( ( ftdi->usb_read_timeout % uSECPERMILLISEC ) *uSECPERMILLISEC );
    if ( timeout.tv_usec >= uSECPERSEC ) {
        timeout.tv_sec++;
        timeout.tv_usec -= uSECPERSEC;
    }

    offset = 0;
    result = 0;

    while ( size > 0 ) {
        result = ftdi_read_data ( ftdi, buf+offset, size );
        if ( result < 0 ) {
            fprintf ( stderr,"Read failed -- error (%d))\n",result );
            break;
        }
        if ( result == 0 ) {
            gettimeofday ( &now, NULL );
            if ( now.tv_sec > timeout.tv_sec || ( now.tv_sec == timeout.tv_sec && now.tv_usec > timeout.tv_usec ) ) {
                fprintf ( stderr,"Read failed -- timeout %d \n",offset );
                break;
            }
            nanosleep ( &tm, NULL ); //sleep for 1 microsecond
            continue;
        }
        size -= result;
        offset += result;
    }
    return offset;
}

void sspi ( void )
{
    fprintf(stderr,"--sspi\n");

    int i,j;
    uint8_t b;
    uint16_t n;
    static uint8_t bufi[128];
    n = 128;
    memset ( bufi,portfirst,n );
    for ( j = 0; j <= 2; j++ ) {
        b=siin[j];
        for ( i = 0; i <= 7; i++ ) {
            bufi[2*i+1+16*j]+=0x20;
            if ( ( ( b & 0x80 ) ==0x80 ) ) {
                bufi[2*i+0+16*j]+=0x80;
                bufi[2*i+1+16*j]+=0x80;
            }
            //b=b*2;
            b = (uint8_t)(b << 1);
        }
    }
    if ( !errorWriteFlag ) {
        if ( ftdi_write_data ( CAM8B, bufi, n ) < 0 ) {
            fprintf ( stderr,"write failed on channel B)\n" );
            errorWriteFlag = true;
        }
        //fprintf ( stderr,"--ftdi_write_data : ");
        //for (int k=0;k<128;k++) fprintf ( stderr,"%X",bufi[k]);
        //fprintf ( stderr,"--ftdi_write_data end\n");

    }
}

void sspo ( void )
{
    fprintf(stderr,"--sspo\n");
    int i;
    int16_t b,n;
    uint16_t byteCnt, byteExpected;
    uint16_t buf;
    static uint8_t bufo[80];


    n=80;
    byteCnt = 0;
    byteExpected = n;
    if ( !errorWriteFlag ) {
        byteCnt = ftdi_read_data ( CAM8B,bufo,n );
        //fprintf ( stderr,"--ftdi_read_data : ");
        //for (int k=0;k<80;k++) fprintf ( stderr,"%X",bufo[k]);
        //fprintf ( stderr,"--ftdi_read_data end\n");

    }
    if ( byteCnt != byteExpected ) {
       fprintf ( stderr,"--sspo : error read byte %i %i\n",byteCnt,byteExpected );
        errorReadFlag = true;
    }
    fprintf ( stderr,"--sspo byteCnt (%d)\n",byteCnt);
    b=0;
    for ( i = 0; i <= 15; i++ ) {
        b = (uint16_t)(b << 1);
        if ((bufo[2 * i + 16 + 2] & 0x40) != 0) b++;
    }
    siout=b;
    fprintf ( stderr,"--sspo siout (%04X)\n",siout);

}

void Spi_comm ( uint8_t comm, uint16_t param )
{
    //fprintf(stderr,"--Spi_comm\n");
    fprintf ( stderr,"--Spi_comm param (%04X) comm (%02X)\n",param,comm );
    //ftdi_tciflush ( CAM8B );
    ftdi_tciflush(CAM8B);
    //ftdi_tcoflush ( CAM8B );
    ftdi_tcoflush(CAM8B);
    siin[0]=comm;
    siin[1]= ( param & 0xFF00 ) >> 8;
    siin[2]= ( param & 0x00FF );
    sspi();
    sspo();
    //usleep ( 20*1000 );
}
void reset ( void )
{
    fprintf(stderr,"--reset\n");

    static uint8_t buf[6000];
    memset ( buf,portfirst,6000 );
    for (int i = 0; i < 2000; i++) buf[i + 2000] = 0x01;

    ftdi_tciflush( CAM8B);
    ftdi_tcoflush( CAM8B);

    if ( !errorWriteFlag ) {
        if ( ftdi_write_data ( CAM8B, buf, 6000 ) < 0 ) {
            fprintf ( stderr,"write failed on channel B)\n" );
            errorWriteFlag = true;
        }
    }
}

uint16_t swap ( uint16_t x )
{
    uint16_t ret;
    uint8_t hibyte = ( x & 0xFF00 ) >> 8;
    uint8_t lobyte = ( x & 0x00FF );
    ret= ( lobyte << 8 ) | hibyte;
    return ret;
}

/*Contraption FT2232HL converting the read buffer to the buffer array image
�� due to the nature AD9822 read the high byte first, then low, and vice versa in delphi.
�� Use the type integer32, instead word16 due to overflow in subsequent operations*/
void posExecute ( void *arg ) // Array itself actually reading through ADBUS port
{
    fprintf ( stderr,"--posExecute\n" );
    uint16_t byteCnt, byteExpected;
    byteCnt = 0;
    byteExpected = kolbyte;
    uint16_t x,y;
    struct ftdi_transfer_control *tc_read;
    //usleep(500*1000);
    byteCnt=ftdi_read_data_modified ( CAM8A,FT_In_Buffer,kolbyte );

    if ( byteCnt!=byteExpected ) {
        errorReadFlag=true;
        fprintf ( stderr,"--posExecute : error read byte %i %i\n",byteCnt,byteExpected );
        if ( !errorWriteFlag ) {
            ftdi_tciflush( CAM8A);
            ftdi_tcoflush( CAM8A);
            for ( y=0; y < 2000; y++ ) {
                for ( x=0; x <3000; x++ ) {
                    bufim[x][y]= 0;
                }
            }
        }
    } else {
        fprintf ( stderr,"--posExecute : read OK %i %i\n",byteCnt,byteExpected );

        if ( mBin == 0 ) {
            for ( y=0; y < mdeltY; y++ ) {
                for ( x=0; x < 1500; x++ ) {
                    bufim[2*x+0][ ( 2* ( y+mYn ) +0 ) *1]= 256* ( FT_In_Buffer[8*x+0+y*12000] )+ FT_In_Buffer[8*x+1+y*12000];
                    bufim[2*x+0][ ( 2* ( y+mYn ) +1 ) *1]= 256* ( FT_In_Buffer[8*x+2+y*12000] )+ FT_In_Buffer[8*x+3+y*12000];
                    bufim[2*x+1][ ( 2* ( y+mYn ) +1 ) *1]= 256* ( FT_In_Buffer[8*x+4+y*12000] )+ FT_In_Buffer[8*x+5+y*12000];
                    bufim[2*x+1][ ( 2* ( y+mYn ) +0 ) *1]= 256* ( FT_In_Buffer[8*x+6+y*12000] )+ FT_In_Buffer[8*x+7+y*12000];
                }

            }
        } else {
            for ( y=0; y <= mdeltY-1; y++ ) {
                for ( x=0; x < 1500; x++ ) {
                    bufim[2*x+0][ ( 2* ( y+mYn ) +0 )]= FT_In_Buffer[2* ( x+7+y*1500 )] + 256*FT_In_Buffer[2* ( x+7+y*1500 )] ;
                    bufim[2*x+0][ ( 2* ( y+mYn ) +1 )]= FT_In_Buffer[2* ( x+7+y*1500 )] + 256*FT_In_Buffer[2* ( x+7+y*1500 )] ;
                    bufim[2*x+1][ ( 2* ( y+mYn ) +1 )]= FT_In_Buffer[2* ( x+7+y*1500 )] + 256*FT_In_Buffer[2* ( x+7+y*1500 )] ;
                    bufim[2*x+1][ ( 2* ( y+mYn ) +0 )]= FT_In_Buffer[2* ( x+7+y*1500 )] + 256*FT_In_Buffer[2* ( x+7+y*1500 )] ;
                }
            }
        }
    }
    ftdi_tciflush( CAM8A);
    ftdi_tcoflush( CAM8A);
    // discard image if sensorClearing was required (Bias frame before exposure)
    if ( sensorClear ) {
        imageReady = false;
        sensorClear=false;
    } else {
        imageReady=true;
    }
    cameraState=cameraIdle;

    ( void ) arg;
    //pthread_exit ( NULL );
    return;
}

/*Filling the output buffer array for transmission and placing byte val at the address adr-chip AD9822.
� The transfer is in sequential code.*/
void AD9822 ( uint8_t adr,uint16_t val )
{
    fprintf ( stderr,"--AD9822\n" );
    int kol = 64;
    uint8_t dan[kol];
    int i;
    memset ( dan,portfirst,kol );
    for ( i = 1; i <= 32; i++ )  {
        dan[i]=dan[i] & 0xFE;
    };
    for ( i = 0; i <= 15; i++ )  {
        dan[2*i+2]=dan[2*i+2] +2 ;
    };

    if ( ( adr & 4 ) ==4 )       {
        dan[3] =dan[3]+4;
        dan[4] =dan[4]+4;
    };
    if ( ( adr & 2 ) ==2 )       {
        dan[5] =dan[5]+4;
        dan[6] =dan[6]+4;
    };
    if ( ( adr & 1 ) ==1 )       {
        dan[7] =dan[7]+4;
        dan[8] =dan[8]+4;
    };

    if ( ( val & 256 ) ==256 )   {
        dan[15]=dan[15]+4;
        dan[16]=dan[16]+4;
    };
    if ( ( val & 128 ) ==128 )   {
        dan[17]=dan[17]+4;
        dan[18]=dan[18]+4;
    };
    if ( ( val &  64 ) ==64 )    {
        dan[19]=dan[19]+4;
        dan[20]=dan[20]+4;
    };
    if ( ( val &  32 ) ==32 )    {
        dan[21]=dan[21]+4;
        dan[22]=dan[22]+4;
    };
    if ( ( val &  16 ) ==16 )    {
        dan[23]=dan[23]+4;
        dan[24]=dan[24]+4;
    };
    if ( ( val &   8 ) ==8 )     {
        dan[25]=dan[25]+4;
        dan[26]=dan[26]+4;
    };
    if ( ( val &   4 ) ==4 )     {
        dan[27]=dan[27]+4;
        dan[28]=dan[28]+4;
    };
    if ( ( val &   2 ) ==2 )     {
        dan[29]=dan[29]+4;
        dan[30]=dan[30]+4;
    };
    if ( ( val &   1 ) ==1 )     {
        dan[31]=dan[31]+4;
        dan[32]=dan[32]+4;
    };
    if ( !errorWriteFlag ) {
        if ( ftdi_write_data ( CAM8B, dan, sizeof ( dan ) ) < 0 ) {
            fprintf ( stderr,"write failed on channel B)\n" );
            errorWriteFlag=true;
        }
    }
}

/*Use 2 modes:
� 1.Tsvetnoy without binning.
� 2.CH / B with 2 * 2 binning.
� Feature ICX453 matrix is that the horizontal register has twice the capacity and
� at one step in a horizontal vertical shift register "falls" just a couple of lines,
� so the number of rows for the two modes are similar.*/

//Filling the output buffer array and the actual frame itself in the read operation mode 1
void readframe ( void )
{
    fprintf ( stderr,"--readframe\n" );

    cameraState = cameraReading;
    ftdi_tciflush ( CAM8A );
    ftdi_tcoflush ( CAM8A );
    //comread();
    Spi_comm ( 0x1B,0 ); //$ffff
    void *arg;
    posExecute(arg);
    //pthread_t t1;
    //pthread_create ( &t1, NULL, posExecute, NULL );
    //pthread_detach ( t1 );
    //pthread_join ( t1,NULL );
    //fprintf ( stderr,"--readframe -- Done !\n" );
}

/*Set camera gain, return bool result*/
bool cameraSetGain ( int val )
{
    fprintf ( stderr,"---------------------------cameraSetGain %i\n",val );
    Spi_comm(0xE0, val);
    //AD9822 ( 3,val );
    return true;
}

/*Set camera offset, return bool result*/
bool cameraSetOffset ( int val )
{
    fprintf ( stderr,"---------------------------cameraSetOffset %i\n",val );
    int x;

    x=abs ( val );
    if ( val < 0 )  x=x+256;
    Spi_comm(0xE1, (uint16_t)x);
    //AD9822 ( 6,x );
    return true;

}

/*Connect camera, return bool result}
Survey attached devices and initialize AD9822*/
bool cameraConnect()
{
    fprintf ( stderr,"--cameraConnect\n" );

    FT_flag=true;
    errorWriteFlag=false;
    sensorTempCache = 0;
    targetTempCache = 0;
    coolerOnCache = false;
    coolerPowerCache = 0;
    firmwareVersionCache = 0;

    CAM8A = ftdi_new();
    CAM8B = ftdi_new();
    if ( ftdi_set_interface ( CAM8A, INTERFACE_A ) <0 ) fprintf ( stderr,"libftdi error set interface A\n" );
    if ( ftdi_set_interface ( CAM8B, INTERFACE_B ) <0 ) fprintf ( stderr,"libftdi error set interface B\n" );
    if ( FT_flag ) {
        if ( ftdi_usb_open ( CAM8A, 0x0403, 0x6010 ) <0 ) {
            fprintf ( stderr,"libftdi error open interface A\n" );
            FT_flag=false;
        }
    }
    if ( FT_flag ) {
        if ( ftdi_usb_open ( CAM8B, 0x0403, 0x6010 ) <0 ) {
            fprintf ( stderr,"libftdi error open interface B\n" );
            FT_flag=false;
        }
    }

// BitBang channel 2
    if ( FT_flag ) {
        if ( ftdi_set_bitmode ( CAM8B, 0xBF, BITMODE_SYNCBB ) <0 ) { //BITMODE_BITBANG / BITMODE_SYNCBB
            fprintf ( stderr,"libftdi error set bitbang mode interface B\n" );
            FT_flag=false;
        }
    }


// Baudrate
    /*if ( FT_flag ) {
        if (!cameraSetBaudrateA ( BRA ) ) {
            fprintf ( stderr,"libftdi error set baudrate interface A\n" );
            FT_flag=false;
        }
    }*/
    if ( FT_flag ) {
        if (!cameraSetBaudrateB ( 20 ) ) {
            fprintf ( stderr,"libftdi error set baudrate interface B\n" );
            FT_flag=false;
        }
    }
//    ftdi_result=ftdi_set_baudrate ( CAM8B,120000 );

    if ( FT_flag ) {
        if ( ftdi_set_latency_timer ( CAM8A,CAM87_LATENCYA ) <0 ) fprintf ( stderr,"libftdi error set latency interface A\n" );
        if ( ftdi_set_latency_timer ( CAM8B,CAM87_LATENCYB ) <0 ) fprintf ( stderr,"libftdi error set latency interface B\n" );;

//timeouts
        CAM8A->usb_read_timeout=4000;
        CAM8B->usb_read_timeout=100;
        CAM8A->usb_write_timeout=100;
        CAM8B->usb_write_timeout=100;
        //ftdi_read_data_set_chunksize(CAM8A,65536);
        ftdi_read_data_set_chunksize ( CAM8A, 1<<14 );
        fprintf ( stderr,"libftdi BRA=%d BRB=%d TA=%d TB=%d CSA=%d \n",CAM8A->baudrate,CAM8B->baudrate,CAM8A->usb_read_timeout,CAM8B->usb_write_timeout,CAM8A->readbuffer_chunksize );


//Purge

        if ( ftdi_tciflush ( CAM8A ) <0 ) fprintf ( stderr,"libftdi error purge RX interface A\n" );
        if ( ftdi_tcoflush ( CAM8A ) <0 ) fprintf ( stderr,"libftdi error purge TX interface A\n" );
        if ( ftdi_tciflush ( CAM8B ) <0 ) fprintf ( stderr,"libftdi error purge RX interface B\n" );
        if ( ftdi_tcoflush ( CAM8B ) <0 ) fprintf ( stderr,"libftdi error purge TX interface B\n" );
        reset();
        usleep ( 200*1000 );
        //cameraSetGain ( 0 );      // Set a gain. that is not full of ADC
        //cameraSetOffset ( 0 );
        fprintf(stderr,"b\n");

        //usleep ( 100*1000 );

        // Remove the 2 bytes that have arisen after the reset
        if ( ftdi_tciflush ( CAM8A ) <0 ) fprintf ( stderr,"libftdi error purge RX interface A\n" );
        if ( ftdi_tcoflush ( CAM8A ) <0 ) fprintf ( stderr,"libftdi error purge TX interface A\n" );
        mBin=0;
    }
    isConnected = FT_flag;
    errorReadFlag=false;
    cameraState = cameraIdle;
    imageReady=false;

    if ( !FT_flag ) {
        cameraState=cameraError;
    }
    fprintf(stderr,"999\n");

    return isConnected;
}

/*Disconnect camera, return bool result*/
bool cameraDisconnect ( void )
{
    bool FT_OP_flag = true;

    ftdi_disable_bitbang ( CAM8B );

    if ( FT_OP_flag ) {
        if ( ftdi_usb_close ( CAM8B ) <0 ) FT_OP_flag=false;
    }
    if ( FT_OP_flag ) ftdi_free ( CAM8B );

    if ( FT_OP_flag ) {
        if ( ftdi_usb_close ( CAM8A ) <0 ) FT_OP_flag=false;
    }
    if ( FT_OP_flag ) ftdi_free ( CAM8A );

    isConnected=!FT_OP_flag;
    return FT_OP_flag;
}

void *ExposureTimerTick ( void *arg )
{
    fprintf ( stderr,"--ExposureTimerTick\n" );
    uint32_t dd;
    dd = ( durat*1000 ) *1000;
    //usleep ( dd );
    fprintf ( stderr,"--ExposureTimerTick : Tick !\n" );
    //Spi_comm ( 0xcb,0 ); //clear frame
    //usleep ( 1000*100 );
    readframe();
    ( void ) arg;
    pthread_exit ( NULL );
}

/*Check camera connection, return bool result*/
bool cameraIsConnected()
{
    return isConnected;
}

void cameraSensorClearFull ( void )
{
    fprintf ( stderr,"--cameraSensorClearFull\n" );

    int expoz;
    errorReadFlag = false;
    imageReady = false;
    mYn=0;
    Spi_comm ( 0x4b,mYn );
    mdeltY=CameraHeight / 2;
    Spi_comm ( 0x5b,mdeltY );

    // use 2x2 binning to increase the reading speed
    // the image will be deleted anyway
    kolbyte=mdeltY*3000;
    //bining
    Spi_comm ( 0x8b,1 );
    mBin=1;

    expoz = 0; // zero exposure
    Spi_comm ( 0x6b,expoz );

    cameraState = cameraExposing;

    eexp=0;
    readframe;

    // wait until the bias frame has been read - we will discard the data
    // This will lock this main thread for a short time... not sure if this is a good thing?
    // this seems to take 1600 ms
    while ( sensorClear ) {
        //usleep ( 10*1000 );
    }

    // now exit to do proper exposure
}

int cameraStartExposure ( int bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light )
{
    fprintf ( stderr,"--cameraStartExposure bin %d x %d y %d w %d h %d s %f l %d\n",bin,StartX,StartY,NumX,NumY,Duration,light );
    //if ( sensorClear ) cameraSensorClearFull;
    uint8_t d0,d1;
    uint16_t expoz;

    errorReadFlag = false;
    imageReady = false;

    mYn = StartY / 2;
    Spi_comm ( 0x4B,mYn );
    mdeltY = NumY / 2;
    Spi_comm ( 0x5B,mdeltY );
//     fprintf ( stderr,"--cameraStartExposure A1\n" );

    if ( bin==2 ) {
        kolbyte=mdeltY*3000;
        Spi_comm ( 0x8B,1 ); //bining
        mBin=1;
    } else {
        kolbyte=mdeltY*12000;
        Spi_comm ( 0x8B,0 ); //no bining
        mBin=0;
    }
//     fprintf ( stderr,"--cameraStartExposure A2\n" );
    expoz=Duration*1000;
    durat=Duration;
    Spi_comm ( 0x6B,expoz );
//     fprintf ( stderr,"--cameraStartExposure A3\n" );

    //camera exposing
    cameraState = cameraExposing;
    if ( expoz > 500 ) {
//          fprintf ( stderr,"--cameraStartExposure B1\n" );
        //Spi_comm ( 0x2B,0 ); //shift3
        //usleep ( 40*1000 );
        //Spi_comm ( 0xCB,0 ); //clear frame
        //usleep ( 180*1000 ); //for time of clear frame
        Spi_comm ( 0x3B,0 ); //off 15v
        eexp= ( 1000* ( Duration-1.2 ) );
        //pthread_create ( &te, NULL, ExposureTimerTick, NULL );
        //pthread_detach ( te );
    } else {
//          fprintf ( stderr,"--cameraStartExposure C1\n" );
        //eexp=0;
        //readframe();
    }
    return true;
}

/*Stop camera exposure when it is possible*/
bool cameraStopExposure()
{
    // "code to kill Exposuretimer thread"
    pthread_kill ( te,0 );
    if ( cameraState==cameraExposing ) readframe();
    return true;
}

//Get camera state, return int result
int  cameraGetCameraState()
{
    if ( !errorWriteFlag ) {
        return cameraState;
    } else {
        return cameraError;
    }
}

bool CameraSetTemp ( float temp )
{
    uint16_t d0;
    fprintf ( stderr,"--CameraSetTemp\n" );
    fprintf ( stderr,"--target =  %.2f\n",temp );
    d0=(uint16_t)16*temp;
    Spi_comm ( 0xAB,d0 );
    return true;
}

float cameraGetSetTemp ()
{
    fprintf ( stderr,"--cameraGetSetTemp\n" );

    float temp;

    Spi_comm ( 0xbe,0 );
    temp = siout *0.0625;
    if ( ( temp > 120 ) || ( temp < -120 ) ) {
        temp = targetTempCache;
    }
    targetTempCache = temp;
    return temp;
}

float CameraGetTemp ( void )
{
    fprintf ( stderr,"--CameraGetTemp\n" );

    Spi_comm ( 0xBF,0 );
    //fprintf ( stderr,"--CameraGetTemp %d \n",siout );
    return (( float ) ( siout )) *0.0625;
}
int cameraGetGain ( void )
{

    fprintf ( stderr,"---------------------------cameraGetGain \n");
    Spi_comm ( 0xBC,0 );
    fprintf ( stderr,"---------------------------cameraGetGain result %i \n",siout );
    return (uint16_t)siout;
}


bool CameraCoolingOn ( void )
{
    fprintf ( stderr,"--CameraCoolingOn\n" );

    Spi_comm ( 0x9B,1 );
    return true;
}

bool CameraCoolingOff ( void )
{
    fprintf ( stderr,"--CameraCoolingOff\n" );
    Spi_comm ( 0x9B,0 );
    return true;
}

uint16_t cameraGetCoolerOn ( void )
{

        Spi_comm ( 0xbd,0 );
        return (uint16_t)siout;
}

uint16_t cameraGetImageXY ( int i,int j )
{
    cameraState=cameraIdle;
    return bufim[i][j];
}

//Get back pointer to image
char *cameraGetImage()
{
    cameraState=cameraDownload;
    cameraState=cameraIdle;
    return 0;//*bufim;
}

/*Check ImageReady flag, is image ready for transfer - transfer image to driver and return bool ImageReady flag*/
bool cameraGetImageReady()
{
    return imageReady;
}

/*Set camera baudrate, return bool result*/
bool cameraSetBaudrate ( int val )
{
    // fprintf ( stderr,"gettemp (%f)\n", ( float ) CameraGetTemp() /80 );
    return true;
}

bool cameraSetBaudrateA ( int val )
{
    bool Result;
    /*setup FT2232 baud rate channel B*/
    if ( ( val>=1 ) & ( val<=150 ) ) {
        ftdi_result=ftdi_set_baudrate ( CAM8A,val*1000 );
        if ( ftdi_result<0 ) {
            fprintf ( stderr,"libftdi error set baud interface A (%d:%s)\n",ftdi_result,ftdi_get_error_string ( CAM8A ) );
        }
        fprintf ( stderr,"libftdi BRA=%d WTA=%d RTA=%d\n",CAM8A->baudrate,CAM8A->usb_write_timeout,CAM8A->usb_read_timeout );
        Result = true;
    } else {
        Result = false;
    }
    return Result;

}

bool cameraSetBaudrateB ( int val )
{
    bool Result;
    /*setup FT2232 baud rate channel B*/
    if ( ( val>=1 ) & ( val<=150 ) ) {
        ftdi_result=ftdi_set_baudrate ( CAM8B,val*1000/4 );
        if ( ftdi_result<0 ) {
            fprintf ( stderr,"libftdi error set baud interface B (%d:%s)\n",ftdi_result,ftdi_get_error_string ( CAM8B ) );
        }
        fprintf ( stderr,"libftdi BRB=%d WTB=%d RTB=%d\n",CAM8B->baudrate,CAM8B->usb_write_timeout,CAM8B->usb_read_timeout );
        Result = true;
    } else {
        Result = false;
    }
    return Result;
}

bool cameraSetLibftdiTimerAR ( int tt )
{
    CAM8A->usb_read_timeout=tt;
    return true;
}

bool cameraSetLibftdiTimerAW ( int tt )
{
    CAM8A->usb_write_timeout=tt;
    return true;
}

bool cameraSetLibftdiTimerBR ( int tt )
{
    CAM8B->usb_read_timeout=tt;
    return true;
}

bool cameraSetLibftdiTimerBW ( int tt )
{
    CAM8B->usb_write_timeout=tt;
    return true;
}

bool cameraSetLibftdiLatA ( int ll )
{
    if ( ftdi_set_latency_timer ( CAM8A,ll ) <0 ) {
        fprintf ( stderr,"libftdi error set latency interface A\n" );
    }
    return true;
}

bool cameraSetLibftdiLatB ( int ll )
{
    if ( ftdi_set_latency_timer ( CAM8B,ll ) <0 ) {
        fprintf ( stderr,"libftdi error set latency interface B\n" );
    }
    return true;
}
//Get camera error state, return bool result
int cameraGetError()
{
    int res=0;
    if ( errorWriteFlag ) res =res+2;
    if ( errorReadFlag )  res =res+1;
    return res;
}

bool cameraSetReadingTime ( int val )
{
    //Spi_comm ( 0xEB,val );
    return true;
}

bool cameraSetCoolerDuringReading ( bool val )
{
    Spi_comm ( 0xFB,!val );
    return true;
}
float cameraGetCoolerPower ( void )
{
    double power;
    Spi_comm ( 0xBA,0 );
    power = 0.390625 * (255 - siout);
    return power;
}

int cameraGetFirmwareVersion()
{
    if ( ( cameraState == cameraReading ) || ( cameraState == cameraDownload ) ) {
        return firmwareVersionCache;
    } else {
        Spi_comm ( 0xbb,0 );
        firmwareVersionCache = siout && 0xff;
        return firmwareVersionCache;
    }
}

int cameraGetLLDriverVersion ()
{
    return softwareLLDriverVersion;
}

