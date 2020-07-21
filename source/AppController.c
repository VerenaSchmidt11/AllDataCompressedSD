/*----------------------------------------------------------------------------*/

/* --------------------------------------------------------------------------- |
 * INCLUDES & DEFINES ******************************************************** |
 * -------------------------------------------------------------------------- */

/* own header files */
#include "XdkAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_APP_CONTROLLER

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "BCDS_CmdProcessor.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "XDK_NoiseSensor.h"
#include "math.h"
#include "arm_math.h"

/* additional interface header files */
#include "XDK_Storage.h"
#include "XDK_LED.h"
#include "BCDS_SDCard_Driver.h"

#include "XDK_Utils.h"
#include "BSP_BoardType.h"
#include "BCDS_Assert.h"
#include <FreeRTOS.h>
#include <timers.h>

#include "AppController.h"



#include "BCDS_CmdProcessor.h"
#include "FreeRTOS.h"
#include "BCDS_SDCard_Driver.h"
#include "ff.h"



#include "XDK_Sensor.h"

#include "BCDS_CmdProcessor.h"

#include "task.h"



// Accelerator

#include "XdkSensorHandle.h"

// Battery Level
#include "BatteryMonitor.h"


/* constant definitions ***************************************************** */
#define APP_TEMPERATURE_OFFSET_CORRECTION   (-3459)



/* constant definitions MQTT / WLAN***************************************************** */




//#define APP_MQTT_DATA_BUFFER_SIZE                   UINT32_C(2048)/**< macro for data size of incoming subscribed and published messages */
#define MQTT_ACC_Send_Data                   UINT32_C(4)/** 10 Data message per time */





/* local variables ********************************************************** */

static CmdProcessor_T *AppCmdProcessor;/**< Handle to store the main Command processor handle to be reused by ServalPAL thread */

static xTimerHandle sensorHandle = NULL;

xTimerHandle accelerometerHandle = NULL;

/* local variables WLAN********************************************************** */


/* --------------------------------------------------------------------------- |
 * HANDLES ******************************************************************* |
 * -------------------------------------------------------------------------- */

static CmdProcessor_T * AppCmdProcessor;/**< Handle to store the main Command processor handle to be used by run-time event driven threads */

static xTaskHandle AppControllerHandle = NULL;/**< OS thread handle for Application controller to be used by run-time blocking threads */


/* --------------------------------------------------------------------------- |
 * VARIABLES ***************************************************************** |
 * -------------------------------------------------------------------------- */

#define DEFAULT_LOGICAL_DRIVE   ""
#define DRIVE_ZERO              UINT8_C(0)
#define FORCE_MOUNT             UINT8_C(1)
#define FIRST_LOCATION          UINT8_C(0)

static FIL fileObject;




/* --------------------------------------------------------------------------- |
 * HANDLES ******************************************************************* |
 * -------------------------------------------------------------------------- */

static CmdProcessor_T * AppCmdProcessor;/**< Handle to store the main Command processor handle to be used by run-time event driven threads */
xTimerHandle acousticHandle = NULL;

/* --------------------------------------------------------------------------- |
 * VARIABLES ***************************************************************** |
 * -------------------------------------------------------------------------- */

const float aku340ConversionRatio = pow(10,(-38/20));

const float SplRatio = pow(10,(-38/20)) * 20e-6;

static arm_rfft_instance_f32 rfft_instance;

static arm_cfft_radix4_instance_f32 cfft_instance;

/** Complex (interleaved) output from FFT. */
static float32_t* fftOutputComplex;

/** Magnitude of complex numbers in FFT output. */
static float32_t* fftOutputMag;

#define SAMPLING_FREQUENCY UINT32_C(100) //1000 = Sek



#define ACOUSTIC_BUFFER_SIZE  UINT32_C(32)

static float32_t acousticSensorBuffer[ACOUSTIC_BUFFER_SIZE];

unsigned int counter = 0;
const char FilePrefix[] = "sensordata_";
const char FileType[]=".csv";
unsigned int FileCounter=0;
unsigned int DataCounter=0;

// Acceleration
unsigned int AccCounter=0;
static float32_t ACC_X_Buffer[MQTT_ACC_Send_Data];
static float32_t ACC_Y_Buffer[MQTT_ACC_Send_Data];
static float32_t ACC_Z_Buffer[MQTT_ACC_Send_Data];
static float32_t ACC_X_Buffer_mean;
static float32_t ACC_Y_Buffer_mean;
static float32_t ACC_Z_Buffer_mean;
static long int Gyro_X_Buffer[MQTT_ACC_Send_Data];
static long int Gyro_Y_Buffer[MQTT_ACC_Send_Data];
static float Gyro_Z_Buffer[MQTT_ACC_Send_Data];
static long int Gyro_X_mean;
static long int Gyro_Y_mean;
static float Gyro_Z_mean;
static int Timer_Buffer[MQTT_ACC_Send_Data];
static long int Pressure_Buffer[MQTT_ACC_Send_Data];
static long int Pressure_Buffer_mean;
static float Noise_Buffer[MQTT_ACC_Send_Data];
static float Noise_Buffer_mean;


uint64_t sntpTimeStampFromServer = 0UL;
static int Timer_zero;



/*
 *   Sensor  Variable*
 */


static Sensor_Setup_T SensorSetup =
        {
                .CmdProcessorHandle = NULL,
                .Enable =
                        {
                                .Accel = true,
                                .Mag = true,
                                .Gyro = true,
                                .Humidity = true, //false, //true,
                                .Temp = true, //false, //true,
                                .Pressure = true, //false, //true,
                                .Light = true, //false, //true,
                                .Noise = false,
                        },
                .Config =
                        {
                                .Accel =
                                        {
                                                .Type = SENSOR_ACCEL_BMA280,
                                                .IsRawData = false,
                                                .IsInteruptEnabled = false,

                                        },
                                .Gyro =
                                        {
                                                .Type = SENSOR_GYRO_BMG160,
                                                .IsRawData = false,
                                        },
                                .Mag =
                                        {
                                                .IsRawData = false,
                                        },
                                .Light =
                                        {
                                                .IsInteruptEnabled = false,

                                        },
                                .Temp =
                                        {
                                                .OffsetCorrection = APP_TEMPERATURE_OFFSET_CORRECTION,
                                        },
                        },
        };/**< Sensor setup parameters */


// Function for ACC


static void readAccelerometer(xTimerHandle xTimer)
{
    (void) xTimer;

    Retcode_T returnValue = RETCODE_FAILURE;

    /* read and print BMA280 accelerometer data */

    Accelerometer_XyzData_T bma280 = {INT32_C(0), INT32_C(0), INT32_C(0)};
    memset(&bma280, 0, sizeof(CalibratedAccel_XyzMps2Data_T));

    returnValue = Accelerometer_readXyzGValue(xdkAccelerometers_BMA280_Handle,&bma280);

    if (RETCODE_OK == returnValue) {
        printf("BMA280 Acceleration Data - M/S2 %f m/s2 %f m/s2 %f m/s2\n\r",
            (float) bma280.xAxisData, (float) bma280.yAxisData, (float) bma280.zAxisData);
    }
}


static void initAccelerometer(void)
{
    Retcode_T returnValue = RETCODE_FAILURE;
    Retcode_T returnBandwidthValue = RETCODE_FAILURE;
    Retcode_T returnRangeValue = RETCODE_FAILURE;

    /* initialize accelerometer */

    returnValue = Accelerometer_init(xdkAccelerometers_BMA280_Handle);

    if ( RETCODE_OK != returnValue) {
        printf("BMA280 Accelerometer initialization failed\n\r");
    }

    returnBandwidthValue = Accelerometer_setBandwidth(xdkAccelerometers_BMA280_Handle,ACCELEROMETER_BMA280_BANDWIDTH_125HZ);

    if (RETCODE_OK != returnBandwidthValue) {
        printf("Configuring bandwidth failed \n\r");
    }
    returnRangeValue = Accelerometer_setRange(xdkAccelerometers_BMA280_Handle,ACCELEROMETER_BMA280_RANGE_2G);

    if (RETCODE_OK != returnRangeValue) {
        printf("Configuring range failed \n\r");
    }
}





/* local functions ********************************************************** */
/*


/* -----
 * Filemanagement
 */




/* --------------------------------------------------------------------------- |
 * EXECUTING FUNCTIONS ******************************************************* |
 * -------------------------------------------------------------------------- */

void InitSdCard(void){
  Retcode_T retVal = RETCODE_FAILURE;
  FRESULT FileSystemResult = FR_OK;
  static FATFS FatFileSystemObject;
  SDCardDriver_Initialize();
  if(SDCARD_INSERTED == SDCardDriver_GetDetectStatus()){
    retVal = SDCardDriver_DiskInitialize(DRIVE_ZERO);
    if(RETCODE_OK == retVal){
      printf("SD Card Disk initialize succeeded \n\r");
      FileSystemResult = f_mount(&FatFileSystemObject, DEFAULT_LOGICAL_DRIVE,
                                    FORCE_MOUNT);
      if (FR_OK != FileSystemResult){
        printf("Mounting SD card failed \n\r");
      }
    }
  }
}

Retcode_T searchForFileOnSdCard(const char* filename, FILINFO* fileData){
  if(FR_OK == f_stat(filename, fileData)){
    printf("File %s found on SD card. \n\r",filename);
    return RETCODE_OK;
  } else {
    printf("File %s does not exist. \n\r",filename);
    return RETCODE_FAILURE;
  }
}



void createFileOnSdCard(const char* filename){
  if(FR_OK == f_open(&fileObject, filename, FA_CREATE_NEW)){
    printf("File %s was created successfully \n\r",filename);
  }
}

void writeDataIntoFileOnSdCard(const char* filename, const char* dataBuffer){
    FRESULT fileSystemResult;
    char ramBufferWrite[UINT16_C(512)]; // Temporay buffer for write file
    uint16_t fileSize;
    UINT bytesWritten;
    fileSize = (uint16_t) strlen(dataBuffer);

    for(uint32_t index = 0; index < fileSize; index++){
        ramBufferWrite[index] = dataBuffer[index];
    }
    f_open(&fileObject, filename, FA_OPEN_EXISTING | FA_WRITE);
    f_lseek(&fileObject, f_size(&fileObject));
    fileSystemResult = f_write(&fileObject, ramBufferWrite, fileSize, &bytesWritten);

    if((fileSystemResult != FR_OK) || (fileSize != bytesWritten)){
        printf("Error: Cannot write to file %s \n\r", filename);
    }
    fileSystemResult = f_close(&fileObject);
}



static void AppControllerFire(void* pvParameters)
{
    BCDS_UNUSED(pvParameters);
// File

  TickType_t  current_timer;


    float acousticData, sp, spl;
    double si;
    char FileContent[550] ;
    const char Filename[100];
    Retcode_T returnValue = RETCODE_FAILURE;
    uint64_t Timer_absolut;
    uint32_t 	outputVoltage;



// MQTT
    Retcode_T retcode = RETCODE_OK;
    Sensor_Value_T sensorValue;


    memset(&sensorValue, 0x00, sizeof(sensorValue));







    if (RETCODE_OK != retcode)
    {
        /* We raise error and still proceed to publish data periodically */
        Retcode_RaiseError(retcode);
        vTaskSuspend(NULL);
    }

         sprintf(Filename,"%s%i%s",FilePrefix,FileCounter,FileType);












    /* A function that implements a task must not exit or attempt to return to
     its caller function as there is nothing to return to. */
    while (1)
    {







    	current_timer=xTaskGetTickCount();


        retcode = RETCODE_OK;



		if (RETCODE_OK == retcode)
		{
			retcode = Sensor_GetData(&sensorValue);
		}

// // Sensor File



Accelerometer_XyzData_T bma280 = {INT32_C(0), INT32_C(0), INT32_C(0)};
		     memset(&bma280, 0, sizeof(CalibratedAccel_XyzMps2Data_T));

		     //current_timer=xTaskGetTickCount();
		     returnValue = Accelerometer_readXyzGValue(xdkAccelerometers_BMA280_Handle,&bma280);

            // Dann Noise




	retcode = NoiseSensor_ReadRmsValue(&acousticData,10U);





	 if (AccCounter < MQTT_ACC_Send_Data)
			     {
		 	 	 	 if (AccCounter == 0)  //Reset Mean
		 	 	 	 {
		 	 	  	   ACC_X_Buffer_mean=0;
		 	 				    	         ACC_Y_Buffer_mean=0;
		 	 				    	         ACC_Z_Buffer_mean=0;
		 	 				    	       Gyro_X_mean=0;
		 	 				    	     Gyro_Y_mean=0;
		 	 				    	   Gyro_Z_mean=0;

		 	 				    	         Pressure_Buffer_mean=0;
		 	 				    	         Noise_Buffer_mean=0;
		 	 	 	 }

			     	ACC_X_Buffer[AccCounter]=bma280.xAxisData;
			     	ACC_Y_Buffer[AccCounter]=bma280.yAxisData;
			     	ACC_Z_Buffer[AccCounter]=bma280.zAxisData;
			     	Timer_Buffer[AccCounter]=current_timer;


			     	ACC_X_Buffer_mean=ACC_X_Buffer_mean+bma280.xAxisData;
			     	ACC_Y_Buffer_mean=ACC_Y_Buffer_mean+bma280.yAxisData;
			     	ACC_Z_Buffer_mean=ACC_Z_Buffer_mean+bma280.zAxisData;


			     	Gyro_X_Buffer[AccCounter]=sensorValue.Gyro.X;
			     	Gyro_Y_Buffer[AccCounter]=sensorValue.Gyro.Y;
			     	Gyro_Z_Buffer[AccCounter]=sensorValue.Gyro.Z;

			     	Gyro_X_mean=Gyro_X_mean+Gyro_X_Buffer[AccCounter];
			     	Gyro_Y_mean=Gyro_Y_mean+Gyro_Y_Buffer[AccCounter];
			     	Gyro_Z_mean=Gyro_Z_mean+Gyro_Z_Buffer[AccCounter];


			     	Pressure_Buffer_mean=Pressure_Buffer_mean+ sensorValue.Pressure;
			     	 Noise_Buffer_mean= Noise_Buffer_mean+acousticData;

			     	//Noise_Buffer[AccCounter]=acousticData;
			     	AccCounter++;

			     }
			     else

			     {
			    	 AccCounter=0;

			    	 Pressure_Buffer_mean=Pressure_Buffer_mean/MQTT_ACC_Send_Data;

			    	 ACC_X_Buffer_mean=ACC_X_Buffer_mean/MQTT_ACC_Send_Data;
			    	 ACC_Y_Buffer_mean=ACC_Y_Buffer_mean/MQTT_ACC_Send_Data;
			    	 ACC_Z_Buffer_mean=ACC_Z_Buffer_mean/MQTT_ACC_Send_Data;
			    	 Noise_Buffer_mean=Noise_Buffer_mean/MQTT_ACC_Send_Data;


			    	 Timer_absolut=sntpTimeStampFromServer+(Timer_Buffer[3]-Timer_zero)/1000;
			    	retcode= BatteryMonitor_MeasureSignal(&outputVoltage);




		if (RETCODE_OK == retcode)
		{



/*

			sprintf(FileContent, "%ld ,%.0lf,%.0lf,%.0lf, %ld, %.0lf,%.0lf,%.0lf, %ld, %.0lf,%.0lf,%.0lf,"
					"%ld, %.0lf, %.0lf,%.0lf,%ld, %.0lf, %.0lf,%.0lf,%ld,%ld,%ld,%ld,%ld, %ld,%ld,%ld,"
         				 "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
      		"%ld, %ld, %ld, %lf,%ld,%ld,%0.2f,%ld,%ld,%ld\n",
					(int) Timer_Buffer[0], (float)ACC_X_Buffer[0], (float) ACC_Y_Buffer[0], (float) ACC_Z_Buffer[0],
																				(int) Timer_Buffer[1], (float)ACC_X_Buffer[1], (float) ACC_Y_Buffer[1], (float) ACC_Z_Buffer[1],
																				(int) Timer_Buffer[2], (float)ACC_X_Buffer[2], (float) ACC_Y_Buffer[2], (float) ACC_Z_Buffer[2],
																				(int) Timer_Buffer[3], (float)ACC_X_Buffer[3], (float) ACC_Y_Buffer[3], (float) ACC_Z_Buffer[3],
																				(int) Timer_Buffer[3], (float)ACC_X_Buffer_mean, (float) ACC_Y_Buffer_mean, (float) ACC_Z_Buffer_mean,


																				(int) Timer_Buffer[0],(long int)Gyro_X_Buffer[0], (long int) Gyro_Y_Buffer[0], (long int) Gyro_Z_Buffer[0],
																				(int) Timer_Buffer[1], (long int)Gyro_X_Buffer[1], (long int) Gyro_Y_Buffer[1], (long int)Gyro_Z_Buffer[1],
																				(int) Timer_Buffer[2], (long int)Gyro_X_Buffer[2], (long int) Gyro_Y_Buffer[2], (long int) Gyro_Z_Buffer[2],
																				(int) Timer_Buffer[3], (long int)Gyro_X_Buffer[3], (long int) Gyro_Y_Buffer[3], (long int) Gyro_Z_Buffer[3],
																				(int) Timer_Buffer[3], (long int)Gyro_X_mean, (long int) Gyro_Y_mean, (long int) Gyro_Z_mean,

																				(long int) sensorValue.Mag.X, (long int) sensorValue.Mag.Y,  (long int) sensorValue.Mag.Z,
																					acousticData,(long int) sensorValue.RH, (long int) Pressure_Buffer_mean,
																				(sensorValue.Temp /= 1000), (long int) sensorValue.Light, (long int) Timer_absolut, outputVoltage );


			writeDataIntoFileOnSdCard(Filename, FileContent);*/

		     /*sprintf(FileContent, "%ld, %.0lf, %.0lf, %.0lf \n ",
		    		 (int) Timer_Buffer[0], (float)ACC_X_Buffer[0], (float) ACC_Y_Buffer[0], (float) ACC_Z_Buffer[0]);*/

			sprintf(FileContent, "%ld ,%.0lf,%.0lf,%.0lf, %ld, %.0lf,%.0lf,%.0lf, %ld, %.0lf,%.0lf,%.0lf,%ld, %.0lf, %.0lf,%.0lf,%ld, %.0lf, %.0lf,%.0lf,%ld,%ld,%ld,%ld,%ld, %ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld, %ld, %ld, %lf,%ld,%ld,%0.2f,%ld,%ld,%ld\n",
								(int) Timer_Buffer[0], (float)ACC_X_Buffer[0], (float) ACC_Y_Buffer[0], (float) ACC_Z_Buffer[0],
																							(int) Timer_Buffer[1], (float)ACC_X_Buffer[1], (float) ACC_Y_Buffer[1], (float) ACC_Z_Buffer[1],
																							(int) Timer_Buffer[2], (float)ACC_X_Buffer[2], (float) ACC_Y_Buffer[2], (float) ACC_Z_Buffer[2],
																							(int) Timer_Buffer[3], (float)ACC_X_Buffer[3], (float) ACC_Y_Buffer[3], (float) ACC_Z_Buffer[3],
																							(int) Timer_Buffer[3], (float)ACC_X_Buffer_mean, (float) ACC_Y_Buffer_mean, (float) ACC_Z_Buffer_mean,


																							(int) Timer_Buffer[0],(long int)Gyro_X_Buffer[0], (long int) Gyro_Y_Buffer[0], (long int) Gyro_Z_Buffer[0],
																							(int) Timer_Buffer[1], (long int)Gyro_X_Buffer[1], (long int) Gyro_Y_Buffer[1], (long int)Gyro_Z_Buffer[1],
																							(int) Timer_Buffer[2], (long int)Gyro_X_Buffer[2], (long int) Gyro_Y_Buffer[2], (long int) Gyro_Z_Buffer[2],
																							(int) Timer_Buffer[3], (long int)Gyro_X_Buffer[3], (long int) Gyro_Y_Buffer[3], (long int) Gyro_Z_Buffer[3],
																							(int) Timer_Buffer[3], (long int)Gyro_X_mean, (long int) Gyro_Y_mean, (long int) Gyro_Z_mean,

																							(long int) sensorValue.Mag.X, (long int) sensorValue.Mag.Y,  (long int) sensorValue.Mag.Z,
																								acousticData,(long int) sensorValue.RH, (long int) Pressure_Buffer_mean,
																							(sensorValue.Temp /= 1000), (long int) sensorValue.Light, (long int) Timer_absolut, outputVoltage );


		/*	sprintf(FileContent, "%ld, %.0lf,%.0lf,%.0lf, %ld, %.0lf,%.0lf,%.0lf,%ld, %.0lf, %.0lf,%.0lf,%ld, %.0lf, %.0lf,%.0lf,%ld,%ld,%ld,%ld,%ld, %ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld, %ld, %ld, %lf,%ld,%ld,%0.2f,%ld,%ld,%ld\n",
											(int) Timer_Buffer[0], (float)ACC_X_Buffer[0], (float) ACC_Y_Buffer[0], (float) ACC_Z_Buffer[0],
																										(int) Timer_Buffer[1], (float)ACC_X_Buffer[1], (float) ACC_Y_Buffer[1], (float) ACC_Z_Buffer[1],
																										(int) Timer_Buffer[2], (float)ACC_X_Buffer[2], (float) ACC_Y_Buffer[2], (float) ACC_Z_Buffer[2],
																										(int) Timer_Buffer[3], (float)ACC_X_Buffer[3], (float) ACC_Y_Buffer[3], (float) ACC_Z_Buffer[3],
																										(int) Timer_Buffer[0],(long int)Gyro_X_Buffer[0], (long int) Gyro_Y_Buffer[0], (long int) Gyro_Z_Buffer[0],
																										(int) Timer_Buffer[1], (long int)Gyro_X_Buffer[1], (long int) Gyro_Y_Buffer[1], (long int)Gyro_Z_Buffer[1],
																										(int) Timer_Buffer[2], (long int)Gyro_X_Buffer[2], (long int) Gyro_Y_Buffer[2], (long int) Gyro_Z_Buffer[2],
																										(int) Timer_Buffer[3], (long int)Gyro_X_Buffer[3], (long int) Gyro_Y_Buffer[3], (long int) Gyro_Z_Buffer[3],
																										(int) Timer_Buffer[3], (long int)Gyro_X_mean, (long int) Gyro_Y_mean, (long int) Gyro_Z_mean,

																										(long int) sensorValue.Mag.X, (long int) sensorValue.Mag.Y,  (long int) sensorValue.Mag.Z,
																											acousticData,(long int) sensorValue.RH, (long int) Pressure_Buffer_mean,
																										(sensorValue.Temp /= 1000), (long int) sensorValue.Light, (long int) Timer_absolut, outputVoltage );

*/
		                     writeDataIntoFileOnSdCard(Filename, FileContent);



		}
			     }



        if (RETCODE_OK != retcode)
        {
            Retcode_RaiseError(retcode);
        }
        DataCounter++;
        vTaskDelay(APP_MQTT_DATA_PUBLISH_PERIODICITY);
    }
}




static void readAcousticSensor(xTimerHandle xTimer)
{
    (void) xTimer;

    //TickType_t  current_timer;
    uint64_t current_timer;


    float acousticData, sp, spl;
    double si;
    char FileContent[100] ;
    const char Filename[100];

    Retcode_T retcode = RETCODE_OK;
    Sensor_Value_T sensorValue;




}

/*
 *   Controler-Functions
 */

static void AppControllerEnable(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);


    Retcode_T retcode;
    TickType_t  current_timer;
    uint32_t timerBlockTime = UINT32_MAX;


    // Sensor
    	 retcode = Sensor_Enable();

    // File

    	 const char Filename[100];

    	 const char FileContent[] =  "Timer1,Acc_X_1, Acc_Y_1, Acc_Z_1,Timer2,Acc_X_2,Acc_Y_2,Acc_Z_2,Timer3,Acc_X_3, Acc_Y_3, Acc_Z_3,"
    			 "Timer4,Acc_X_4,Acc_Y_4,Acc_Z_4,Timer4,Acc_X_D,Acc_Y_D,Acc_Z_D,Timer1,Gyro_X_1,Gyro_Y_1,Gyro_Z_1,Timer2,Gyro_X_2,Gyro_Y_2,Gyro_Z_2,"
    		"Timer3,Gyro_X_3,Gyro_Y_3,Gyro_Z_3,Timer4,Gyro_X_4,Gyro_Y_4,Gyro_Z_4,Timer4,Gyro_X_D,Gyro_Y_D,Gyro_Z_D,"
      		"Mag_X,Mag_Y,Mag_Z,Noise, Humidity,Pressure,Temperature,Light,Time_absolut,Battery_Voltage \n";


           sprintf(Filename,"%s%i%s",FilePrefix,FileCounter,FileType);

         while(RETCODE_OK == searchForFileOnSdCard(Filename, NULL)){

        	   FileCounter=FileCounter+1;
        	   sprintf(Filename,"%s%i%s",FilePrefix,FileCounter,FileType);

           }



           createFileOnSdCard(Filename);

           writeDataIntoFileOnSdCard(Filename, FileContent);









           //if (RETCODE_OK == retcode)
           if ( RETCODE_OK == NoiseSensor_Enable())
              {
                  if (pdPASS != xTaskCreate(AppControllerFire, (const char * const ) "AppController", TASK_STACK_SIZE_APP_CONTROLLER, NULL, TASK_PRIO_APP_CONTROLLER, &AppControllerHandle))
                  {
                      retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
                  }
              }


Utils_PrintResetCause();

}







static void AppControllerSetup(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);
    Retcode_T retcode = RETCODE_OK;



    // Noise

    if (RETCODE_OK == NoiseSensor_Setup(22050U)) {
        uint32_t Delay = SAMPLING_FREQUENCY;
        uint32_t timerAutoReloadOn = UINT32_C(1);

        acousticHandle = xTimerCreate((const char *) "readAcousticSensor", Delay,timerAutoReloadOn, NULL, readAcousticSensor);
    }

    InitSdCard();

    retcode = Sensor_Setup(&SensorSetup); // Setup other sensors
    initAccelerometer();

    // Init Battery Level
    retcode = BatteryMonitor_Init	();


    retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerEnable, NULL, UINT32_C(0));
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerSetup : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}




void AppController_Init(void * cmdProcessorHandle, uint32_t param2)
{
    BCDS_UNUSED(param2);

    Retcode_T retcode = RETCODE_OK;

    if (cmdProcessorHandle == NULL)
    {
        printf("AppController_Init : Command processor handle is NULL \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        AppCmdProcessor = (CmdProcessor_T *) cmdProcessorHandle;
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerSetup, NULL, UINT32_C(0));
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/** ************************************************************************* */


