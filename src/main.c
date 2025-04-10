/*==================================================================================================
*    Copyright 2021-2024 NXP
*
*    NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be
*    used strictly in accordance with the applicable license terms. By expressly
*    accepting such terms or by downloading, installing, activating and/or otherwise
*    using the software, you are agreeing that you have read, and that you agree to
*    comply with and are bound by, such license terms. If you do not agree to be
*    bound by the applicable license terms, then you may not retain, install,
*    activate or otherwise use the software.
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
 *                                        INCLUDE FILES
 * 1) system and project includes
 * 2) needed interfaces from external units
 * 3) internal and external interfaces from this unit
==================================================================================================*/

#include "display.h"
#include "receiver.h"
#include "CDD_I2c.h"
#include "esc.h"
#include "servo.h"
#include "linear_camera.h"
#include "main_functions.h"
#include "hbridge.h"
#include "Mcal.h"
#include "pixy2.h"
/*==================================================================================================
 *                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
 *                                       LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
 *                                      LOCAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
 *                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
 *                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
 *                                      GLOBAL VARIABLES
==================================================================================================*/

/*==================================================================================================
 *                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/

/*==================================================================================================
 *                                       LOCAL FUNCTIONS
==================================================================================================*/

/*==================================================================================================
 *                                       GLOBAL FUNCTIONS
==================================================================================================*/
/**
 * @brief        Main function of the example
 * @details      Initializes the used drivers, hbridge, display, Pixy and servo.  The code
 *                  tries to keep the car between the two black lines using a Pixy2 camera.
 */

int main(void)
{
    int CarSteer;
    DetectedVectors PixyVectors;

    /*Initialize RTD drivers with the compiled configurations*/
    DriversInit();

    /*Initialize Esc driver*/
    /*First parameter: The Pwm Channel that was configured in Peripherals tool for the Esc*/
    /*Next parameters: Amount of Pwm ticks needed to achieve 1ms, 1.5ms and 2ms long signals, standard for controlling an esc.*/
    /*EscInit(0U, 1638U, 2457U, 3276U);*/

    /*Initialize Servo driver*/
    /*First parameter: The Pwm Channel that was configured in Peripherals tool for the Servo*/
    /*Next parameters: Amount of Pwm ticks needed to achieve maximum desired left turn, right turn and middle position for your servo.*/
    ServoInit(1U, 3300U, 1700U, 2500U);
    /*ServoTest();*/

    /*Initialize Hbridge driver*/
    /*First two parameters: The Pwm Channels that were configured in Peripherals tool for the motors' speeds*/
    /*Next parameters: The Pcr of the pins used for the motors' directions*/
    HbridgeInit(2U, 3U, 32U, 33U, 6U, 64U);
    /*HbridgeTest();*/

    /*Initialize display driver*/
    /*The display driver is for a 0.96", 128x32 OLED monochrome using a SSD1306 chip, like the one in the MR-CANHUBK344 kit*/
    /*Parameter: Should the driver rotate the fontmap before printing. You should leave this to 1U unless you have a special use case*/
    /*DisplayInit(0U);*/
    /*DisplayTest();*/

    /*Initialize receiver driver*/
    /*First parameter: The Icu channel that was configured in Peripherals tool for the input pin connected to the receiver's PPM signal*/
    /*Second parameter: The Gpt channel that was configured in Peripherals tool for measuring the input signal's length*/
    /*Next parameters: Amount of Gpt ticks needed to measure the shortest channel signal length, median signal length,
     * maximum signal length and the minimum signal length for the portion between the PPM signals*/
    /*ReceiverInit(0U, 0U, 11700U, 17700U, 23700U, 26000U);*/
    /*ReceiverTest();*/

    /*Initialize linear camera driver*/
    /*First parameter: The Pwm channel that was configured in Peripherals tool for the clock sent to the camera*/
    /*Second parameter: The Gpt channel that was configured in Peripherals tool for determining the length of the shutter signal*/
    /*Third parameter: The Adc group that was configured in Peripherals tool for sampling the analog signal output of the camera*/
    /*Fourth parameter: The Pcr of the pin used for sending the shutter signal to the camera*/
    /*LinearCameraInit(4U, 1U, 0U, 97U);*/
    /*CameraTest();*/

    /*Initialize Pixy2 camera driver*/
    /*First parameter: The I2c address that was configured on the Pixy2 that the driver should communicate with*/
    /*Second parameter: The I2c channel that was configured in Peripherals tool for communication with the camera and display*/
    Pixy2Init(0x54, 0U);
    /*Pixy2Test();*/

    while(1){
        double m0 = 0, m1 = 0;
        Pixy2GetVectors(&PixyVectors);
        if(PixyVectors.NumberOfVectors>=2){
            double x0 = PixyVectors.Vectors[0].x0;
            double x1 = PixyVectors.Vectors[0].x1;

            double y0 = PixyVectors.Vectors[0].y0;
            double y1 = PixyVectors.Vectors[0].y1;

            double x0_1 = PixyVectors.Vectors[1].x0;
            double x1_1 = PixyVectors.Vectors[1].x1;

            double y0_1 = PixyVectors.Vectors[1].y0;
            double y1_1 = PixyVectors.Vectors[1].y1;
            m0 = (x0 - x1) / (y0-y1);
            m1 = (x0_1 - x1_1) / (y0_1 - y1_1);

        }
        else if(PixyVectors.NumberOfVectors == 1){
            double x0 = PixyVectors.Vectors[0].x0;
            double x1 = PixyVectors.Vectors[0].x1;

            double y0 = PixyVectors.Vectors[0].y0;
            double y1 = PixyVectors.Vectors[0].y1;
            m0 = (x0 - x1) / (y0-y1);
        }
        CarSteer = (m0  + m1) * 65;
        HbridgeSetSpeed(100);
        Steer(CarSteer);
    }
}


#ifdef __cplusplus
}
#endif

/** @} */
