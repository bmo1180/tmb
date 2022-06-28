/*!
 *              | v1.0.0 -> Ported entire code to FreeRTOS framework
 *              | v1.1.0 -> Added OTA functionality via Github
 *              | v1.1.1 -> Added battery voltage level.
 * 
 * @note    This is a library written in C++ using Arduino IDE with ESP32
 *          devkit, OLED and a GPS.
 * 
 *          This program reads GPS data does some calculations after user
 *          interactions and displays it.
 * 
 */

#ifndef __ESP32_H__
#define __ESP32_H__

//===== INCLUDES SECTION =====================================================//
//#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h> // including both doesn't use more code or ram
#include <GxEPD2_3C.h> // including both doesn't use more code or ram
#include <U8g2_for_Adafruit_GFX.h>

#include <HardwareSerial.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

#include "Tactile.h"
#include "cert.h"
//#include "logo.h"
//============================================================================//

//===== DEFINITIONS SECTION ==================================================//
#ifndef M_PI
#define M_PI            3.14159265358979323846
#endif
//============================================================================//

//===== IO FUNCTION SECTION ==================================================//
/*!
 * @brief A thread for handling all IO task and events.
 * @param None.
 * @return None.
 */
void IOTask(void *param);

/*!
 * @brief This function initializes the GPIO.
 * @param None.
 * @return None.
 */
void InitGPIO(void);

/*!
 * @brief This function initializes the serial.
 * @param None.
 * @return None.
 */
void InitSerial(void);

/*!
 * @brief This function initializes the timer.
 * @param None.
 * @return None.
 */
void InitTimer(void);

/*!
 * @brief This function displays the string on serial.
 * @param [str] String to display.
 * @return None.
 */
void serialDisplay(uint8_t *str);

/*!
 * @brief Checks if any press event has occoured or not.
 * @param None.
 * @return None.
 */
void checkForPressEvent(void);

/*!
 * @brief Handles all events related to button press.
 * @param None.
 * @return None.
 */
void handlePressEvent(void);

/*!
 * @brief Gets the current battery voltage level.
 * @param None.
 * @return None.
 */
void getBatteryLevel(void);

/*!
 * @brief Maps the inuput value into the given range,
 * @param [double] - Input value to be mapped.
 * @param [double] - Input minimum value.
 * @param [double] - Input maximum value.
 * @param [double] - Output minimum value.
 * @param [double] - Output maximum value.
 * @return [duoble] - Mapped user input into the output range.
 */
double myMap(double x, double inMin, double inMax, double outMin, double outMax);
//============================================================================//

//===== GPS FUNCTION SECTION =================================================//
/*!
 * @brief A thread for handling all GPS task and events.
 * @param None.
 * @return None.
 */
void GPSTask(void *param);

/*!
 * @brief Initializes the serial for GPS.
 * @param None.
 * @return None.
 */
void InitGPS(void);

/*!
 * @brief This function does the calculations for directions.
 * @param [lat1] GPS latitude point 1.
 * @param [lng1] GPS longitude point 1.
 * @param [lat2] GPS latitude point 2.
 * @param [lng2] GPS longitude point 2.
 * @return Returns direction .
 */
double getDirection(double lat1, double lng1, double lat2, double lng2);

/*!
 * @brief This function does the calculations for speed.
 * @param [lat1] GPS latitude point 1.
 * @param [lng1] GPS longitude point 1.
 * @param [lat2] GPS latitude point 2.
 * @param [lng2] GPS longitude point 2.
 * @param [tim1] Time of first reading.
 * @param [tim2] Time of second reading.
 * @return Returns speed.
 */
double getSpeed(double lat1, double lng1, double lat2, double lng2, double tim1, double tim2);

/*!
 * @brief This function check the accuracy of GPS based on satellites.
 * @param [sat] Current no. of satellites.
 * @param [str] Returning message.
 * @return None.
 */
void getAccuracy(uint8_t sat, uint8_t *str);
//============================================================================//

//===== OLED FUNCTION SECTION ================================================//
/*!
 * @brief A thread for handling all OLED based task and events.
 * @param None.
 * @return None.
 */
void OLEDTask(void *param);

/*!
 * @brief This function initializes the OLED.
 * @param None.
 * @return None.
 */
void InitOLED(void);

/*!
 * @brief This function displays the logo.
 * @param None.
 * @return None.
 */
void oledDisplayLogo(void);

/*!
 * @brief This function displays the string on OLED.
 * @param [str] String to display.
 * @return None.
 */
void oledDisplay(uint8_t *str);
//============================================================================//

//===== WIFI FUNCTION SECTION ================================================//
/*!
 * @brief A thread for handling all Wifi based task and events.
 * @param None.
 * @return None.
 */
void WifiTask(void *param);

/*!
 * @brief Initializes the wifi.
 * @param None.
 * @return None.
 */
void InitWifi(void);

/*!
 * @brief Check for updates on regular interval or interrupts.
 * @param None.
 * @return None.
 */
void updateCheck(void);

/*!
 * @brief Connects to the nearby given access point
 * @param None.
 * @return None.
 */
void connectWifi(void);

/*!
 * @brief Disconnect and turns off the wifi to conserve power.
 * @param None.
 * @return None.
 */
void disconnectWifi(void);

/*!
 * @brief Updates to the latest firmware version.
 * @param None.
 * @return None.
 */
void firmwareUpdate(void);

/*!
 * @brief Checks if the device is running latest firmware or not. 
 * @param None.
 * @return [bool] - True if the device is up to date else false.
 */
bool isCurrentVersion(void);
//============================================================================//

#endif
