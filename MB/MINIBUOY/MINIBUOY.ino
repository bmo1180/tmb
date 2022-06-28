/*!
 * @version v1.1.0
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

//===== INCLUDES SECTION =====================================================//
#include "ESP32.h"
//============================================================================//

//===== DEFINITIONS SECTION ==================================================//
#define debug   Serial
#define WINDOW_SIZE 200

// GPIO
static const uint8_t LED_MAIN       = 2;
static const uint8_t BTN_BOOT       = 0;
static const uint8_t BTN_USER       = 27;
static const uint8_t GPS_TX         = 16;
static const uint8_t GPS_RX         = 17;
static const uint8_t BAT_PIN        = 35;

// Timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// RTOS
xSemaphoreHandle mutex = NULL;
//============================================================================//

//===== VARIABLES SECTION ====================================================//

// GPS
static const uint32_t GPS_BAUD      = 115200;
static const double R               = 6371e3;
//static const double toKnots         = 1.9438445; wenn unkommentiert, dann auch Zeile 370 und 480 ändern

// OLED
/*static const uint8_t OLED_ADDR      = 0x3D;
static const uint8_t OLED_WIDTH     = 128;
static const uint8_t OLED_HEIGHT    = 64;
static const uint8_t OLED_RESET     = 4;
static const uint32_t OLED_CLK      = 100000UL;*/

// BATTERY
static const double MIN_LIPO = 3.05;
static const double MAX_LIPO = 4.05;
static const double ADC_RES = 4096.0;
String batstat = "FULL";
//static const double correction = 0.03;
//static const double AVREF = 3.3;
//static const double R1 = 4.7;
//static const double R2 = 10.0;


// WIFI / UPDATER
//===== CHANGE BELOW AS PER NEEDS ============================================//
static const uint8_t SSID[]         = "MINIBUOY";   // SSID of AP here.
static const uint8_t PASS[]         = "coachbuoy";   // PASS of AP here.
static const uint8_t FW_VERSION[]   = "1.0.0";
static const uint32_t UPDATE_CHECK_INTERVAL = 60 * 60 * 1000;
static const char URL_FW_VER[] = "https://raw.githubusercontent.com/bmo1180/tmb/master/fota/FW-Version.txt";
static const char URL_FW_BIN[] = "https://raw.githubusercontent.com/bmo1180/tmb/master/fota/FW.bin";
//============================================================================//

// Button
static const uint32_t DEB_TIME      = 50;

// Library
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
//Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET, OLED_CLK, OLED_CLK);
GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT> display(GxEPD2_213_B74(5, 26, 33, 32)); //CS, DC, RST, BUSY
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;
IO::Tactile button(BTN_USER);

// Variables
bool mFix                           = false;
bool point1Flag                     = false; 
bool point2Flag                     = false;
bool timerFlag                      = false;
bool buttonFlag                     = false;
bool shortPressFlag                 = false;
bool longPressFlag                  = false;
bool overPressFlag                  = false;
bool checkForUpdateFlag             = false;
bool updateBatteryFlag              = false;
bool updaterEN                      = true;

uint8_t mSat                        = 0;
double mLat     = 0.0,      mLng    = 0.0;
double mLat1    = 0.0,      mLat2   = 0.0;
double mLng1    = 0.0,      mLng2   = 0.0;
uint32_t mTime1 = 0,        mTime2  = 0;
char latco[256] = {0}; 
char lngco[256] = {0};

double voltage  = 0.0;
double percent  = 0.0;
double dir      = 0.0;
double spd      = 0.0;
uint8_t acc[32] = {0};
int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;

char msg[256] = {0};
//============================================================================//

//===== INTERRUPTS SECTION ===================================================//
void IRAM_ATTR bootBtnISR() {
    static uint32_t debounceTimer = 0;
    if(millis() - DEB_TIME > debounceTimer) {
        debounceTimer = millis();
        buttonFlag = true;
    }
}

void IRAM_ATTR userBtnISR() {
    static uint32_t debounceTimer = 0;
    if(millis() - DEB_TIME > debounceTimer) {
        debounceTimer = millis();
        buttonFlag = true;
    }
}

void IRAM_ATTR timerISR() {
    portENTER_CRITICAL_ISR(&timerMux);
    
    static uint32_t ledCounter = 0;
    ledCounter++;
    if(ledCounter >= 1000) {
        digitalWrite(LED_MAIN, !digitalRead(LED_MAIN));
        ledCounter = 0;
    }

    static uint32_t updateCounter = 0;
    if(updaterEN) {
        updateCounter++;
        if(updateCounter >= UPDATE_CHECK_INTERVAL) {
            updaterEN = false;
            checkForUpdateFlag = true;
            updateCounter = 0;
        }
    }

    static uint32_t updateBattery = 55000;
    updateBattery++;
    if(updateBattery > 60000) {
        updateBattery = 0;
        updateBatteryFlag = true;
    }
    
    portEXIT_CRITICAL_ISR(&timerMux);
}
//============================================================================//

//===== FUCTION DEFINITIONS SECTION -=========================================//
void setup() {
    SystemInit();

    mutex = xSemaphoreCreateMutex();
    xTaskCreate(IOTask, "IO Task", 1024 * 2, NULL, 2, NULL);
    xTaskCreate(GPSTask, "GPS Task", 1024 * 2, NULL, 2, NULL);
    xTaskCreate(OLEDTask, "OLED Task", 1024 * 2, NULL, 2, NULL);
    xTaskCreate(WifiTask, "Wifi Task", 1024 * 10, NULL, 2, NULL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void SystemInit(void) {
    InitSerial();
    InitGPIO();
    InitTimer();
    InitGPS();
    InitOLED();
    InitWifi();
}
//============================================================================//

//===== IO FUNCTION SECTION ==================================================//
/*******************************************************************************
This task is working with IO's and its states. It is dealing with all user
interaction and does not depend on other parallel tasks.
*******************************************************************************/
void IOTask(void *param) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(1) {
        button.loop();
        checkForPressEvent();
        handlePressEvent();

        if(updateBatteryFlag) {
            updateBatteryFlag = false;
            getBatteryLevel();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void InitGPIO(void) {
    serialDisplay("[INFO] Init GPIO");

    // ANALOG
    analogReadResolution(12);

    // LED
    pinMode(LED_MAIN, OUTPUT);
    digitalWrite(LED_MAIN, LOW);

    // BTN 
    //pinMode(BTN_BOOT, INPUT);   // Boot button can be used for GPS grab as well
    pinMode(BTN_USER, INPUT);   // User button is primarily used for grabbing GPS

    // INTERRUPT
    //attachInterrupt(BTN_BOOT, bootBtnISR, FALLING);
    attachInterrupt(BTN_USER, userBtnISR, FALLING);
}

void InitSerial(void) {
    debug.begin(115200);
    while(!debug);
    delay(100);
}

void InitTimer(void) {
    serialDisplay("[INFO] Init Timer");

    timer = timerBegin(1, 80, true);    // Timer1, PreScaler 80, UpCounter
    timerAttachInterrupt(timer, &timerISR, true);
    timerAlarmWrite(timer, 1000, true); // every 1ms
    timerAlarmEnable(timer);
}

void serialDisplay(char *msg) {
    debug.println((char *) msg);
}

void checkForPressEvent(void) {
    if(buttonFlag) {
        if(button.getState() == IO::TACTILE_STATE_SHORT) {
            buttonFlag = false;
            shortPressFlag = true;
        } else if(button.getState() == IO::TACTILE_STATE_LONG) {
            buttonFlag = false;
            longPressFlag = true;
        } else if(button.getState() == IO::TACTILE_STATE_OVER) {
            buttonFlag = false;
            overPressFlag = true;
        }
    }
}

void handlePressEvent(void)  {
    if(shortPressFlag) {
        shortPressFlag = false;
        point1Flag = true;

        // Grabbing data
        mLat1 = gps.location.lat();
        mLng1 = gps.location.lng();
        mTime1 = millis();

        // Showing data
        xSemaphoreTake(mutex, portMAX_DELAY);
        sprintf(msg, "Short press.\r\n Lat: %lf, Lng: %lf, Millis: %d", mLat1, mLng1, mTime1);
        serialDisplay(msg);
        xSemaphoreGive(mutex);

    } else if(longPressFlag) {
        longPressFlag = false;
        point2Flag = true;

        // Grabbing data
        mLat2 = gps.location.lat();
        mLng2 = gps.location.lng();
        mTime2 = millis();

        // Showing data.
        xSemaphoreTake(mutex, portMAX_DELAY);
        sprintf(msg, "Long press.\r\n Lat: %lf, Lng: %lf, Millis: %d", mLat2, mLng2, mTime2);
        serialDisplay(msg);
        xSemaphoreGive(mutex);

    } else if(overPressFlag) {
        overPressFlag = false;
        checkForUpdateFlag = true;

        // Showing data.
        xSemaphoreTake(mutex, portMAX_DELAY);
        sprintf(msg, "Over press.");
        serialDisplay(msg);
        xSemaphoreGive(mutex);
    }
}

void getBatteryLevel(void) {
        SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
        VALUE = analogRead(BAT_PIN);        // Read the next sensor value
        READINGS[INDEX] = VALUE;           // Add the newest reading to the window
        SUM = SUM + VALUE;                 // Add the newest reading to the sum
        INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
        AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result 
        voltage = (double) analogRead(BAT_PIN) / ADC_RES * 5.06; // - correction;   
        //voltage = ((double) analogRead(BAT_PIN) / ADC_RES * (R1 + R2) / R2 * AVREF);
        voltage = voltage > MAX_LIPO ? MAX_LIPO : voltage < MIN_LIPO ? MIN_LIPO : voltage;
        percent = myMap(voltage, MIN_LIPO, MAX_LIPO, 0, 100);

       
    if (percent > 80) {
        batstat = "FULL";
    
    } else  if (percent <= 80 && percent > 20) {
        batstat = "  OK";
    
    } else  if (percent <= 20) {
        batstat = " LOW";
}

}

double myMap(double x, double inMin, double inMax, double outMin, double outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
//============================================================================//

//===== GPS FUNCTION SECTION =================================================//
/*******************************************************************************
This task is only acquiring GPS data periodically and storing this data for use
during calculation process. This acquired will not be effected or changed during
any calcualtion processes.
*******************************************************************************/
void GPSTask(void *param) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(1) {
        while(SerialGPS.available() > 0) {
            gps.encode(SerialGPS.read());
        }
        mFix = gps.location.isValid();
        mSat = gps.satellites.value();
        mLat = gps.location.lat();
        mLng = gps.location.lng();
        double satlat = gps.location.lat();
        double satlng = gps.location.lng();
        int dlat = satlat;
        int dlng =satlng;
        int mlat = (satlat - (double)dlat) * 60.0;
        int mlng = (satlng - (double)dlng) * 60.0;      
        int slat = (satlat - (double)dlat - (double)mlat / 60.0) * 60.0 * 60.0;
        int slng = (satlng - (double)dlng - (double)mlng / 60.0) * 60.0 * 60.0;
        sprintf(latco, "%2d°%2d'%2d''", dlat, mlat, slat);
        //serialDisplay(latco);
        sprintf(lngco, "%2d°%2d'%2d''", dlng, mlng, slng);
        //serialDisplay(lngco);
        
        // Wait for 1000ms before taking new data.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void InitGPS(void) {
    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_TX, GPS_RX);
}

double getDirection(double lat1, double lng1, double lat2, double lng2) {
    // LAT1/LNG1 is old, LAT2/LNG2 is current.
    double d = 0.0;
    double rLat1 = lat1 * M_PI / 180.0;
    double rLat2 = lat2 * M_PI / 180.0;
    double rLng1 = lng1 * M_PI / 180.0;
    double rLng2 = lng2 * M_PI / 180.0;
    double dLat = rLat1 - rLat2;  //old - cur
    double dLng = rLng1 - rLng2;  //old - cur
    double y = sin(rLng2 - rLng1) * cos(rLat2);
    double x = cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(rLng2 - rLng1);
    double th = atan2(y, x);
    d = (th * (180.0 / M_PI)) + 360.0 +180;  //+180 ebtfernen um Richtung nach anzuzeigen //Jetzt Richtung aus der der Strom kommt
    while(d > 360.0) {
        d -= 360.0;
    }
    return d;   
}

double getSpeed(double lat1, double lng1, double lat2, double lng2, double tim1, double tim2) {
    // LAT1/LNG1 is old, LAT2/LNG2 is current.
    double s = 0.0; //speed in m/min
    double sps = 0.0; //speed in m/s
    double rLat1 = lat1 * M_PI / 180.0;
    double rLat2 = lat2 * M_PI / 180.0;
    double rLng1 = lng1 * M_PI / 180.0;
    double rLng2 = lng2 * M_PI / 180.0;
    double dLat = rLat1 - rLat2;  //old - cur
    double dLng = rLng1 - rLng2;  //old - cur
    double dTime = (tim2 - tim1) / 1000.0;  //Convert millis to sec
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(rLat2) * cos(rLat1) * sin(dLng / 2) * sin(dLng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = R * c;
    sps = (d / (double) dTime); //* toKnots;
    s = sps * 60;
    return s;
}

void getAccuracy(uint8_t sat, uint8_t *str) {
    static uint8_t idx = 0;
    static uint8_t val[4] = {0, 0, 0, 0};

    // Moving averaging filter.
    val[idx++] = sat;
    if(idx >= 4) idx = 0;
    uint8_t avgSat = (val[0] + val[1] + val[2] + val[3]) / 4;

    if(3 <= avgSat && avgSat <= 5) {
        strcpy((char *) str, "BAD");
    } else if(6 <= avgSat && avgSat <= 8) {
        strcpy((char *) str, "OK");
    } else if(avgSat >= 9) {
        strcpy((char *) str, "GOOD");
    } else {
        strcpy((char *) str, "N/A");
    }
}
//============================================================================//

//===== OLED FUNCTION SECTION ================================================//
/*******************************************************************************
This task is only reading states changes done by the other task and 
displaying according to current state. This task will never effect or change any
states of the system.
*******************************************************************************/
void OLEDTask(void *param) {
    oledDisplayLogo();
    vTaskDelay(pdMS_TO_TICKS(5000));
    while(1) {
        if(!mFix && mSat == 0) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            display.setPartialWindow(0, 0, 250, 120);
            display.firstPage();
              do{
              //display.fillScreen(GxEPD_WHITE);
              u8g2Fonts.setFont(u8g2_font_courB08_tf);
              u8g2Fonts.setCursor(0, 10);
              u8g2Fonts.print("SAT:");
              u8g2Fonts.setCursor(30, 10);
              sprintf(msg, "%2d", mSat);
              u8g2Fonts.print(msg);
              u8g2Fonts.setCursor(195, 10);
              u8g2Fonts.print("BAT:");
              u8g2Fonts.setCursor(225, 10);
              u8g2Fonts.print(batstat);
              u8g2Fonts.setFont(u8g2_font_courB24_tf);
              u8g2Fonts.setCursor(31, 58);
              u8g2Fonts.println("Searching");
              //u8g2Fonts.setCursor(24, 84);
              u8g2Fonts.print(" Satellites");
              }while(display.nextPage());
              xSemaphoreGive(mutex);
        } else if(mFix && mSat < 3) {
              xSemaphoreTake(mutex, portMAX_DELAY);
              display.setPartialWindow(0, 0, 250, 120);
              display.firstPage();
              do{
              //display.fillScreen(GxEPD_WHITE);
              u8g2Fonts.setFont(u8g2_font_courB08_tf);
              u8g2Fonts.setCursor(0, 10);
              u8g2Fonts.print("SAT:");
              u8g2Fonts.setCursor(30, 10);
              sprintf(msg, "%2d", mSat);
              u8g2Fonts.print(msg);
              u8g2Fonts.setCursor(195, 10);
              u8g2Fonts.print("BAT:");
              u8g2Fonts.setCursor(225, 10);
              u8g2Fonts.print(batstat);
              u8g2Fonts.setFont(u8g2_font_courB24_tf);
              u8g2Fonts.setCursor(31, 58);
              u8g2Fonts.println("Searching");
              //u8g2Fonts.setCursor(24, 84);
              u8g2Fonts.print(" Satellites");
              }while(display.nextPage());
              xSemaphoreGive(mutex);
        } else if(mFix &&  mSat > 3) {
            if(!point1Flag && !point2Flag) {
              xSemaphoreTake(mutex, portMAX_DELAY); 
              display.setPartialWindow(0, 0, 250, 120);
              display.firstPage();
              do{
              //display.fillScreen(GxEPD_WHITE);
              u8g2Fonts.setFont(u8g2_font_courB08_tf);
              u8g2Fonts.setCursor(0, 10);
              u8g2Fonts.print("SAT:");
              u8g2Fonts.setCursor(30, 10);
              sprintf(msg, "%2d", mSat);
              u8g2Fonts.print(msg);
              u8g2Fonts.setCursor(195, 10);
              u8g2Fonts.print("BAT:");
              u8g2Fonts.setCursor(225, 10);
              u8g2Fonts.print(batstat);
              u8g2Fonts.setFont(u8g2_font_courB24_tf);
              u8g2Fonts.setCursor(71, 72);
              u8g2Fonts.print("READY");
              u8g2Fonts.setFont(u8g2_font_courB12_tf);
              u8g2Fonts.setCursor(0, 114);
              u8g2Fonts.print(latco);
              u8g2Fonts.setCursor(154, 114);
              u8g2Fonts.print(lngco);
              }while(display.nextPage());
              xSemaphoreGive(mutex);
          } else if(point1Flag && !point2Flag) {
              xSemaphoreTake(mutex, portMAX_DELAY);
              display.setPartialWindow(0, 0, 250, 120);
              display.firstPage();
              do{
              //display.fillScreen(GxEPD_WHITE);
              u8g2Fonts.setFont(u8g2_font_courB08_tf);
              u8g2Fonts.setCursor(0, 10);
              u8g2Fonts.print("SAT:");
              u8g2Fonts.setCursor(30, 10);
              sprintf(msg, "%2d", mSat);
              u8g2Fonts.print(msg);
              u8g2Fonts.setCursor(195, 10);
              u8g2Fonts.print("BAT:");
              u8g2Fonts.setCursor(225, 10);
              u8g2Fonts.print(batstat);
              u8g2Fonts.setFont(u8g2_font_courB24_tf);
              u8g2Fonts.setCursor(36, 72);
              u8g2Fonts.print("MEASURING");
              }while(display.nextPage());
              xSemaphoreGive(mutex);
          } else if(point1Flag && point2Flag) {
              dir = getDirection(mLat1, mLng1, mLat2, mLng2);
              spd = getSpeed(mLat1, mLng1, mLat2, mLng2, mTime1, mTime2);
              getAccuracy(mSat, acc);
              xSemaphoreTake(mutex, portMAX_DELAY);
              display.setPartialWindow(0, 0, 250, 120);
              display.firstPage();
              do{
              //display.fillScreen(GxEPD_WHITE);
              u8g2Fonts.setFont(u8g2_font_courB08_tf);
              u8g2Fonts.setCursor(0, 10);
              u8g2Fonts.print("SAT:");
              u8g2Fonts.setCursor(30, 10);
              sprintf(msg, "%2d", mSat);
              u8g2Fonts.print(msg);
              u8g2Fonts.setCursor(195, 10);
              u8g2Fonts.print("BAT:");
              u8g2Fonts.setCursor(225, 10);
              u8g2Fonts.print(batstat);
              u8g2Fonts.setFont(u8g2_font_courB18_tf);
              u8g2Fonts.setCursor(0, 54);
              sprintf(msg, "DIR: %3.0lf°\r\nSPD: %3.1lfm/min", dir, spd);
              u8g2Fonts.print(msg);
              u8g2Fonts.setFont(u8g2_font_courB12_tf);
              u8g2Fonts.setCursor(0, 114);
              u8g2Fonts.print(latco);
              u8g2Fonts.setCursor(154, 114);
              u8g2Fonts.print(lngco);
              }while(display.nextPage());        
              xSemaphoreGive(mutex);
          } else {
              xSemaphoreTake(mutex, portMAX_DELAY);
              display.setPartialWindow(115, 50, 200, 40);
              display.firstPage();
              do{
              u8g2Fonts.setFont(u8g2_font_courB08_tf);
              u8g2Fonts.setCursor(0, 10);
              u8g2Fonts.print("SAT:");
              u8g2Fonts.setCursor(30, 10);
              sprintf(msg, "%2d", mSat);
              u8g2Fonts.print(msg);
              u8g2Fonts.setCursor(195, 10);
              u8g2Fonts.print("BAT:");
              u8g2Fonts.setCursor(225, 10);
              u8g2Fonts.print(batstat);
              u8g2Fonts.setFont(u8g2_font_courB14_tf);
              u8g2Fonts.setCursor(0, 44);
              sprintf(msg, "Lat: %lf\r\n Lng: %lf\r\n", mLat, mLng);
              }while(display.nextPage());
                xSemaphoreGive(mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void InitOLED(void) {
    serialDisplay("[INFO] Init OLED");

  display.init();
  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  display.setRotation(3);

  u8g2Fonts.begin(display); // connect u8g2 procedures to Adafruit GFX
  
  uint16_t bg = GxEPD_WHITE;
  uint16_t fg = GxEPD_BLACK;
  u8g2Fonts.setForegroundColor(fg);         // apply Adafruit GFX color
  u8g2Fonts.setBackgroundColor(bg);

  do{
  }while (display.nextPage());
}

void oledDisplayLogo(void) {
    do{
    //display.fillScreen(GxEPD_BLACK);
    //display.drawBitmap(0, 0, myBitmap, 250, 120, GxEPD_BLACK);
    }while (display.nextPage());
    
}

void oledDisplay(const char *str) {
    //display.clearDisplay();
    do{
    display.fillScreen(GxEPD_WHITE);  
    u8g2Fonts.setFont(u8g2_font_courB18_tf);
    display.setCursor(5, 60);
    display.println(str);
    }while (display.nextPage());
}
//============================================================================//

//===== WIFI FUNCTION SECTION ================================================//
/*******************************************************************************
This task is only wifi releated and is used to update the firmware whenever
a user presses the button for more than 5 seconds as well as every interval
as defined by the user. Also this thread will run in blocking mode.
*******************************************************************************/
void WifiTask(void *param) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(1) {
        if(checkForUpdateFlag) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            updateCheck();
            xSemaphoreGive(mutex);
            updaterEN = true;
            checkForUpdateFlag = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void InitWifi(void) {
    sprintf(msg, "[INFO] Init Wifi");
    serialDisplay(msg);
    oledDisplay(msg);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
}

void updateCheck(void) {
    if(!WiFi.isConnected()) {
        connectWifi(); 
        if(!isCurrentVersion()) {
            firmwareUpdate();
        }
    }
    disconnectWifi();
}

void connectWifi(void) {
    uint8_t noOfAP = WiFi.scanNetworks();
    if(noOfAP == 0) {
        sprintf(msg, "[WARN] No access point found");
        serialDisplay(msg);
        oledDisplay(msg);
        return;
    }

    sprintf(msg, "[INFO] Connecting to wifi");
    serialDisplay(msg);
    oledDisplay(msg);

    WiFi.begin((char *) SSID, (char *) PASS);
    while(!WiFi.isConnected()) vTaskDelay(pdMS_TO_TICKS(1000));

    debug.println(WiFi.localIP());
    sprintf(msg, "[INFO] Connected to wifi");
    serialDisplay(msg);
    oledDisplay(msg);
}

void disconnectWifi(void) {
    sprintf(msg, "[INFO] Disconnecting wifi");
    serialDisplay(msg);
    oledDisplay(msg);

    if(WiFi.isConnected()) {
        WiFi.disconnect(true);
    }
}

void firmwareUpdate(void) {
    sprintf(msg, "[INFO] Updating firmware");
    serialDisplay(msg);
    oledDisplay(msg);

    WiFiClientSecure client;
    client.setCACert((char *) certificate);
    httpUpdate.setLedPin(LED_MAIN, LOW);
    t_httpUpdate_return ret = httpUpdate.update(client, URL_FW_BIN);
    switch(ret) {
        case HTTP_UPDATE_FAILED:
            sprintf(msg, "Update failed. Error: (%d): %s", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            serialDisplay(msg);
            oledDisplay(msg);
            break;

        case HTTP_UPDATE_NO_UPDATES:
            sprintf(msg, "Device is up to date.");
            serialDisplay(msg);
            oledDisplay(msg);
            break;
        
        case HTTP_UPDATE_OK:
            sprintf(msg, "Update OK.");
            serialDisplay(msg);
            oledDisplay(msg);
            break;

        default:
            break;
    }
}

bool isCurrentVersion(void) {
    sprintf(msg, "[INFO] Checking for update");
    serialDisplay(msg);
    oledDisplay(msg);

    WiFiClientSecure *client = new WiFiClientSecure();
    client->setCACert((char *) certificate);

    HTTPClient request;
    if(request.begin(*client, URL_FW_VER)) {
        if(request.GET() == HTTP_CODE_OK) {
            String payload = request.getString();
            payload.trim();
            if(!payload.equals((char *) FW_VERSION)) {
                sprintf(msg, "Found FW_VER: %s\r\nCurrent FW_VER: %s", payload.c_str(), FW_VERSION);
                serialDisplay(msg);
                oledDisplay(msg);
                return false;
            } else {
                sprintf(msg, "[INFO] Device is up-to-date");
                serialDisplay(msg);
                oledDisplay(msg);
            }
        }
        request.end();
    }

    delete client;
    return true;
}
//============================================================================//
