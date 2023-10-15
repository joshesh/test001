/* Includes */
#include <OneButton.h>
#include <BleCombo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Time.h>
#include <MapFloat.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/* Defines */
#define PIN_INPUT 32
#define PIN_LED1 19
#define PIN_LED2 18
#define Battery_Pin 33
#define Device_Name "ZenMouse1067"
#define Manufacturer "Edukaizen LLP"

/* Objects */
MPU6050 accelgyroh(0x69), accelgyrov(0x68);
BleComboKeyboard Keyboard(Device_Name,Manufacturer,100);
BleComboMouse Mouse(&Keyboard);
ESP32Time rtc;
/* Globals */
int16_t axh, ayh, azh;
int16_t gxh, gyh, gzh;
int16_t axv, ayv, azv;
int16_t gxv, gyv, gzv;
int16_t sensitivityDivider = 300;   //250
/* radius in which motion is ignored. Not used since much smaller than sensitivityDivider.*/
int deadZone = 100; 
signed char mouseVX, mouseVY;
OneButton button(PIN_INPUT, true, true);
unsigned long pressStartTime;
unsigned long currentMillis, currentMinutes;
unsigned long previousMillis = 0;
unsigned long previousMinutes = 0;  
int ledState = LOW; 

uint8_t bLevel;



/**************** Button ISR ********************/
void IRAM_ATTR checkTicks()
{
   button.tick();
}
/******************** Procedures ************************/
void processMouse(int dX, int dY)
{
  mouseVX = dX / sensitivityDivider;
  mouseVY = dY / sensitivityDivider;
//  Serial.println(mouseVX);
//  Serial.println(mouseVY);
  if (mouseVX != 0 || mouseVY != 0)
  {
    Mouse.move(mouseVX, mouseVY);
  }
}

void updateBatteryLevel(void)
{
  static int rawReading;
  static float voltage;
  rawReading = analogReadMilliVolts(Battery_Pin);
  voltage = (2*(rawReading/1000.000));
  bLevel = mapFloat(voltage,3.7,4.2,10.0,100.0);
  bLevel = roundf(bLevel);
  Keyboard.setBatteryLevel(bLevel);
}

void calibrate(void)
{
  digitalWrite(PIN_LED1,HIGH);
  accelgyroh.CalibrateGyro(6);
  accelgyrov.CalibrateGyro(6);
  digitalWrite(PIN_LED1,LOW);
}


/***************** Button Procedures *****************/
void singleClick(void) 
{
  Mouse.click(MOUSE_LEFT);
//  Serial.println("singleClick() detected.");
}

void doubleClick(void) 
{
   Mouse.click(MOUSE_LEFT);
   Mouse.click(MOUSE_LEFT);
}
void multiClick(void) 
{
  int n = button.getNumberClicks();
  if (n == 3)
  {
    Mouse.click(MOUSE_RIGHT);
  }
  else if (n == 4)
  {
    Keyboard.press(KEY_LEFT_GUI);
    Keyboard.press(KEY_LEFT_CTRL);
    Keyboard.write('o');
    delay(100);
    Keyboard.releaseAll();
  }
  else
  if( n== 10)
  {
    calibrate();
  }
//  else
//  if( n == 20)
//  {
////    handleUpdate();
//  }
//  else
//  {
//    
//      Serial.print("multiClick(");
//      Serial.print(n);
//      Serial.println(") detected.");
//  }
}  

void pressStart()  //Long press hold start time
{
//   Serial.println("pressStart()");
   pressStartTime = millis() - 1000; // as set in setPressTicks()
   Mouse.press();
}

void pressStop() // pressStop()
{
  int x = 0;
  x = millis() - pressStartTime;
  Mouse.release();
}
/******************** Main Procedure *********************/
void setup()
{
   Mouse.begin();
   Keyboard.begin();
   pinMode(PIN_LED1,OUTPUT);
   pinMode(PIN_LED2, OUTPUT);
   digitalWrite(PIN_LED2, HIGH);
   Serial.begin(115200);
   analogReadResolution(12);
   analogSetAttenuation(ADC_11db);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   Wire.begin(25, 26);
   Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
   Fastwire::setup(400, true);
#endif
   accelgyroh.initialize();
   accelgyrov.initialize();
   calibrate();
   updateBatteryLevel();
   attachInterrupt(digitalPinToInterrupt(PIN_INPUT), checkTicks, CHANGE);
   button.attachClick(singleClick);
   button.attachDoubleClick(doubleClick);
   button.attachMultiClick(multiClick);
   button.setPressTicks(500);
   button.attachLongPressStart(pressStart);
   button.attachLongPressStop(pressStop);
}
void loop()
{
   button.tick();
   currentMinutes = rtc.getMinute();
   accelgyroh.getRotation(&gxh, &gyh, &gzh);
   accelgyrov.getRotation(&gxv, &gyv, &gzv);
   processMouse(-gyv, gxh);
   if(currentMinutes - previousMinutes >= 5)
   {
      updateBatteryLevel();
      previousMinutes = currentMinutes;
   }
   delay(20);
}
