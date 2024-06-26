#ifndef Excelsior_h
#define Excelsior_h

#include <Arduino.h>
#include <limits.h>
#include <Array.h>              // Used for dynamic length array, as in the error messages
#include <Adafruit_GFX.h>       // Include core graphics library for the display
#include <Adafruit_SSD1306.h>   // Include Adafruit_SSD1306 library to drive the display
#include <Adafruit_Sensor.h>  	// Needed for the Adafruit_BNO055 Gyrosensor
#include <Adafruit_BNO055.h>    // Needed for the Adafruit_BNO055 Gyrosensor
#include <utility/imumaths.h>
#include <Wire.h>
#include <math.h>

#include <Fonts/FreeMonoBold9pt7b.h>  // Add a custom font -> "!" from error-Triangle
#include <Fonts/FreeMono9pt7b.h>      // Add a custom font -> other text

using namespace std;

//define what color is which number for the switch cases and also what variables are used here
#undef WHITE              //defined via the Adafruit_SSD1306 library as: SSD1306_WHITE, needs to be undefined to be redefined

#define OFF          0      //COLORS      
#define WHITE        1      //  .      
#define RED          2      //  .      
#define GREEN        3      //  .      
#define BLUE         4      //  .      
#define CYAN         5      //  .      
#define MAGENTA      6      //  .      
#define YELLOW       7      //COLORS      
#define LIGHT        8      //IOTYPES / SENSORTYPES
#define LIGHT_NXT    9      //        .
#define TOUCH_NXT    10     //        .
#define TOUCH_EV3    11     //        .
#define INFRARED     12     //        .
#define VCC          13     //        .
#define GND          14     //        .
#define DIGITAL_OUT  16     //        .
#define ANALOG_OUT   17     //        .
#define DIGITAL_IN   18     //        .
#define ANALOG_IN    19     //IOTYPES / SENSORTYPES
#define CUSTOM       20     //SENSORTYPES
#define MOTOR_A      21     //MOTORS
#define MOTOR_B      22     //  .
#define MOTOR_C      23     //  .
#define MOTOR_D      24     //MOTORS
#define GYRO_X       25     //GYROSCOPE
#define GYRO_Y       26     //  .
#define GYRO_Z       27     //GYROSCOPE

//--German definitions:
#define AUS          OFF
#define WEISS        WHITE
#define ROT          RED
#define GRUEN        GREEN
#define BLAU         BLUE
//#define CYAN         CYAN
//#define MAGENTA      MAGENTA
#define GELB         YELLOW
#define LICHT        LIGHT
#define LICHT_NXT    LIGHT_NXT
#define TAST_NXT     TOUCH_NXT
#define TAST_EV3     TOUCH_EV3
#define INFRAROT     INFRARED

//--Further definitions:
#define OUT_A MOTOR_A
#define OUT_B MOTOR_B
#define OUT_C MOTOR_C
#define OUT_D MOTOR_D


typedef Array<int,10> _VecInt10;                      //datatype similar to c++ vector with a maximum of 10 indecies

class Excelsior
{
  public:
    Adafruit_SSD1306 display;
    Adafruit_BNO055 bno055;
    String ExcelsiorVersion;                            //tracks the Version of the Excelsior, that is being used
    Excelsior()                                         {Excelsior("A");};
    Excelsior(String _ExcelsiorVersion);
    void sensorSetup(int port, int type);
    void sensorSetup(int port, int pin, int IOtype);
    void sensorSetup(int port, int IOtype1, int IOtype2, int IOtype3, int IOtype4);
    void sensorSetup(int port, int (&IOtypes)[5]);
    void setSensor(int port, int type)                                                    {sensorSetup(port,type);};
    void setSensor(int port, int pin, int IOtype)                                         {sensorSetup(port,pin,IOtype);};
    void setSensor(int port, int IOtype1, int IOtype2, int IOtype3, int IOtype4)          {sensorSetup(port,IOtype1,IOtype2,IOtype3,IOtype4);};
    void setSensor(int port, int (&IOtypes)[5])                                           {sensorSetup(port,IOtypes);};
    void lightDelay(int delay);
    void motor(int port, int dir);
    void motorFwd(int val1 = -1000, int val2 = -1000, int val3 = -1000, int val4 = -1000, int val5 = -1)       {_motors(val1,val2,val3,val4,val5,false);};            //-1000 is used as undefined in _motors();     
    void motorRev(int val1 = -1000, int val2 = -1000, int val3 = -1000, int val4 = -1000, int val5 = -1)       {_motors(val1,val2,val3,val4,val5,true);};
    void motorOff();                                      
    void motorOff(int port)                             {motor(port,0);};
    void invertMotor(int motor1 = -1, int motor2 = -1, int motor3 = -1, int motor4 = -1);
    bool button();
    void sensorWrite(int port, int pin, int signal);
    int  sensorRead(int port);
    int  sensorRead(int port, int colorOrPin);
    int  sensorRead(int port, int color, bool percent);
    int  gyroRead(int axis);
    void gyroReset()                                    {gyroReset(-1);};
    void gyroReset(int axis)                            {gyroReset(axis,false);};
    void gyroReset(int axis, bool toOriginal);
    void dU()                                           {displayUpdate(0);};
    void displayUpdate()                                {displayUpdate(0);};
    void dU(int type)                                   {displayUpdate(type);};
    void displayUpdate(int type); 
    void dU(int (&layout)[8], String errorMessage, bool clearDisplay)      {displayUpdate(layout,errorMessage,clearDisplay);};
    void displayUpdate(int (&layout)[8], String errorMessage, bool clearDisplay);
    void dU(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8)             {displayUpdate(layout1,layout2,layout3,layout4,layout5,layout6,layout7,layout8);};
    void displayUpdate(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8);
    void dT(int x_, int y_, String s_)                  {displayText(x_,y_,s_);};
    void displayText(int x_, int y_, String s_);
    void dTC()                                          {displayTextClear();};
    void displayTextClear();
    void dB()                                           {displayBorder();};
    void displayBorder();
    void dC()                                           {displayClear();}; 
    void displayClear();

    //--German definitions:
    void SensorSetup(int port, int type)                                                  {sensorSetup(port,type);};
    void SensorSetup(int port, int pin, int IOtype)                                       {sensorSetup(port,pin,IOtype);};
    void SensorSetup(int port, int IOtype1, int IOtype2, int IOtype3, int IOtype4)        {sensorSetup(port,IOtype1,IOtype2,IOtype3,IOtype4);};
    void SensorSetup(int port, int (&IOtypes)[5])                                         {sensorSetup(port,IOtypes);};
    void SetSensor(int port, int type)                                                    {sensorSetup(port,type);};
    void SetSensor(int port, int pin, int IOtype)                                         {sensorSetup(port,pin,IOtype);};
    void SetSensor(int port, int IOtype1, int IOtype2, int IOtype3, int IOtype4)          {sensorSetup(port,IOtype1,IOtype2,IOtype3,IOtype4);};
    void SetSensor(int port, int (&IOtypes)[5])                                           {sensorSetup(port,IOtypes);};
    void LichtVerzoegerung(int delay)                   {lightDelay(delay);};
    void Motor(int port, int dir)                       {motor(port,dir);};
    void MotorFwd(int val1 = -1000, int val2 = -1000, int val3 = -1000, int val4 = -1000, int val5 = -1000)      {motorFwd(val1,val2,val3,val4,val5);}; 
    void MotorRev(int val1 = -1000, int val2 = -1000, int val3 = -1000, int val4 = -1000, int val5 = -1000)      {motorRev(val1,val2,val3,val4,val5);}; 
    void MotorAus()                                     {motorOff();};
    void MotorAus(int port)                             {motorOff(port);};
    void MotorInvertieren(int motor1 = -1, int motor2 = -1, int motor3 = -1, int motor4 = -1)       {invertMotor(motor1,motor2,motor3,motor4);};
    bool Knopf()                                        {return button();};
    int  SensorWert(int port)                           {return sensorRead(port);};
    int  SensorWert(int port, int color)                {return sensorRead(port,color);};
    int  SensorWert(int port, int color, bool percent)  {return sensorRead(port,color,percent);};
    int  GyroWert(int axis)                             {return gyroRead(axis);};
    void GyroReset()                                    {gyroReset();};
    void GyroReset(int axis)                            {gyroReset(axis);};
    void GyroReset(int axis, bool toOriginal)           {gyroReset(axis,toOriginal);};
    void DisplayAktualisieren()                         {displayUpdate();};
    void DA()                                           {dU();};
    void DA(int type)                                   {dU(type);};
    void DisplayAktualisieren(int type)                 {dU(type);};
    void DA(int (&layout)[8], String errorMessage, bool clearDisplay)                                                                     {dU(layout, errorMessage, clearDisplay);};
    void DisplayAktualisieren(int (&layout)[8], String errorMessage, bool clearDisplay)                                                   {dU(layout, errorMessage, clearDisplay);};
    void DA(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8)                       {dU(layout1, layout2, layout3, layout4, layout5, layout6, layout7, layout8);};
    void DisplayAktualisieren(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8)     {dU(layout1, layout2, layout3, layout4, layout5, layout6, layout7, layout8);};
    void DT(int x_, int y_, String s_)                  {dT(x_,y_,s_);};
    void DisplayText(int x_, int y_, String s_)         {dT(x_,y_,s_);};
    void DTL()                                          {dTC();};
    void DisplayTextLoeschen()                          {dTC();};  
    void DR()                                           {dB();};
    void DisplayRand()                                  {dB();};
    void DL()                                           {dC();};
    void DisplayLoeschen()                              {dC();};

  private:
    int  _lightSensorValue(int port, int color);
    long _lightSensorPercent(int port, int color);
    void _motors(int val1 = -1000, int val2 = -1000, int val3 = -1000, int val4 = -1000, int val5 = -1000, bool signSwitch = false);
    void _getOrientation(double *vec);
    void _displayError(int error)                       {_displayError(error,0);};
    void _displayError(int error, int input);
    void _displayError(int error, _VecInt10 & variables);
    void _displaySSD1306Update(int (&layout)[8], String errorMessage, bool clearDisplay);        //for the small display in the A Variants
    void _displayILI9225Update(byte dataAndDisplayType, String errorMessage, bool clearDisplay); //for the bigger display in the B Variants
    void _displayTransmit(byte (&message)[32]);
    //Teensy 4.1 Pinout

    const int _pinout[13][4] =  {{ 7, 6, 2}       //---Motors        (3x PWM)
                                ,{ 8, 9, 3}       //      |
                                ,{11,10, 4}       //      |
                                ,{29,28, 5}       //      |
                                ,{24,25,12}       //---Internal I2C  (SCL2,SDA2) + Button
                                ,{23,15,14,13}    //1---Sensors       (3x Digital, 1x Analog)  //White, Black, Yellow, Blue
                                ,{22,38,37,36}    //2     |
                                ,{21,35,34,33}    //3     |
                                ,{20,30,31,32}    //4     |
                                ,{41,49,50,48}    //5     |
                                ,{40,52,54,53}    //6     |
                                ,{39,27,26,51}    //7     |
                                ,{16,19,18,17}};  //8---Open I2C  (SCL1,SCL,SDA,SDA1)

    static const int _sensShift = 4;                       //number needed to be added to port for mapping to the pinout (counting starts at 1)
    static const int _maxSensors = 8;
    static const int _maxSubSensors = 4;
    static const int _maxMotors = 4;
    static const int _DisplayX  = 10;
    static const int _DisplayY  =  4;

    static const int _sensorPresetsLength = 5;
    int _sensorPresets[_sensorPresetsLength][_maxSubSensors + 1] = {
              //KABLECOLOR WHITE    BLACK   YELLOW  BLUE  (GREEN -> 5V | RED -> GND)
              //PINS        1         2       5       6
            {LIGHT,     INPUT_PULLUP, OUTPUT, OUTPUT, OUTPUT},     //LIGHT
            {LIGHT_NXT, INPUT_PULLUP, GND   , VCC   , -1    },     //LIGHT_NXT
            {TOUCH_NXT, INPUT_PULLUP, GND   , -1    , -1    },     //TOUCH_NXT
            {TOUCH_EV3, -1          , -1    , -1    , INPUT },     //TOUCH_EV3
            {INFRARED,  -1          , -1    , -1    , INPUT }      //INFRARED
    };
                                                        //stores the type of a sensor and if it hasn't been       {generall sensor type, subsensortype 1..4 or I/O 1..4}        
    int _sensors[_maxSensors][_maxSubSensors + 1];      // initialized it will be -1 (only for external sensors)     {CUSTOM, LIGHT_NXT, TOUCH_NXT, TOUCH_NXT, INFRARED}
                                                                                                                      //      {LIGHT_NXT, INPUT_PULLUP,GND,VCC,-1},
    int _sensorValues[_maxSensors + 7][_maxSubSensors]; //stores the values of all sensors, the used gyroscope values the gyroscope offset values and the button
                                                        //if a value is equal to INT_MAX, then it hasn't been used and shouldn't be displayed

    //LIGHT       LIGHT_NXT       TOUCH_NXT       TOUCH_EV3      INFRARED      KABLECOLOR  (GREEN -> 5V | RED -> GND)
    //Red         NULL            NULL            Signal         Signal        BLUE    6
    //Green       Led             NULL            NULL                         YELLOW  5
    //Blue        GND             GND             NULL                         BLACK   2
    //Signal      Signal          Signal          NULL                         WHITE   1


    int _lightDelay = 1;                                  //not realy neccessary to have a higher number, as even 1 millisecond doesnt reduce the quality of the brightnesvalue
    int _motorSpeeds[_maxMotors];                         //stores the speed / direction of each motor
    bool _invertMotors[_maxMotors];                       //stores which motors should be inverted

    const int _ambassadorAddress = 4;                     //I2C Address of the Ambassador
    const byte _protocolVersion = 1;                     //Version of the transmission protocol used to communicate to the ambassador
    String _Display[_DisplayX][_DisplayY];                //stores what is supposed to be shown on the display
    bool _displayOutline = false;                         //stores if the display-Outline is supposed to be displayed
    bool _errorTriangle = false;                          //stores if the error-Triangle is supposed to be displayed
    _VecInt10 _errorVariables;
};


template<typename type> type absolute(type v){
  if(v < 0)
    return -1 * v;
  return v;
}

template double absolute(double);
template int absolute(int);
template float absolute(float);
template long absolute(long);

const unsigned char logo [] PROGMEM = {               //BitMap of the Logo
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x07, 0xff, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x1f, 0xff, 0xff, 0xff, 0x80, 0x00, 
  0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x3f, 0xff, 0xff, 0xff, 0xc0, 0x00, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3f, 0xff, 0xff, 0xff, 0xf0, 0x00, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x7f, 0xff, 0xff, 0xff, 0xf8, 0x00, 
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xf0, 
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x1f, 0xff, 0xf8, 
  0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x07, 0xff, 0xf8, 
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xe0, 0x00, 0x3f, 0xff, 0xff, 0xf8, 0x00, 0x03, 0xff, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x00, 0x01, 0xff, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0xff, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x7f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x07, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x07, 0xff, 0xff, 0x80, 0x00, 0x00, 0x3f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x3f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x03, 0xff, 0xff, 0x00, 0x00, 0x00, 0x3f, 0xff, 
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x00, 0x03, 0xff, 0xff, 0x00, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x07, 0xff, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x07, 0xff, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x0f, 0xfe, 0x7f, 0xc0, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x0f, 0xfe, 0x3f, 0xe0, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xfc, 0x3f, 0xe0, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x00, 0x3f, 0xfc, 0x1f, 0xf0, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x3f, 0xf8, 0x1f, 0xf0, 0x00, 0x00, 0x1f, 0xff, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x7f, 0xf8, 0x0f, 0xf8, 0x00, 0x00, 0x3f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x7f, 0xf0, 0x0f, 0xf8, 0x00, 0x00, 0x3f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0xff, 0xf0, 0x07, 0xfc, 0x00, 0x00, 0x3f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x01, 0xff, 0xe0, 0x07, 0xfc, 0x00, 0x00, 0x7f, 0xfe, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x01, 0xff, 0xe0, 0x03, 0xfe, 0x00, 0x00, 0x7f, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x03, 0xff, 0xc0, 0x03, 0xff, 0x00, 0x00, 0xff, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xe0, 0x07, 0xff, 0xc0, 0x03, 0xff, 0x00, 0x01, 0xff, 0xfc, 
  0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0x80, 0x03, 0xff, 0xf8, 
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x01, 0xff, 0x80, 0x07, 0xff, 0xf8, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x01, 0xff, 0xc0, 0x1f, 0xff, 0xf0, 
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0xff, 0xc0, 0x7f, 0xff, 0xf0, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0xff, 0xff, 0xff, 0xff, 0xe0, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc0, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x80, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xfc, 0x00, 
  0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xff, 0xff, 0xf0, 0x00, 
  0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xff, 0xe0, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0x80, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0x80, 0x00, 0xff, 0xff, 0xfe, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xf0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#endif
