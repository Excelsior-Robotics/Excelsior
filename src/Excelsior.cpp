#include <Arduino.h>
#include <Excelsior.h>

using namespace std;
//------SETUP------------------
Excelsior::Excelsior(String _ExcelsiorVersion) : display(128, 64, &Wire2) , bno055(55,0x28,&Wire2), ExcelsiorVersion(_ExcelsiorVersion){
  Serial.begin(9600);
  delay(1000);
  Serial.println("Hello-Excelsior");

  for(int i = 0; i < _maxSensors; i++){
      _sensors[i][0] = -1;
    for(int j = 0; j < _maxSubSensors; j++){
      _sensors[i][j + 1] = -1;                            //initiallises array as not initialised
      _sensorValues[i][j] = INT_MAX;                  //initiallises array as empty
      digitalWrite(_pinout[_sensShift + i][j], LOW);  //avoids the OUTPUT HIGH state (pullup resistor still enabled) 
      pinMode(_pinout[_sensShift + i][j],OUTPUT);     //sets every pin as output
    }
  }

  pinMode(_pinout[_sensShift][2], INPUT);     //internal Button

  for(int i = 0; i < _maxMotors; i++){
    pinMode(_pinout[i][0],OUTPUT);      //functions as digital port   (direction)
    pinMode(_pinout[i][1],OUTPUT);      //functions as digital port   (direction)
    pinMode(_pinout[i][2],OUTPUT);      //functions as PWM port       (speed)
  }

  delay(100);                           // This delay is needed to let the display to initialise
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize display with the I2C address of 0x3C
  display.clearDisplay();               // Clear the buffer
  display.setTextColor(SSD1306_WHITE);  // Set color of the text
  display.setRotation(0);               // Set orientation. Goes from 0, 1, 2 or 3
  display.setTextWrap(true);            // By default, long lines of text are set to automatically “wrap” back to the leftmost column.
                                        // To override this behavior (so text will run off the right side of the display - useful for
                                        // scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored
                                        // with setTextWrap(true).
  display.dim(0);                       //Set brightness (0 is maximun and 1 is a little dim)

  Wire2.begin();
  if(!bno055.begin()){
    _displayError(-1,0);        //Error message: Gyro not found
  }
  delay(100);                   //short wait, to initialize the bno055
  gyroReset();
  displayUpdate(-1);            //Shows default display
}

//------SENSOR SETUP------------------
void Excelsior::sensorSetup(int port, int type){
  switch (type) {     //sensorSetup(int, int , int[4]) checks if the port is valid, therefor not necessary here
    case LIGHT:       sensorSetup(port, _sensorPresets[0]); break;
    case LIGHT_NXT:   sensorSetup(port, _sensorPresets[1]); break;
    case TOUCH_NXT:   sensorSetup(port, _sensorPresets[2]); break;
    case TOUCH_EV3:   sensorSetup(port, _sensorPresets[3]); break;
    case INFRARED:    sensorSetup(port, _sensorPresets[4]); break;
    default:  _displayError(-3,type); break;    //ERROR: SensorTYPE not defined
  }
}


void Excelsior::sensorSetup(int port, int pin, int IOtype){
  if(port < 1 || port > _maxSensors){
    _displayError(-2,port);      //ERROR: SensorPORT not defined
    return;
  }
  if(IOtype < LIGHT || IOtype > ANALOG_IN){
    _displayError(-3,IOtype);   //ERROR: SensorTYPE not defined
    return;
  }
  int pinindex = -1;
  switch(pin){              //converts the pinout from the cable into the index in the array
    case 1: pinindex = 0; break;
    case 2: pinindex = 1; break;
    case 5: pinindex = 2; break;
    case 6: pinindex = 3; break;
    default: _displayError(-10,pin); return;      //ERROR: SensorPIN not defined
  }
  _sensors[port - 1][0] = CUSTOM;
  _sensors[port - 1][pinindex + 1] = IOtype;
  sensorSetup(port, _sensors[port - 1]);
}

void Excelsior::sensorSetup(int port, int IOtype1, int IOtype2, int IOtype3, int IOtype4){
  int IOtypes[5] = {CUSTOM, IOtype1, IOtype2, IOtype3, IOtype4};
  sensorSetup(port, IOtypes);
}

void Excelsior::sensorSetup(int port, int (&types)[5]){
  if(port < 1 || port > _maxSensors){
    _displayError(-2,port);      //ERROR: SensorPORT not defined
    return;
  }
  if(types[0] < LIGHT || types[0] > CUSTOM){
    _displayError(-3,types[0]);     //ERROR: SensorTYPE not defined
    return;
  }
  _sensors[port - 1][0] = types[0];

  Serial.print("PORT: "); //TEMP
  Serial.print(port); //TEMP
  Serial.print("  type: "); //TEMP
  Serial.print(types[0]); //TEMP

  for(int i = 0; i < _maxSubSensors; i++){
    _sensors[port - 1][i + 1] = types[i + 1];
    _sensorValues[port - 1][i] = INT_MAX;     //if not yet read or is a GND Pin, then it is not to be displayed

    Serial.print("\t IOtype: ");  //TEMP
    Serial.print(types[i + 1]); //TEMP

    int IOtype = OUTPUT;
    int output = LOW;
    
    switch (types[i + 1]) {
      case DIGITAL_IN:
      case ANALOG_IN:
      case TOUCH_EV3:
      case INFRARED:
      case INPUT:         IOtype = INPUT; output = LOW; break;
      case LIGHT_NXT:
      case TOUCH_NXT:
      case INPUT_PULLUP:  IOtype = INPUT_PULLUP; output = HIGH; break;
      case VCC:           output = HIGH; break;
      case GND:
      case DIGITAL_OUT:
      case ANALOG_OUT:           
      default:            output = LOW; break;
    }
    digitalWrite(_pinout[_sensShift + port][i], LOW);       //set to LOW to avoid the OUTPUT HIGH state if the pin was previously set to INPUT_PULLUP
    pinMode(_pinout[_sensShift + port][i], IOtype);         //set I/O type
    digitalWrite(_pinout[_sensShift + port][i], output);    //sets OUTPUT low/high | INPUT no-pullup/pullup
  }
  
  Serial.println(); //TEMP

}


void Excelsior::lightDelay(int delay){
  _lightDelay = delay;
}

//------DRIVING MOTORS------
void Excelsior::motor(int port, int dir){
  if(port < MOTOR_A || port >= (MOTOR_A + _maxMotors)){
    _displayError(-5,port);     //ERROR: MotorPORT not defined
  }else if(dir < -255 || dir > 255){
    _displayError(-6,dir);      //ERROR: Speed not defined
  }else{
    _motorSpeeds[port - MOTOR_A] = dir;
    digitalWrite(_pinout[port - MOTOR_A][0], dir < 0? HIGH:LOW);   //if dir == 0, then both go LOW (motor off)
    digitalWrite(_pinout[port - MOTOR_A][1], dir > 0? HIGH:LOW);   //else if dir determines direction of rotation
    analogWrite (_pinout[port - MOTOR_A][2], abs(dir));            //takes the absolute value to determine rotation speed
  }
}

void Excelsior::motorOff(){                        //turns off all motors
  for(int i = 0; i < _maxMotors; i++){
    motorOff(MOTOR_A + i);
  }
}

//------WRITING TO CUSTOM SENSORS------

void Excelsior::sensorWrite(int port, int pin, int signal){
  if(port < 1 || port > _maxSensors){
    _displayError(-2,port);                     //ERROR: SensorPORT not defined
    return;
  }
  if(_sensors[port - 1][0] != CUSTOM){
    _displayError(-11,port);                    //ERROR: Sensor does not support the write command
    return;
  }
  int pinIndex = -1;                            //PIN needs to be converted into array index --> pin[1,2,5,6] to pinIndex[0,1,2,3]
  switch (pin){
    case 1: pinIndex = 0; break;
    case 2: pinIndex = 1; break;
    case 5: pinIndex = 2; break;
    case 6: pinIndex = 3; break;
    default: _displayError(-10, pin);  return; //ERROR: PIN not defined
  }
  switch (_sensors[port - 1][pinIndex + 1]){
    case VCC:
    case GND:
    case DIGITAL_OUT:
    case OUTPUT:  digitalWrite(_pinout[_sensShift + port][pinIndex], signal); break;
    case DIGITAL_IN: analogWrite(_pinout[_sensShift + port][pinIndex], signal); break;
    default: return;
  }
}

//------READING SENSORS------
bool Excelsior::button(){
  _sensorValues[_maxSensors + 6][0] = !digitalRead(_pinout[_sensShift][2]);
  _displayOutline = _sensorValues[_maxSensors + 6][0];
  return _sensorValues[_maxSensors + 6][0];
}

int Excelsior::sensorRead(int port){
  //Port Valid?
  //What type of sensor is it
    // --> if Light, read as Light (save value to first of sensor array and return that)
    // --> if Preset, read as Preset (save value to first of sensor array and return that)
    // --> if CUSTOM (has subsensors)
      //go through array and for every "InputType" read the sensor depending on that type and store in sensor array
    
    //return first of sensor array

  if(port < 1 || port > _maxSensors){
    _displayError(-2,port);                     //ERROR: SensorPORT not defined
    return -1;
  }
  switch (_sensors[port - 1][0]){               //holds the sensor type
    case LIGHT:     _sensorValues[port - 1][0] = sensorRead(port, OFF); break;
    case LIGHT_NXT: _sensorValues[port - 1][0] = sensorRead(port, OFF); break;
    case TOUCH_NXT: _sensorValues[port - 1][0] = !digitalRead(_pinout[_sensShift + port][0]); break;
    case TOUCH_EV3: _sensorValues[port - 1][0] = map(digitalRead(_pinout[_sensShift + port][3]),0,2,1,0);  break;     //0 and 1 have to be flipped because of different sensor funciton compared to the TOUCH_NXT
    case INFRARED:  {
      int pulse = pulseIn(_pinout[_sensShift + port][3], HIGH, 20000);                                  //timeout in microseconds
      _sensorValues[port - 1][0] = min(2000, max(0, 2 * (pulse - 1000))); break;                        //2000 mm is the maximum that will be returned, anything lower will be calculated
    }
    case CUSTOM: {   
                    _sensorValues[port - 1][0] = sensorRead(port, 1);     //the sensorRead need the number of the PIN
                    _sensorValues[port - 1][1] = sensorRead(port, 2);
                    _sensorValues[port - 1][2] = sensorRead(port, 5);
                    _sensorValues[port - 1][3] = sensorRead(port, 6); break;
    }

    default: _displayError(-4,port); return -1; //ERROR: Sensor not Initialised
  }
  return _sensorValues[port - 1][0];
}

int Excelsior::sensorRead(int port, int colorOrPin){
  if(port < 1 || port > _maxSensors){
    _displayError(-2,port);                       //ERROR: SensorPORT not defined
    return -1;
  }
  if(_sensors[port - 1][0] == -1){
    _displayError(-4,port);                       //ERROR: Sensor not Initialised
    return -1;
  }
  
  if(_sensors[port - 1][0] == LIGHT_NXT){         //here colorOrPin refers to the COLOR of the sensor Light (ON or OFF)
    if(colorOrPin)
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
    else
      digitalWrite(_pinout[_sensShift + port][2], LOW);
    _sensorValues[port - 1][0] = map(analogRead(_pinout[_sensShift + port][1]),0,1024,1024,0);    //sensorrange gets flipped so that low values correspond to black
    return _sensorValues[port - 1][0];
  }
  
  if(_sensors[port - 1][0] == LIGHT)              //here colorOrPin refers to the COLOR of the sensor Light
    return sensorRead(port, colorOrPin, false);
  
  if(_sensors[port - 1][0] == CUSTOM){            //here colorOrPin refers to the PIN that is connected
    int pinIndex = -1;                            //PIN needs to be converted into array index --> pin[1,2,5,6] to pinIndex[0,1,2,3]
    switch (colorOrPin){
      case 1: pinIndex = 0; break;
      case 2: pinIndex = 1; break;
      case 5: pinIndex = 2; break;
      case 6: pinIndex = 3; break;
      default: _displayError(-10, colorOrPin);  return -1; //ERROR: PIN not defined
    }
    switch (_sensors[port - 1][pinIndex + 1]){    //CAUTION: _sensors has SubSensors starting at index 1 not index 0
      case INPUT:                                 //INPUT and INPUT_PULLUP are treated as analog sensors
      case INPUT_PULLUP:  
      case ANALOG_IN:   _sensorValues[port - 1][pinIndex] = analogRead(_pinout[_sensShift + port][pinIndex]); break;
      case DIGITAL_IN:  _sensorValues[port - 1][pinIndex] = digitalRead(_pinout[_sensShift + port][pinIndex]); break;
      case LIGHT_NXT:   _sensorValues[port - 1][pinIndex] = map(analogRead(_pinout[_sensShift + port][pinIndex]),0,1024,1024,0);  break;  //sensorrange gets flipped so that low values correspond to black
      case TOUCH_NXT:   _sensorValues[port - 1][pinIndex] = !digitalRead(_pinout[_sensShift + port][pinIndex]); break;
      case TOUCH_EV3:   _sensorValues[port - 1][pinIndex] = map(digitalRead(_pinout[_sensShift + port][pinIndex]),0,2,1,0);  break;
      case INFRARED: {
        int pulse = pulseIn(_pinout[_sensShift + port][pinIndex], HIGH, 20000);
        _sensorValues[port - 1][pinIndex] = min(2000, max(0, 2 * (pulse - 1000))); break;
      }
      default: _sensorValues[port - 1][pinIndex] = INT_MAX; break;    //if not an INPUT type, then the value is set to INT_MAX so it can be ignored
    }
    return _sensorValues[port - 1][pinIndex];
  }
  return -1;
}

int Excelsior::sensorRead(int port, int color, bool percent){
  if(port < 1 || port > _maxSensors){            //looks if the given port is not part of the possible ports
    _displayError(-2,port);                      //ERROR: SensorPORT not defined
  }else if(_sensors[port - 1][0] == -1){
    _displayError(-4,port);                      //ERROR: Sensor not Initialised
  }else{
    if(_sensors[port - 1][0] == LIGHT){
      _sensorValues[port - 1][0] = percent? _lightSensorPercent(port,color) : _lightSensorValue(port,color);
      Serial.println(_sensorValues[port - 1][0]);
      return _sensorValues[port - 1][0];
    }else if(_sensors[port - 1][0] == LIGHT_NXT){
      _sensorValues[port - 1][0] = percent? map(sensorRead(port,color),0,1024,0,100) : sensorRead(port,color);
      return _sensorValues[port - 1][0];
    }
    return -1;
  }
  return -1;
}

int Excelsior::_lightSensorValue(int port, int color){             //gets the "raw" sensor-value
  switch(color){                                       //defines what color the sensor glows
    case WHITE:
      digitalWrite(_pinout[_sensShift + port][3], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      delay(_lightDelay);                               //small delay to make sure the colors have changed
      return (1024 - analogRead(_pinout[_sensShift + port][0]));       //subtracts sensor-value from the maximum value returned by analogRead + 1 --> before: WHITE(0 - 1023)BLACK ; after: WHITE(1024 - 1)BLACK
    case RED:
      digitalWrite(_pinout[_sensShift + port][3], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][0]));
    case GREEN:
      digitalWrite(_pinout[_sensShift + port][3], LOW);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][0]));
    case BLUE:
      digitalWrite(_pinout[_sensShift + port][3], LOW);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][0]));
    case CYAN:
      digitalWrite(_pinout[_sensShift + port][3], LOW);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][0]));
    case MAGENTA:
      digitalWrite(_pinout[_sensShift + port][3], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][0]));
    case YELLOW:
      digitalWrite(_pinout[_sensShift + port][3], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][0]));
    case OFF:
      digitalWrite(_pinout[_sensShift + port][3], LOW);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][0]));
    default: _displayError(-7,color); return -1;
  }
}

long Excelsior::_lightSensorPercent(int port, int color){
  int _red,_green,_blue,_cyan,_magenta,_yellow;             // "__" to avoid possible Redefinitions in the main programm
  switch(color){
    case OFF:
      return map(_lightSensorValue(port,OFF),0,1024,0,100);
    case WHITE:
      return map(_lightSensorValue(port,WHITE),0,1024,0,100);
    case RED:
      _red   = _lightSensorValue(port,RED);
      _green = _lightSensorValue(port,GREEN);
      _blue  = _lightSensorValue(port,BLUE);
      _lightSensorValue(port,OFF);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _red * 100L / (_red + _green + _blue);         //the L defines the output as the datatype long, allowing bigger values
    case GREEN:
      _red   = _lightSensorValue(port,RED);
      _green = _lightSensorValue(port,GREEN);
      _blue  = _lightSensorValue(port,BLUE);
      _lightSensorValue(port,OFF);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _green * 100L / (_red + _green + _blue);
    case BLUE:
      _red   = _lightSensorValue(port,RED);
      _green = _lightSensorValue(port,GREEN);
      _blue  = _lightSensorValue(port,BLUE);
      _lightSensorValue(port,OFF);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _blue * 100L / (_red + _green + _blue);
    case CYAN:
      _cyan     = _lightSensorValue(port,CYAN);
      _magenta  = _lightSensorValue(port,MAGENTA);
      _yellow   = _lightSensorValue(port,YELLOW);
      _lightSensorValue(port,OFF);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _cyan * 100L / (_cyan + _magenta + _yellow);
    case MAGENTA:
      _cyan     = _lightSensorValue(port,CYAN);
      _magenta  = _lightSensorValue(port,MAGENTA);
      _yellow   = _lightSensorValue(port,YELLOW);
      _lightSensorValue(port,OFF);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _magenta * 100L / (_cyan + _magenta + _yellow);
    case YELLOW:
      _cyan     = _lightSensorValue(port,CYAN);
      _magenta  = _lightSensorValue(port,MAGENTA);
      _yellow   = _lightSensorValue(port,YELLOW);
      _lightSensorValue(port,OFF);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _yellow * 100L / (_cyan + _magenta + _yellow);
    default: _displayError(-7,color); return -1;
  }
}

//------BNO055------
void Excelsior::_getOrientation(double *vec){
  imu::Quaternion quat = bno055.getQuat();
  quat.normalize();
  imu::Vector<3> euler = quat.toEuler();

  vec[0] = - euler.x() * 180/M_PI;
  vec[1] =   euler.y() * 180/M_PI;
  vec[2] = - euler.z() * 180/M_PI;

  return;
}

int Excelsior::gyroRead(int axis){    //0,1,2 --> The returned and displayed Values ;  3,4,5 --> The offset of the actual Value and the desired Value
  double orientation[3];
  _getOrientation(orientation);      //fetches the orientation data of the Gyroscopesensor

  for(int i = 0; i < 3; i++){         //turns the normal range of -180 to +180 to an infinte range for more easy usage of the angles
    double lastOrientation =  _sensorValues[_maxSensors + i][0] - _sensorValues[_maxSensors + i + 3][0];
    if(lastOrientation - orientation[i] > 200)
      _sensorValues[_maxSensors + i + 3][0] += 360;
    else if(lastOrientation - orientation[i] < -200)
      _sensorValues[_maxSensors + i + 3][0] -= 360;
    _sensorValues[_maxSensors + i][0] = orientation[i] + _sensorValues[_maxSensors + i + 3][0];
  }

  if(axis >= GYRO_X && axis <= GYRO_Z)                             //looks if axis is between X and Z
    return _sensorValues[_maxSensors + axis - GYRO_X][0];
  _displayError(-8,axis);
  return -1;
}

void Excelsior::gyroReset(int axis, bool toOriginal){          //Resets the Gyroscope Values (if toOriginal --> reverts back to the actual gyroscope Values by setting the offsets to 0)
  double orientation[3];
  _getOrientation(orientation);      //fetches the orientation data of the Gyroscopesensor

  switch(axis){
    case GYRO_X:  _sensorValues[_maxSensors + 3][0] = toOriginal? 0 : - orientation[0];
                  break;

    case GYRO_Y:  _sensorValues[_maxSensors + 4][0] = toOriginal? 0 : - orientation[1];
                  break;

    case GYRO_Z:  _sensorValues[_maxSensors + 5][0] = toOriginal? 0 : - orientation[2];
                  break;

    default:      _sensorValues[_maxSensors + 3][0] = toOriginal? 0 : - orientation[0];
                  _sensorValues[_maxSensors + 4][0] = toOriginal? 0 : - orientation[1];
                  _sensorValues[_maxSensors + 5][0] = toOriginal? 0 : - orientation[2];
                  break;
  }

  for(int i = 0; i < 3; i++){
    _sensorValues[_maxSensors + i][0] = orientation[i] + _sensorValues[_maxSensors + i + 3][0];
  }
}

//------OLED DISPLAY------------------
void Excelsior::displayText(int x_, int y_, String s_){
  if(x_ >= 0 && x_ < _DisplayX && y_ >= 0 && y_ < _DisplayY){
    _Display[x_][y_] = s_;
  }else{
    _errorVariables.push_back(x_);
    _errorVariables.push_back(y_);
    _displayError(-9,_errorVariables);
  }
}

void Excelsior::displayBorder(){
    _displayOutline = !_displayOutline;
}

void Excelsior::_displayError(int error, int input){
  _errorVariables.push_back(input);
  _displayError(error,_errorVariables);
}

void Excelsior::_displayError(int error, _VecInt10 & variables){
  String errorMessage = "";
  int layout[8];
  switch(error){
    case -1:  errorMessage = (String) "Gyrosensor \n   nicht \n gefunden!"; break;
    case -2:  errorMessage = (String) "Sensorport \n " + variables[0] + " nicht \n definiert"; break;
    case -3:  errorMessage = (String) " Sensorart \n " + variables[0] + " nicht \n definiert"; break;
    case -4:  errorMessage = (String) "Sensorport \n" + variables[0] + " nicht in-\nitialisiert"; break;
    case -5:  errorMessage = (String) " Motorport \n " + variables[0] + " nicht \n definiert"; break;
    case -6:  errorMessage = (String) "Geschwin-\ndigkeit " + variables[0] + "\nundefiniert"; break;
    case -7:  errorMessage = (String) "Lichtfarbe\n " + variables[0] + " nicht \n definiert"; break;
    case -8:  errorMessage = (String) "Gyro-Achse\n " + variables[0] + " nicht \n definiert"; break;
    case -9:  errorMessage = (String) "DisplayX/Y\n (" + variables[0] + "," + variables[1] + ")\nundefiniert"; break;
    case -10: errorMessage = (String) " Sensorpin \n " + variables[0] + " nicht \n definiert"; break;
    case -11: errorMessage = (String) "Sensor " + variables[0] + "\nunterstützt\nkein write"; break;
    default:  errorMessage = (String) "   Nicht\ndefinierter\n   Fehler"; break;
  }
  _errorVariables.clear();
  displayUpdate(layout,errorMessage);
}

void Excelsior::displayUpdate(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8){
  int layout[] =  {layout1,layout2,layout3,layout4,layout5,layout6,layout7,layout8};
  displayUpdate(layout, "");
}

void Excelsior::displayUpdate(int type){     //definiert presets
  int layout[8];
  switch(type){
    case 0:                         //shows all sensors
      layout[0] = 1;
      layout[1] = 2;
      layout[2] = 3;
      layout[3] = 4;
      layout[4] = 5;
      layout[5] = 6;
      layout[6] = 7;
      layout[7] = 8;
      break;
    case 1:                         //shows all motors and gyroscopes
      layout[0] = MOTOR_A;
      layout[1] = MOTOR_B;
      layout[2] = MOTOR_C;
      layout[3] = MOTOR_D;
      layout[4] = GYRO_X;
      layout[5] = GYRO_Y;
      layout[6] = GYRO_Z;
      layout[7] = 0;                //last entry is not displayed
      break;
    case 2:                         //shows custom text
      layout[0] = -2; break;

    case 3:                         //displays NOTHING
      for(int i = 0; i < 8; i++){
        layout[i] = 0;
      }
      break;

    default:                        //default displays "Excelsior"
      layout[0] = -1;      break;
  }
  displayUpdate(layout, "");
}

void Excelsior::displayUpdate(int (&layout)[8], String errorMessage){   //selects which Display is used
  byte dataAndDisplayType = 1;
  if(ExcelsiorVersion == "B")
    _displayILI9225Update(dataAndDisplayType, errorMessage);
  else                                                                  //if ExcelsiorVersion is A or something else
     _displaySSD1306Update(layout, errorMessage);
}

void Excelsior::_displaySSD1306Update(int (&layout)[8], String errorMessage){          //array of length 8 that determines the order of displayed entries (only takes arrays of this length)
  display.clearDisplay();                                        // Clear the display so we can refresh
  display.setFont(&FreeMono9pt7b);                               // Set a custom font

  if(errorMessage != ""){                                        //Display Error Message if errorMessage exists
    _errorTriangle = true;
    display.setTextSize(0);
    display.setCursor(0,10);
    display.println(errorMessage);

  }else if(layout[0] == -1){                                     //-1 at the first index displays logo
    display.drawBitmap(0,0, logo, 128, 64, SSD1306_WHITE);

  }else if(layout[0] == -2){                                     //-2 at the first index enables custom text
    display.setTextSize(0);
    for(int y = 0; y < _DisplayY; y++){
      for(int x = 0; x < _DisplayX; x++){
        display.setCursor(1 + 12*x, 12 + 16*y);
        display.println(_Display[x][y]);
      }
    }
  
  }else{
    for(int i = 0; i < 8; i++){                                  //shows all sensorvalues -> if less then 8 ports, then the last three are the gyroscope values
      if(layout[i] == 0)
        continue;                                                //displays NOTHING if the entry is 0

      display.setTextSize(0);
      display.drawRoundRect((i < 4)? 0:65,  0 + 16*(i % 4), 13, 15, 1, SSD1306_WHITE);
      int positionBoxX = (i < 4)? 1:66;        //Shows the Character in the square Box
      int positionValueX = (i < 4)? 17:82;     //Shows the value after the Box
      int positionY = 12 + 16*(i % 4);

      display.setCursor(positionBoxX, positionY);
      if(layout[i] >= 1 && layout[i] <= 8){                      //if the Sensors are displayed ( 1 - 8)
        display.println(layout[i]);
        display.setCursor(positionValueX, positionY);
        String values = "";               //prints the subvalues of the sensor, if they exist
        for(int j = 0; j < _maxSubSensors; j++){
          values += (_sensorValues[layout[i] - 1][j] == INT_MAX) ? "": (values == "" ? _sensorValues[layout[i] - 1][j] : (String) "," + _sensorValues[layout[i] - 1][j]);
        }
        display.println(values);          //corrects the of by one input

      }else if(layout[i] >= MOTOR_A && layout[i] < MOTOR_A + _maxMotors){    //if the Motors are displayed
        display.println(char('A' + layout[i] - MOTOR_A));
        display.setCursor(positionValueX, positionY);
        display.println(_motorSpeeds[layout[i] - MOTOR_A]);

      }else if(layout[i] >= GYRO_X && layout[i] <= GYRO_Z){      //if the Gyroscope is displayed
        display.println(char('X' + layout[i] - GYRO_X));
        display.setCursor(positionValueX, positionY);
        display.println(_sensorValues[_maxSensors + layout[i] - GYRO_X][0]);
      }
    }
  }
  if(_displayOutline){                                           //displays a outline to show if the button is pressed
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  }
  if(_errorTriangle){                                            //displays an error-symbol in the top right corner of the display to indicate, that an errorMessagehas been shown
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextSize(0);
    display.drawTriangle(127,14,113,14,120,0,SSD1306_WHITE);
    display.setCursor(115,12);
    display.println("!");
  }
  display.display();                                             // Print everything we set previously
  if(errorMessage != "")
    delay(1000);        //shows the error Message for longer
}

void Excelsior::_displayILI9225Update(byte dataAndDisplayType, String errorMessage){
  byte message[32];
  if(_protocolVersion == 1){   //length = 28 bytes
    byte line6 =  ((_motorSpeeds[0] < 0) << 7) +
                  ((_motorSpeeds[1] < 0) << 6) + 
                  ((_motorSpeeds[2] < 0) << 5) +                 
                  ((_motorSpeeds[3] < 0) << 4) + 
                   (_errorTriangle << 3);
    byte line15 = ((_sensorValues[0][0] < 0) << 7) +
                  ((_sensorValues[1][0] < 0) << 6) + 
                  ((_sensorValues[2][0] < 0) << 5) +                 
                  ((_sensorValues[3][0] < 0) << 4) + 
                  ((_sensorValues[4][0] < 0) << 3) + 
                  ((_sensorValues[5][0] < 0) << 2) +                 
                  ((_sensorValues[6][0] < 0) << 1) + 
                   (_sensorValues[7][0] < 0); 


    message[0] = _protocolVersion;
    message[1] = dataAndDisplayType;
    for(int i = 0; i < _maxMotors; i++)
      message[2 + i] = abs(_motorSpeeds[i]);
    message[6] = line6;
    for(int i = 0; i < _maxSensors; i++)
      message[7 + i] = abs(_sensorValues[i][0]);
    message[15] = line15;
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 4; j++){
        message[16 + i*4 + j] = _sensorValues[i + _maxSensors][0] >> ((3 - j)*8);
      }
    }

    _displayTransmit(message);

    if(errorMessage != ""){
      /*
      message[0] = _protocolVersion;
      message[1] = 0;                 //ErrorMessage
      message[2] = errorMessage.length();
      for(unsigned int i = 0; i < errorMessage.length(); i++){
        message[3 + i] = errorMessage.charAt(i);
      }
      */
    }
  }
}

void Excelsior::_displayTransmit(byte (&message)[32]){
  Wire.beginTransmission(_ambassadorAddress);   //sets to which I2C address the transmission goes
  for(int i = 0; i < 28; i++){
    Wire.write(message[i]);              // sends one byte    
  }
  Wire.endTransmission();    // stop transmitting
  delay(100);                //DELAY that should be replaced by a timer
}
