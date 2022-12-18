#include <Arduino.h>
#include <Excelsior.h>

using namespace std;
//------SETUP------------------
Excelsior::Excelsior() : display(128, 64, &Wire2) , bno055(55,0x28,&Wire2){
  Serial.begin(9600);
  delay(1000);
  Serial.println("Hello-Excelsior");

  for(int i = 0; i < _maxSensors; i++){
    _sensors[i] = -1;                        //initiallises array as empty
  }

  pinMode(_pinout[_sensShift][2], INPUT);     //internal Button

  for(int i = 0; i < _maxMotors; i++){
    pinMode(_pinout[i][0],OUTPUT);      //functions as digital port   (direction)
    pinMode(_pinout[i][1],OUTPUT);      //functions as digital port   (direction)
    pinMode(_pinout[i][2],OUTPUT);      //functions as PWM port       (speed)
  }

  delay(100);                           // This delay is needed to let the display to initialize
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
  displayUpdate(-1);     //Shows default display
}

//------SENSOR SETUP------------------
void Excelsior::sensorSetup(int port, int type){           //Digital- / Analog-Ports
  if(port < 1 || port > _maxSensors){
    _displayError(-2,port);      //ERROR: SensorPORT not defined
  }else if(type < LIGHT || type > INFRARED){
    _displayError(-3,type);      //ERROR: SensorTYPE not defined
  }else{
      _sensors[port - 1] = type;                                                                                            //LIGHT       LIGHT_NXT       TOUCH_NXT       TOUCH_EV3      INFRARED      KABLECOLOR  (GREEN -> 5V | RED -> GND)
      pinMode(_pinout[_sensShift + port][0],(type == TOUCH_EV3 || type == INFRARED)? INPUT:OUTPUT);               //Red         NULL            NULL            Signal         Signal        BLUE
      pinMode(_pinout[_sensShift + port][1],OUTPUT);                                                             //Green       Led             NULL            NULL                         YELLOW
      pinMode(_pinout[_sensShift + port][2],OUTPUT);                                                             //Blue        GND             GND             NULL                         BLACK
      pinMode(_pinout[_sensShift + port][3],INPUT_PULLUP);                                                       //Signal      Signal          Signal          NULL                         WEIß

      digitalWrite(_pinout[_sensShift + port][0],LOW);         //if defined as INPUT, this disables the internal pullup resistor
      digitalWrite(_pinout[_sensShift + port][1],LOW);
      digitalWrite(_pinout[_sensShift + port][2],LOW);
  }
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

    Serial.print("MOTORS:");
    Serial.print(port - MOTOR_A);
    Serial.print(" - ");
    Serial.print(_pinout[port - MOTOR_A][0]);
    Serial.print(" ");
    Serial.print(_pinout[port - MOTOR_A][1]);
    Serial.print(" ");
    Serial.println(_pinout[port - MOTOR_A][2]);    
  }
}

//------READING SENSORS------
bool Excelsior::button(){
  _sensorValues[_maxSensors + 6] = !digitalRead(_pinout[_sensShift][2]);
  _displayOutline = _sensorValues[_maxSensors + 6];
  return _sensorValues[_maxSensors + 6];
}

int Excelsior::sensorRead(int port){
  if(port < 1 || port > _maxSensors){            //looks if the given port is not part of the possible ports
    _displayError(-2,port);                      //ERROR: SensorPORT not defined
  }else if(_sensors[port] == -1){
    _displayError(-4,port);                      //ERROR: Sensor not Initialised
  }
  else{
    if(_sensors[port - 1] == TOUCH_EV3){
      _sensorValues[port - 1] =  map(digitalRead(_pinout[_sensShift + port][0]),0,2,1,0);       //0 and 1 have to be flipped because of different sensor funciton compared to the TOUCH_NXT
      return _sensorValues[port - 1];
    }else if(_sensors[port - 1] == TOUCH_NXT){
      _sensorValues[port - 1] = !digitalRead(_pinout[_sensShift + port][3]);
      return _sensorValues[port - 1];
    }else if(_sensors[port - 1] == INFRARED){
      int pulse = pulseIn(_pinout[_sensShift + port][3], HIGH, 20000);                                    //timeout in microseconds
      _sensorValues[port - 1] = min(2000, max(0, 2 * (pulse - 1000)));                                               //2000 mm is the maximum that will be returned, anything lower will be calculated
      return _sensorValues[port - 1];
    }
    return sensorRead(port, OFF);
  }
  return -1;
}

int Excelsior::sensorRead(int port, int color){
  if(port < 1 || port > _maxSensors){            //looks if the given port is not part of the possible ports
    _displayError(-2,port);                      //ERROR: SensorPORT not defined
  }else if(_sensors[port] == -1){
    _displayError(-4,port);                      //ERROR: Sensor not Initialised
  }else{
    if(_sensors[port - 1] == LIGHT_NXT){
      if(color)
        digitalWrite(_pinout[_sensShift + port][1], HIGH);
      else
        digitalWrite(_pinout[_sensShift + port][1], LOW);
      _sensorValues[port - 1] = map(analogRead(_pinout[_sensShift + port][3]),0,1024,1024,0);    //sensorrange gets flipped so that low values correspond to black
      return _sensorValues[port - 1];
    }
    return sensorRead(port, color, false);
  }
  return -1;
}

int Excelsior::sensorRead(int port, int color, bool percent){
  if(port < 1 || port > _maxSensors){            //looks if the given port is not part of the possible ports
    _displayError(-2,port);                      //ERROR: SensorPORT not defined
  }else if(_sensors[port] == -1){
    _displayError(-4,port);                      //ERROR: Sensor not Initialised
  }else{
    if(_sensors[port - 1] == LIGHT){
      _sensorValues[port - 1] = percent? _lightSensorPercent(port,color) : _lightSensorValue(port,color);
      return _sensorValues[port - 1];
    }else if(_sensors[port - 1] == LIGHT_NXT){
      _sensorValues[port - 1] = percent? map(sensorRead(port,color),0,1024,0,100) : sensorRead(port,color);
      return _sensorValues[port - 1];
    }
    return -1;
  }
  return -1;
}

int Excelsior::_lightSensorValue(int port, int color){             //gets the "raw" sensor-value
  switch(color){                                       //defines what color the sensor glows
    case WHITE:
      digitalWrite(_pinout[_sensShift + port][0], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      delay(_lightDelay);                               //small delay to make sure the colors have changed
      return (1024 - analogRead(_pinout[_sensShift + port][3]));       //subtracts sensor-value from the maximum value returned by analogRead + 1 --> before: WHITE(0 - 1023)BLACK ; after: WHITE(1024 - 1)BLACK
    case RED:
      digitalWrite(_pinout[_sensShift + port][0], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][3]));
    case GREEN:
      digitalWrite(_pinout[_sensShift + port][0], LOW);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][3]));
    case BLUE:
      digitalWrite(_pinout[_sensShift + port][0], LOW);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][3]));
    case CYAN:
      digitalWrite(_pinout[_sensShift + port][0], LOW);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][3]));
    case MAGENTA:
      digitalWrite(_pinout[_sensShift + port][0], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      digitalWrite(_pinout[_sensShift + port][2], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][3]));
    case YELLOW:
      digitalWrite(_pinout[_sensShift + port][0], HIGH);
      digitalWrite(_pinout[_sensShift + port][1], HIGH);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][3]));
    case OFF:
      digitalWrite(_pinout[_sensShift + port][0], LOW);
      digitalWrite(_pinout[_sensShift + port][1], LOW);
      digitalWrite(_pinout[_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(_pinout[_sensShift + port][3]));
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
    double lastOrientation =  _sensorValues[_maxSensors + i] - _sensorValues[_maxSensors + i + 3];
    if(lastOrientation - orientation[i] > 200)
      _sensorValues[_maxSensors + i + 3] += 360;
    else if(lastOrientation - orientation[i] < -200)
      _sensorValues[_maxSensors + i + 3] -= 360;
    _sensorValues[_maxSensors + i] = orientation[i] + _sensorValues[_maxSensors + i + 3];
  }

  if(axis >= GYRO_X && axis <= GYRO_Z)                             //looks if axis is between X and Z
    return _sensorValues[_maxSensors + axis - GYRO_X];
  _displayError(-8,axis);
  return -1;
}

void Excelsior::gyroReset(int axis, bool toOriginal){          //Resets the Gyroscope Values (if toOriginal --> reverts back to the actual gyroscope Values by setting the offsets to 0)
  double orientation[3];
  _getOrientation(orientation);      //fetches the orientation data of the Gyroscopesensor

  switch(axis){
    case GYRO_X:  _sensorValues[_maxSensors + 3] = toOriginal? 0 : - orientation[0];
                  break;

    case GYRO_Y:  _sensorValues[_maxSensors + 4] = toOriginal? 0 : - orientation[1];
                  break;

    case GYRO_Z:  _sensorValues[_maxSensors + 5] = toOriginal? 0 : - orientation[2];
                  break;

    default:      _sensorValues[_maxSensors + 3] = toOriginal? 0 : - orientation[0];
                  _sensorValues[_maxSensors + 4] = toOriginal? 0 : - orientation[1];
                  _sensorValues[_maxSensors + 5] = toOriginal? 0 : - orientation[2];
                  break;
  }

  for(int i = 0; i < 3; i++){
    _sensorValues[_maxSensors + i] = orientation[i] + _sensorValues[_maxSensors + i + 3];
  }
}

//------OLED DISPLAY------------------
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
    case -4:  errorMessage = (String) "Sensorport \n " + variables[0] + " nicht ini-\ntialisiert"; break;
    case -5:  errorMessage = (String) " Motorport \n " + variables[0] + " nicht \n definiert"; break;
    case -6:  errorMessage = (String) "Geschwin-\ndigkeit " + variables[0] + "\nundefiniert"; break;
    case -7:  errorMessage = (String) "Lichtfarbe\n " + variables[0] + " nicht \n definiert"; break;
    case -8:  errorMessage = (String) "Gyro-Achse\n " + variables[0] + " nicht \n definiert"; break;
    case -9:  errorMessage = (String) "DisplayX/Y\n (" + variables[0] + "," + variables[1] + ")\nundefiniert"; break;
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

void Excelsior::displayUpdate(int (&layout)[8], String errorMessage){          //array of length 8 that determines the order of displayed entries (only takes arrays of this length)
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
        display.println(_sensorValues[layout[i] - 1]);          //corrects the of by one input

      }else if(layout[i] >= MOTOR_A && layout[i] < MOTOR_A + _maxMotors){    //if the Motors are displayed
        display.println(char('A' + layout[i] - MOTOR_A));
        display.setCursor(positionValueX, positionY);
        display.println(_motorSpeeds[layout[i] - MOTOR_A]);

      }else if(layout[i] >= GYRO_X && layout[i] <= GYRO_Z){      //if the Gyroscope is displayed
        display.println(char('X' + layout[i] - GYRO_X));
        display.setCursor(positionValueX, positionY);
        display.println(_sensorValues[_maxSensors + layout[i] - GYRO_X]);
      }
    }
  }
  if(_displayOutline){                                           //displays a outline to show if the button is pressed
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  }
  if(_errorTriangle){                                            //displays an error-simbol in the top right corner of the display to indicate, that an errorMessagehas been shown
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
