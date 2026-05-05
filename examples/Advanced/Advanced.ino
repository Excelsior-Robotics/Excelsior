#include <Excelsior.h>
Excelsior E;

void setup()
{
  E.SensorSetup(1,TAST_NXT);      //Initialisierung des Tastsensors auf SensorPort 1
  E.SensorSetup(2,LICHT);         //Initialisierung des lichtsensors auf SensorPort 2
  E.SetSensor(3,LICHT);           //Initialisierung des lichtsensors auf SensorPort 2
}

void loop()
{
  E.Motor(MOTOR_A, 50);
  E.Motor(MOTOR_B, 100);
  E.Motor(MOTOR_C, 10);
  E.MotorInvertieren(MOTOR_A,MOTOR_C);
  if(E.SensorWert(1) == false)
  {
    E.SensorWert(2);          //Wird aufgerufen, damit die Werte
    E.GyroWert(GYRO_X);       //auf dem Display auch aktualisiert werden
    E.DisplayAktualisieren(1, MOTOR_A, MOTOR_B, MOTOR_C, 2, GYRO_X, GYRO_Y, GYRO_Z);     //Zeigt eine selbst definierte Ansicht an, bei der auf den 8 Stellen des Displays
                                                                                         //nun die Geschwindigkeiten von Motor A - C, die drei Gyroskopwerte
                                                                                         //und die Sensorwerte von Sensorport 1 und 2 angezeigt
    E.MotorFwd(MOTOR_A,MOTOR_B,100);
    delay(1000);
    E.MotorRev(MOTOR_B,MOTOR_C,MOTOR_A,50);
    delay(1000);
    E.MotorAus(MOTOR_C);
    delay(1000);
    E.MotorAus();
    E.invertMotor(MOTOR_A);
  }
  else
  {
    E.DisplayText(1,1,"Tastsensor \n gedrueckt!");       //Zeigt anstelle der Übersicht benutzerdefinierten Text an. Die beiden Ganzzahlen geben die Startposition des Texts
    E.DisplayAktualisieren(2);                           //auf dem Display an. Das "\n" erzeugt einen Zeilenumbruch. Mit dem Parameter 2 wird der Text auf dem Display angezeigt
    delay(1000);
    E.DisplayLoeschen();
    delay(1000);
    E.DisplayText(1,2,"Hier ist \n nochmal \n Text");
    E.DisplayAktualisieren(2);                           
    delay(1000);
    E.DisplayTextLoeschen();
    E.DisplayAktualisieren(2);                           
    delay(1000);
  }
  delay(100);
}