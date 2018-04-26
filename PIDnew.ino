#include <PID_v1.h>

/********************************************************
//Arduino PID controller 2 Heat Zones, 6 readings
//Ashley Newton
//Cannula bending process

//Uses Ard
uino PID library without autotune

//Thermistor 1 controlls the heat output for heater 1
//Thermistor 2 controlls the heat output for heater 2
//4 additional thermistor pins to read temps for heat information on the face
//There are 5 comments that specify peices of code to comment or remove for only 2 thermistor sensors (normal workflow)
//Fan is controlled by the mean of Thermistor 1 and 2
//Fan helps control overheating

//When using, define setpoint 20 degrees lower than target
//Once the temperature has peaked compile again with a true setpoint

//See wiring and circuit diagrams
//See also instruction manual

 ********************************************************/
//Define Output pins
#define PIN1_OUTPUT 9                     //Heater 1
#define PIN2_OUTPUT 10                    //Heater 2
#define PIN3_OUTPUT 11`                   //Fan

//Define Input pins
#define PIN1_INPUT 0                      //Controlls Heater 1
#define PIN2_INPUT 1                      //Controlls Heater 2
#define PIN3_INPUT 2
#define PIN4_INPUT 3
#define PIN5_INPUT 4
#define PIN6_INPUT 5

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpoint2, Input2, Output2;
double Input3, Input4, Input5, Input6;

int PRINT_INTERVAL = 500;
uint32_t lastPrint = 0;

//Specify the links and initial tuning parameters
double Kp=1, Ki=.2, Kd=0;
double Kp2=1, Ki2=.2, Kd2=0;

//Give PID function 2 myPID objects and the corresponding variables
PID myPID1(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

//PID window size in milliseconds
int WindowSize = 5000;
unsigned long windowStartTime;

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(PIN1_INPUT);
  Input2 = analogRead(PIN2_INPUT);
  //comment this chunk for 2 thermistors
  Input3 = analogRead(PIN3_INPUT);
  Input4 =analogRead(PIN4_INPUT);
  Input5 =analogRead(PIN5_INPUT);
  Input6 =analogRead(PIN6_INPUT);

  pinMode(11,OUTPUT);

  //initialize serial output to window baud
  Serial.begin(9600);

  windowStartTime = millis();

  // Setpoint corresponds to first heater, Setpoint 2 with the second
  Setpoint = 95;                                                                                         //SET TEMPERATURES HERE
  Setpoint2 = 95;

  //tell the PID to range between 0 and the full window size
  myPID1.SetOutputLimits(0, WindowSize);
  myPID2.SetOutputLimits(0, WindowSize);


  //turn the PID on
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);  
}

void loop()
{
  double tempRead = analogRead(PIN1_INPUT);
  double tempK = log(10000.0 * ((1024.0 / tempRead - 1)));
  Input = (1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK ))-273.15;           //  Temp 1 in C
  double tempRead2 = analogRead(PIN2_INPUT);
  double tempK2 = log(10000.0 * ((1024.0 / tempRead2 - 1)));
  Input2 = (1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK2 * tempK2 )) * tempK2 ))-273.15;       //  Temp 2 in C

  //Comment this chunk for only 2 thermistors
  double tempRead3 = analogRead(PIN3_INPUT);
  double tempK3 = log(10000.0 * ((1024.0 / tempRead3 - 1)));
  Input3 = (1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK3 * tempK3 )) * tempK3 ))-273.15;       //  Temp 3 in C 
  double tempRead4 = analogRead(PIN4_INPUT);
  double tempK4 = log(10000.0 * ((1024.0 / tempRead4 - 1)));
  Input4 = (1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK4 * tempK4 )) * tempK4 ))-273.15;       //  Temp 4 in C
  double tempRead5 = analogRead(PIN5_INPUT);
  double tempK5 = log(10000.0 * ((1024.0 / tempRead5 - 1)));
  Input5 = (1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK5 * tempK5 )) * tempK5 ))-273.15;       //  Temp 4 in C
  double tempRead6 = analogRead(PIN6_INPUT);
  double tempK6 = log(10000.0 * ((1024.0 / tempRead6 - 1)));
  Input6 = (1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK6 * tempK6 )) * tempK6 ))-273.15;       //  Temp 4 in C

  //Run PID computations
  myPID1.Compute();
  myPID2.Compute();

  //send time proportional output to pins
  analogWrite(PIN1_OUTPUT, Output);
  analogWrite(PIN2_OUTPUT, Output2);


  //for fan on top
  
  //Calculate mean temperature for temperature reference
  double meanTemp = (Input + Input2)/2;
  //Set a threshold above the set temperature for the fan to turn on
  double thresholdFan = Setpoint + 2;                                                                 //SET FAN THRESHOLD HERE

  if (meanTemp > thresholdFan)
     digitalWrite(11, HIGH);           //turn fan on
  else
     digitalWrite(11, LOW);            //turn fan off
     

   // Only print at most every PRINT_INTERVAL milliseconds
   if(millis() >= lastPrint + PRINT_INTERVAL) {
     Serial.print(Input);
     Serial.print ("\t");
     Serial.print ("\t");
     Serial.println(Input2);
//     Serial.print ("\t");
//     Serial.print ("\t");
//     //comment this chunk for 2 thermistors
//     Serial.print(Input3);
 //    Serial.print ("\t");
//     Serial.print ("\t");
 //    Serial.print(Input4);
 //    Serial.print ("\t");
 //    Serial.print ("\t");
 //    Serial.print(Input5);
 //    Serial.print ("\t");
 //    Serial.print ("\t");
 //    Serial.println(Input6);

     
     lastPrint = millis();
   }

  
}



