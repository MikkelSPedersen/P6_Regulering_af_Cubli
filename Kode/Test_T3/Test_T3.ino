#include <FreeRTOS_SAMD21.h>
#include <VidorFPGA.h>
#include <math.h>
#include <Servo.h>

//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>

//  IMU
#define scale 65.5        //Scale for gyroscope (From datasheet)
#define scaleacc 16384.0  //Scale for accelerometer (From datasheet)
#define MPU_addr1 104     //I2C address forhe IMU. (CODE WILL BE STUCK IF NOT CORRECT!
#define MPU_addr2 105     //I2C address for 2nd IMU  (AGAIN CODE WILL BE STUCK IF NOT CORRECT!)

//  Sampling
float angleRef = 0.035;   // 0.02 for IMU1, 0.05 for IMU2 og 0.035 for begge
float ang = 0;
float ang2 = 0;
float ang_filt_old = 0;
int samplingperiod = 10;//8
float speedWheel;
float speedFrame;

//  Servo
Servo brake;
#define brk 95  //Servo position when braking
#define go 110   //Servo position when running

//  Referense justering
#define step_size 0.000005
//#define step_size 0.00001
#define large_step_size 0.0001
int x = 0;
int reject = 0;
int N = 50;
float adjustVal[50];
float adjustAverage = 0.0;

//  Generel
#define RPM2Rad 2 * M_PI / 60
enum State {
  Off,
  Running,
  Falling,
  Fallen,
  Ready,
  Stabilizing
};
String StateString(State state) {
  String returnString = "";
  switch (state) {
    case Off:
      returnString = "Off";
      break;
    case Running:
      returnString = "Running";
      break;
    case Falling:
      returnString = "Falling";
      break;
    case Fallen:
      returnString = "Fallen";
      break;
    case Ready:
      returnString = "Ready";
      break;
    case Stabilizing:
      returnString = "Stabilizing";
      break;
    default:
      // statements
      break;
  }
  return returnString;
}
State CubliState = Ready;
//State CubliState = Running;

// Debug
#define TestPin 4
//#define configUSE_PREEMPTION 0

//  FreeRTOS

void ControllerTask( void *pvParameters );
void SamplingTask( void *pvParameters );
void RecoveryTask( void *pvParameters );

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(A1, INPUT);         // Potentiometer
  analogReadResolution(12);
  pinMode(A5, INPUT);         // Hjul hastighed
  pinMode(1, OUTPUT);         // Driver enable
  pinMode(TestPin, OUTPUT);   // Digital pin til debug af eksekveringstid
  digitalWrite(1, LOW);       // Sikre driveren er slukket
  // Opsætning af PWM til driveren
  FPGA.begin();
  FPGA.analogWriteResolution(10, 4700);
  FPGA.pinMode(45, 3);
  FPGA.analogWrite(45, map(50, 0, 100, pow(2, 10), 0));
  //
  brake.attach(2);            // Initializere servo
  brake.write(go);            // Skriver vinklen for "go"
  // Opsætning af IMU
  Wire.begin();
  IMU_Setup(MPU_addr1);
  IMU_Setup(MPU_addr2);
  //Wire.end();
  start_ang();
  //
  digitalWrite(1, HIGH);      // Tænder for driveren

  // FreeRTOS
  xTaskCreate(
    ControllerTask,         // Function that should be called
    "Controller",           // Name of the task (for debugging)
    1000,                   // Stack size (bytes)
    NULL,                   // Parameter to pass
    3,                      // Task priority (max:3 low:0)
    NULL                    // Task handle
  );
  xTaskCreate(SamplingTask, "Sampling", 1000, NULL, 3, NULL);
  xTaskCreate(RecoveryTask, "Recovery", 1000, NULL, 2, NULL);

  vTaskStartScheduler();        // starter skeduleringen
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop()
{
}
//*****************************************************************

//*****************************************************************
// Create a thread that prints out B to the screen every second
// this task will run forever
//*****************************************************************
void ControllerTask( void *pvParameters )
{
  //digitalWrite(TestPin, HIGH);
  //float k1 = -0.0005;    //for reasons unknown
  //float k2 = -1.0;
  //float k3 = -0.1;

  // Rigtig rækkefølge
  float k1 = -1.1536;
  float k2 = -0.1998;
  float k3 = -0.0015;
  
  //float k1 = -0.0009;
  //float k2 = -1.3381;
  //float k3 = -0.1594;
  

  //float k1 = -0.0018077;//-0.0316       //SpeedOfWheel
  //float k2 = -15.113;//-15.6593      //AngleError
  //float k3 = -0.5296;//-0.5             //SpeedOfFrame
  float kt = 0.0335;
  float tau_m = 0;
  float angError = 0;
  int stabile = 0;

  TickType_t xLastWakeTime;
  const TickType_t xPeriod = samplingperiod;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // Afventer næste gang tasken skal køre
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    //digitalWrite(TestPin, HIGH);

    //Serial.println(StateString(CubliState));
    if (CubliState == Running || CubliState == Stabilizing) {

      // Bestemmer hvilken vinkelmåling der anvendes
      if (CubliState == Running) {
        digitalWrite(TestPin, HIGH);      //Trigger
        angError = ang - angleRef;
      } else if (CubliState == Stabilizing) {
        angError = ang2;
        stabile ++;
        if (stabile >= 1000 / samplingperiod) {
          CubliState = Running;
          stabile = 0;
        }
      }

      // Diskretiseret controller og relaterede implementering
      tau_m = k1 * angError + k2 * speedFrame + k3 * speedWheel;
      float current = (tau_m / kt);
      if (current >= 7.5) current = 7.49;
      else if (current <= -7.5) current = -7.49;
      float duty = interpolate(current, -7.5, 7.5, 90, 10);
      float pwm = map(duty, 0, 100, pow(2, 10), 0);
      FPGA.analogWrite(45, pwm);

      adjust(); // Tilpasning af reference

      // Skift state
      if (ang >= 0.66 || ang <= -0.60) { // var på abs(ang) >= 0.6 før
        FPGA.analogWrite(45, map(50, 0, 100, pow(2, 10), 0));
        CubliState = Falling;
      }
    }
    else if (CubliState == Falling) {
      if (ang >= 0.81 || ang <= -0.75) { // var på abs(ang) >= 0.77 før
        if (abs(speedWheel) < 1000 * RPM2Rad) brake.write(brk); //var 200, men meget langsomt
        if (abs(speedWheel) <= 20 * RPM2Rad) {
          CubliState = Fallen;
        }
      }
    }
    else if (CubliState == Fallen) {
      brake.write(go);
      CubliState = Ready;
      digitalWrite(TestPin, LOW);   //trigger
    }
    //digitalWrite(TestPin, LOW);
  }
}
//*****************************************************************

//*****************************************************************
// Create a thread that prints out B to the screen every second
// this task will run forever
//*****************************************************************
void SamplingTask( void *pvParameters )
{
  float prevAng = 0;

  TickType_t xLastWakeTime;
  const TickType_t xPeriod = samplingperiod;
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    // Afventer næste gang tasken skal køre
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    //digitalWrite(TestPin, HIGH);

    //  Potentiometer
    ang2 = (float)(analogRead(A1) - 1991.0) * (90.0 / 4096.0) * (M_PI / 180); //1991
    //  IMU
    ang = complementaryFilter();
    speedWheel = hastighedHjul(); // hvorfor er det er egentlig en funktion?

    //Serial.print("Ang: ");
    //Serial.print(ang);
    //Serial.print("; Hjul: ");
    //Serial.println(speedWheel);
    
    //digitalWrite(TestPin, LOW);
  }
}
//*****************************************************************

//*****************************************************************
// Create a thread that prints out B to the screen every second
// this task will run forever
//*****************************************************************
void RecoveryTask( void *pvParameters )
{
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = 60;
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    // Afventer næste gang tasken skal køre
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    //digitalWrite(TestPin, HIGH);
    if (CubliState == Ready) {
      oprejsning();
    }

    //digitalWrite(TestPin, LOW);
  }
}
//*****************************************************************
