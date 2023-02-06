  //#include <analogWrite.h>
#include <HardwareSerial.h>
#include "arm_fk.h"
#include "drive.h"

//---ESP32_PIN_CONFIG---
//Arm
#define dir_swivel 4
#define pwm_swivel 18
#define dir_link1 12
#define pwm_link1 27
#define dir_link2 16
#define pwm_link2 17

//Drive
#define Rdir 23
#define Ldir 32
#define Rpwm 19
#define Lpwm 33

//Limit Switch
#define L1 34
#define L2 35
#define L3 15
#define L4 14
#define Stow 5

// SERIAL
HardwareSerial SerialPort(0); // use UART0
HardwareSerial Sender(1);
const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Arm Pins
const int pwm_pin[] = {pwm_swivel, pwm_link1, pwm_link2, Rpwm, Lpwm};
const int dir_pin[] = {dir_swivel, dir_link1, dir_link2, Rdir, Ldir};
const int limit_pin[] = {L1, L2, L3, L4, Stow};

// Globals
long prevT = 0;
int changeMode = 0;

// Arm Variables
int set_link[3];
int pwr[3];
int dir[3];

// Class instances
FK fk;
Drive drive;


void arm(int swivel, int l1, int l2, int Reset)
{
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  //Get Arm controls
  fk.arm_control(swivel, l1, l2, deltaT, Reset);
}

void gripper(int grip)
{
  switch (grip)
  {
    case 0: // safety
      Sender.write('0');
//      Serial.println("0");
      break;

    case 1: // safety
      Sender.write('1');
//      Serial.println("1");
      break;

    case 2: // safety
      Sender.write('2');
//      Serial.println("2");
      break;

    case 3: // safety
      Sender.write('3');
//      Serial.println("3");
      break;

    case 4: // safety
      Sender.write('4');
//      Serial.println("4");
      break;

    case 5: // safety
      Sender.write('5');
//      Serial.println("5");
      break;

    case 6: // safety
      Sender.write('6');
//      Serial.println("6");
      break;

     case 7: // safety
      Sender.write('7');
//      Serial.println("7");
      break;

    default:
      Sender.write('0');
//      Serial.println("7");
      break;
  }
}



void setup()
{
  Serial.begin(115200);
  SerialPort.begin(115200) ;   // Use default serial for debug output
  Sender.begin(115200, SERIAL_8N1, 0, 2);//(baud rate,protocol,Tx,Rx)

  //  // Set Arm Output Pins
  //  for (int i = 0; i < 3; i++)
  //  {
  //    pinMode(pwm_pin[i], OUTPUT);
  //    pinMode(dir_pin[i], OUTPUT);
  //  }

  // Limit Switch INPUT Pins
  for (int i = 0; i < 5; i++)
  {
    pinMode(limit_pin[i], INPUT);
  }

  // Set Drive Output Pins
  drive.setDrivePins(dir_pin[3], dir_pin[4], pwm_pin[3], pwm_pin[4]);
  fk.setArmPins(dir_pin[0], pwm_pin[0], dir_pin[1], pwm_pin[1], dir_pin[2], pwm_pin[2]);
  Serial.println("Sare pins set hogaye!");
  delay(100);

  fk.set_pid();
  Serial.println("PID set");
  delay(10);

  fk.set_IMU();
  Serial.println("IMUs connected!!!");

  fk.set_init_angle();

}

void loop()
{
  if (SerialPort.available())
  {
    // read the data into the buffer
    while (SerialPort.available())
    {
      rxBuffer[bufferIndex] = (char)SerialPort.read();
      bufferIndex++;
      // Make sure we don't overflow the buffer
      if (bufferIndex >= BUFFER_SIZE)
        bufferIndex = 0;
    }
    //    Serial.println(rxBuffer);

    // Find the positions of the "M", "X", "Y", "P", "Q", "A", "S", "R", "D" and "E" characters in the buffer
    char *M_index = strchr(rxBuffer, 'M');
    char *x_index = strchr(rxBuffer, 'X');
    char *y_index = strchr(rxBuffer, 'Y');
    char *P_index = strchr(rxBuffer, 'P');
    char *Q_index = strchr(rxBuffer, 'Q');
    char *A_index = strchr(rxBuffer, 'A');
    char *S_index = strchr(rxBuffer, 'S');
    char *R_index = strchr(rxBuffer, 'R');
    char *D_index = strchr(rxBuffer, 'D');
    char *E_index = strchr(rxBuffer, 'E');

    if (M_index != NULL && x_index != NULL && y_index != NULL && P_index != NULL && Q_index != NULL && A_index != NULL && S_index != NULL && R_index != NULL && D_index != NULL && E_index != NULL)
    {
      // Extract the values from the packet
      char m = *(M_index + 1);
      int M = m - '0';
      int x = atoi(x_index + 1);
      int y = atoi(y_index + 1);
      int l2 = atoi(P_index + 1);
      if (abs(l2) < 3)
      {
        l2 = 0;
      }
      int l1 = atoi(Q_index + 1);
      if (abs(l1) < 3)
      {
        l1 = 0;
      }
      int grip = atoi(A_index + 1);
      int swivel = atoi(S_index + 1);
      if (abs(swivel) < 3)
      {
        swivel = 0;
      }
      int Reset = atoi(R_index + 1);
      int Mode = atoi(D_index + 1);
      drive.MotorCode(x, y, M);
      if (Mode == 1)
      {
        if (changeMode == 0)
        {
          fk.Stop();
          changeMode = 1;
        }
        drive.MotorCode(x, y, M);
      }
      else
      {
        if (changeMode == 1)
        {
          fk.drive_rm();
          changeMode = 0;
        }
        drive.MotorCode(x, y, M);
        if (Reset == 1)
        {
          swivel = 0;
        }
        arm(swivel, l1, l2, Reset);
        gripper(grip);
      }
      delay(10);

    }
    else
    {
      Serial.println("Invalid Packet received");
    }
  }
  else
  {
    fk.Stop();
    drive.Stop();

  }
  bufferIndex = 0;
}
