#include "pid.h"
#include "imu.h"

class FK
{
  private:
    //---DEFINE TARGET JOINT ANGLES---
    float swivel = 0;
    float pitch1 = 0;
    float pitch2 = 0;
    int dir_link1, pwm_link1, dir_link2, pwm_link2, dir_swivel, pwm_swivel;
    int pwmswl, dirswl, pwm1, dir1, pwm2, dir2;
    int freq, L1channel, L2channel, Swchannel, resolution;
    float set_link[3];

    // PID class instances
    PID arm_pid[3]; //swl,L1,L2

  public:
    FK()
    {
      freq = 8000;
      L1channel = 2;
      L2channel = 3;
      Swchannel = 4;
      resolution = 8;
    }

    void setArmPins( int dir_sw, int pwm_sw, int dir_1, int pwm_1, int dir_2, int pwm_2)
    {
      dir_link1 = dir_1;
      pwm_link1 = pwm_1;
      dir_link2 = dir_2;
      pwm_link2 = pwm_2;
      dir_swivel = dir_sw;
      pwm_swivel = pwm_sw;

      ledcSetup(L1channel, freq, resolution);
      ledcSetup(L2channel, freq, resolution);
      ledcSetup(Swchannel, freq, resolution);
      ledcAttachPin(pwm_link1, L1channel);
      ledcAttachPin(pwm_link2, L2channel);
      ledcAttachPin(pwm_swivel, Swchannel);
      pinMode(dir_link1, OUTPUT);
      pinMode(dir_link2, OUTPUT);
      pinMode(dir_swivel, OUTPUT);
    }
    void set_pid(void)
    {
      //                   p,    d,     i,     max,min,hold
      arm_pid[0].setParams(20, 0.0000, 0.00000, 255, 25, 1); //swl
      arm_pid[1].setParams(20, 0.0000, 0.00000, 255, 25, 1); //L1
      arm_pid[2].setParams(20, 0.0000, 0.00000, 255, 25, 1); //L2
    }
    void set_IMU(void)
    {
      //---BNO055-1---
      setBNO0551();
      delay(500);

      //---BNO055-2---
      setBNO0552();
      delay(500);
    }
    void set_init_angle(void)
    {
      set_link[0] = zBNO0552();
      set_link[1] = yBNO0551();
      set_link[2] = yBNO0552();

      //      Serial.print("set_swivel: ");
      //      Serial.print(set_link[0]);
      //      Serial.print("set_L1: ");
      //      Serial.print(set_link[1]);
      //      Serial.print("set_L2: ");
      //      Serial.println(set_link[2]);
    }
    void Stop(void)
    {
      ledcWrite(Swchannel, 0);
      ledcWrite(L1channel, 0);
      ledcWrite(L2channel, 0);
    }
    void drive_rm(void)
    {
      setBNO0551();
      delay(50);
      setBNO0552();
      delay(50);

      set_link[0] = 0;
      set_link[1] = yBNO0551();
      set_link[2] = yBNO0552();
    }

    void arm_control(int swivel, int l1, int l2, float deltaT, int Reset)
    {

      set_link[0] += swivel * 0.1;
      set_link[1] += l1 * 0.1;
      set_link[2] += l2 * 0.1;
      if (swivel == 0)
      {
        set_link[0] = zBNO0552();
      }
      if (l1 == 0)// && (set_link[1] > 8) && (set_link[1] < 170))
      {
        set_link[1] = yBNO0551();
      }
      if (l2 == 0)// && ((pitch1 - pitch2) < 125))
      {
        set_link[2] = yBNO0552();
      }

      if (Reset == 1)
      {
        setBNO0551();
        delay(10);
        setBNO0552();
        delay(50);

        set_link[0] = 0;
        set_link[1] = yBNO0551();
        set_link[2] = yBNO0552();
      }

      //---BNO055-2-Sw---
      float pitchSw = zBNO0552();
      delay(10);

      //---BNO055-1-L1---
      float pitch1 = yBNO0551();
      delay(10);

      //---BNO055-2-L2--
      float pitch2 = yBNO0552();
      delay(10);

//      if (set_link[0] > 90)
//      {
//        set_link[0] = 90;
//      }
//      if (set_link[0] < -90)
//      {
//        set_link[0] = -90;
//      }
      if ((pitch1 - pitch2) > 125)
      {
        set_link[2] += 2 ;
      }
      if (set_link[1] > 170)
      {
        set_link[1] = 170;
      }
      if (set_link[1] < 8)
      {
        set_link[1] = 8;
      }
      //      if (set_link[2] > 90)
      //      {
      //        set_link[2] = 90;
      //      }
      //      if (set_link[2] > 90)
      //      {
      //        set_link[2] = 90;
      //      }

      Serial.print("pitSW: ");
      Serial.print(pitchSw);
      Serial.print(" | pit1: ");
      Serial.print(pitch1);
      Serial.print(" | pit2: ");
      Serial.print(pitch2);
      Serial.print("  ||  setSW: ");
      Serial.print(set_link[0]);
      Serial.print(" | set1: ");
      Serial.print(set_link[1]);
      Serial.print(" | set2: ");
      Serial.println(set_link[2]);

      arm_pid[0].getpid(pitchSw, set_link[0], deltaT, pwmswl, dirswl);
      arm_pid[1].getpid(pitch1, set_link[1], deltaT, pwm1, dir1);
      arm_pid[2].getpid(pitch2, set_link[2], deltaT, pwm2, dir2);

      //Set arm controls
      digitalWrite(dir_swivel, dirswl);
      ledcWrite(Swchannel, pwmswl);
      digitalWrite(dir_link1, dir1);
      ledcWrite(L1channel, pwm1);
      digitalWrite(dir_link2, dir2);
      ledcWrite(L2channel, pwm2);

      //      Serial.print("Swivel:-  Dir:");
      //      Serial.print(dirswl);
      //      Serial.print(",Pwm:");
      //      Serial.print(pwmswl);
      //      Serial.print(" | Link 1:-  Dir:");
      //      Serial.print(dir1);
      //      Serial.print(",Pwm:");
      //      Serial.print(pwm1);
      //      Serial.print(" | Link 2:-  Dir:");
      //      Serial.print(dir2);
      //      Serial.print(",Pwm:");
      //      Serial.println(pwm2);
    }
};
