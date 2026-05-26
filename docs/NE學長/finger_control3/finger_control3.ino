#include "Finger_servo.h"
// #include "TimerOne.h"

FINGER finger(1, 4095);
byte buf[20];
unsigned long time;
int vibration_level = 0; 
bool stop[10] = {false};  // 每个马达的停止状态

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000);
  // Timer1.initialize(100000);   
}

void loop() 
{
  if (Serial.available())
  {
    int n = Serial.parseInt();
    // Serial.println(n);
    switch (n)
    {
      // case 0:
      // {
      //   int count = 0;
      //   while (count == 0)
      //   {
      //     Serial.println("輸入角度");
      //     if (Serial.available())
      //     {
      //       float degree;
      //       degree = Serial.parseFloat();
      //       finger.angle_control(10, degree, 1500);
      //       // finger.angle_control(3, degree, 1500);
      //       count++;
      //     }
      //     delay(1000);
      //   }
      // }
      // break;

      case 1:
      {
        int count = 0;
        while (count == 0)
        {
          Serial.println("輸入角度");
          if (Serial.available())
          {
            float degree;
            degree = Serial.parseFloat();
            finger.angle_control(5, degree, 1500);
            // finger.angle_control(3, degree, 1500);
            count++;
          }
          delay(1000);
        }
      }
      break;

      case 2:
      {
        int count = 0;
        while (count == 0)
        {
          Serial.println("輸入角度");
          if (Serial.available())
          {
            float degree;
            degree = Serial.parseFloat();
            finger.angle_control_acc(1, degree, 2000, 50);
            count++;
          }
          delay(1000);
        }
      }
      break;

      case 3:
      {
        int count = 0;
        while (count == 0)
        {
          finger.ReadPOS(1);
          finger.ReadPOS(2);
          finger.ReadPOS(3);
          finger.ReadPOS(4);
          finger.ReadPOS(5);
          finger.ReadPOS(6);
          finger.ReadPOS(7);
          finger.ReadPOS(8);
          finger.ReadPOS(9);
          finger.ReadPOS(10);
          count++;
          delay(1000);
        }
        delay(500);
      }
      break;

      case 4:
      {
        int count = 0;
        while (count == 0)
        {
          finger.ReadTorque(9);
          count++;
          delay(1000);
        }
        delay(500);
      }
      break;

      case 5: // 双手 10只手指 双手托盘
      {
        const int maxTorque = 250;
        const int maxTorque2 = 800;

        int counts[10] = {0};  // 每个马达的旋转次数
        float lastPos[10] = {0};  // 每个马达的最后位置
        int state[10] = {0};  // 0: 运行, 1: 达到扭矩限制 


        // finger.angle_control(1, 2600, 2500); // 右手小拇指
        // delay(1);
        // finger.angle_control(2, 2600, 2500); // 右手无名指
        // delay(1);
        // finger.angle_control(3, 2600, 2500); // 右手中指
        // delay(1);
        // finger.angle_control(4, 2600, 2500); // 右手食指
        // delay(1);
        finger.angle_control(5, 2600, 2400); // 右手大拇指 
        delay(1);
        // finger.angle_control(6, 2600, 2500);  //左手小拇指 
        // delay(1);
        // finger.angle_control(7, 2600, 2500);  //左手无名指
        // delay(1);
        // finger.angle_control(8, 2600, 2500);  //左手中指
        // delay(1);
        // finger.angle_control(9, 2600, 2500);  //左手食指
        // delay(1);
        finger.angle_control(10, 2600, 2400); //左手大拇指

        while (true)
        {
          int check = 8;

          for (int i = 0; i < 10; i++)
          {
            Serial.print(state[i]);
            Serial.print("  ");
          }
          Serial.println();

          for (int i = 0; i < 10; i++)
          {
            Serial.print(counts[i]);
            Serial.print("  ");
          }
          Serial.println();

          for(int i = 0; i < 10; i++)
          {
            if (state[i] == 1)
            {
              check += 1;
            }
          }
          if (check >= 10) break;
          Serial.println(check);

          finger.ReadTorque(1);
          finger.ReadPOS(1);
          int Tor1 = finger.getTorque(1);
          float Pos1 = finger.getAngle(1);

          finger.ReadTorque(2);
          finger.ReadPOS(2);
          int Tor2 = finger.getTorque(2);
          float Pos2 = finger.getAngle(2);

          finger.ReadTorque(3);
          finger.ReadPOS(3);
          int Tor3 = finger.getTorque(3);
          float Pos3 = finger.getAngle(3);

          finger.ReadTorque(4);
          finger.ReadPOS(4);
          int Tor4 = finger.getTorque(4);
          float Pos4 = finger.getAngle(4);

          finger.ReadTorque(5);
          finger.ReadPOS(5);
          int Tor5 = finger.getTorque(5);
          float Pos5 = finger.getAngle(5);

          finger.ReadTorque(6);
          finger.ReadPOS(6);
          int Tor6 = finger.getTorque(6);
          float Pos6 = finger.getAngle(6);

          finger.ReadTorque(7);
          finger.ReadPOS(7);
          int Tor7 = finger.getTorque(7);
          float Pos7 = finger.getAngle(7);

          finger.ReadTorque(8);
          finger.ReadPOS(8);
          int Tor8 = finger.getTorque(8);
          float Pos8 = finger.getAngle(8);

          finger.ReadTorque(9);
          finger.ReadPOS(9);
          int Tor9 = finger.getTorque(9);
          float Pos9 = finger.getAngle(9);

          finger.ReadTorque(10);
          finger.ReadPOS(10);
          int Tor10 = finger.getTorque(10);
          float Pos10 = finger.getAngle(10);


          if (state[0] == 0) 
          {
            if (Pos1 < 60 && lastPos[0] > 300)
            {
              counts[0]++;
            }
            lastPos[0] = Pos1;

            if (abs(Tor1) > maxTorque)
            {
              state[0] = 1;
              finger.angle_control(1, Pos1 + (counts[0] * 360 + 13) , 10);  
            }
            delay(10);
          }


          if (state[1] == 0) 
          {
            if (Pos2 < 60 && lastPos[1] > 300)
            {
              counts[1]++;
            }
            lastPos[1] = Pos2;

            if (abs(Tor2) > maxTorque)
            {
              state[1] = 1;
              finger.angle_control(2, Pos2 + (counts[1] * 360 + 13) , 10);
            }
            delay(10);
          }

  
          if (state[2] == 0) 
          {
            if (Pos3 < 60 && lastPos[2] > 300)
            {
              counts[2]++;
            }
            lastPos[2] = Pos3;

            if (abs(Tor3) > maxTorque)
            {
              state[2] = 1;
              finger.angle_control(3, Pos3 + (counts[2] * 360 + 13) , 10);
            }
            delay(10);
          }


          if (state[3] == 0) 
          {
            if (Pos4 < 60 && lastPos[3] > 300)
            {
              counts[3]++;
            }
            lastPos[3] = Pos4;

            if (abs(Tor4) > maxTorque)
            {
              state[3] = 1;
              finger.angle_control(4, Pos4 + (counts[3] * 360 + 13) , 10);
            }
            delay(10);
          }

      
          if (state[4] == 0) 
          {
            if (Pos5 < 60 && lastPos[4] > 300)
            {
              counts[4]++;
            }
            lastPos[4] = Pos5;

            if (abs(Tor5) > maxTorque2)
            {
              state[4] = 1;
              finger.angle_control(5, Pos5 + (counts[4] * 360 + 13) , 10);
            }
            delay(10);
          }

  

          if (state[5] == 0) 
          {
            if (Pos6 < 60 && lastPos[5] > 300)
            {
              counts[5]++;
            }
            lastPos[5] = Pos6;

            if (abs(Tor6) > maxTorque)
            {
              state[5] = 1;
              finger.angle_control(6, Pos6 + (counts[5] * 360 + 13) , 10);  
            }
            delay(10);
          }

          

          if (state[6] == 0) 
          {
            if (Pos7 < 60 && lastPos[6] > 300)
            {
              counts[6]++;
            }
            lastPos[6] = Pos7;

            if (abs(Tor7) > maxTorque)
            {
              state[6] = 1;
              finger.angle_control(7, Pos7 + (counts[6] * 360 + 13) , 10);
            }
            delay(10);
          }

          
          if (state[7] == 0) 
          {
            if (Pos8 < 60 && lastPos[7] > 300)
            {
              counts[7]++;
            }
            lastPos[7] = Pos8;

            if (abs(Tor8) > maxTorque)
            {
              state[7] = 1;
              finger.angle_control(8, Pos8 + (counts[7] * 360 + 13) , 10);
            }
            delay(10);
          }

          

          if (state[8] == 0) 
          {
            if (Pos9 < 60 && lastPos[8] > 300)
            {
              counts[8]++;
            }
            lastPos[8] = Pos9;

            if (abs(Tor9) > maxTorque)
            {
              state[8] = 1;
              finger.angle_control(9, Pos9 + (counts[8] * 360 + 13) , 10);
            }
            delay(10);
          }

          

          if (state[9] == 0) 
          {
            if (Pos10 < 60 && lastPos[9] > 300)
            {
              counts[9]++;
            }
            lastPos[9] = Pos10;

            if (abs(Tor10) > maxTorque2)
            {
              state[9] = 1;
              finger.angle_control(10, Pos10 + (counts[9] * 360 + 13) , 10);
            }
            delay(10);
          }
        }

      }
      break;

      case 6: //扭矩控制 3隻手指 右手3指抓球 
      {
        const int maxTorque = 600;
        const int maxTorque2 = 750;
        int counts[5] = {0};  // 每个马达的旋转次数
        float lastPos[5] = {0};  // 每个马达的最后位置
        int state[5] = {0};  // 0: 运行，1: 达到扭矩限制 

        // finger.angle_control(1, 2600, 2000);
        // delay(1);
        // finger.angle_control(2, 2600, 2000);
        // delay(1);
        finger.angle_control(3, 2600, 2500);
        delay(1);
        finger.angle_control(4, 2600, 2500);
        delay(1);
        finger.angle_control(5, 2600, 2000);
        delay(1);

        while (true)
        {
          // bool allMotorsDone = true;
          // for (int i = 0; i < 5; i++)
          // {
          //   if (state[i] != 1)
          //   {
          //     allMotorsDone = false;
          //     break;
          //   }
          // }
          // if (allMotorsDone)
          // {
          //   break;
          // }
          int check = 2;

          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(state[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(counts[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          for(int i = 0; i < 5; i++)
          {
            if (state[i] == 1)
            {
              check += 1;
            }
          }
          if (check >= 5) break;
          // Serial.println(check);

          finger.ReadTorque(1);
          finger.ReadPOS(1);
          int Tor1 = finger.getTorque(1);
          float Pos1 = finger.getAngle(1); 

          finger.ReadTorque(2);
          finger.ReadPOS(2);
          int Tor2 = finger.getTorque(2);
          float Pos2 = finger.getAngle(2); 

          finger.ReadTorque(3);
          finger.ReadPOS(3);
          int Tor3 = finger.getTorque(3);
          float Pos3 = finger.getAngle(3); 

          finger.ReadTorque(4);
          finger.ReadPOS(4);
          int Tor4 = finger.getTorque(4);
          float Pos4 = finger.getAngle(4); 

          finger.ReadTorque(5);
          finger.ReadPOS(5);
          int Tor5 = finger.getTorque(5);
          float Pos5 = finger.getAngle(5);

          
          // for motor 1
          if (state[0] == 0) 
          {
            // finger.ReadTorque(6);
            // finger.ReadPOS(6);
            // int Tor6 = finger.getTorque(6);
            // float Pos6 = finger.getAngle(6); 

            //-------------------------------------------
            // Serial.print("ID: 1 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor1);
            // Serial.print("Position: ");
            // Serial.println(Pos1);

            if (Pos1 < 60 && lastPos[0] > 300)
            {
              counts[0]++;
            }
            lastPos[0] = Pos1;
            // delay(10);

            if (abs(Tor1) > maxTorque)
            {
              state[0] = 1;
              // Serial.print("ID: 1 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(1, Pos1 + (counts[0] * 360 + 13) , 10);  
              // continue;
            }
            // Serial.print(Pos6);
            // Serial.print("  ");
            // Serial.println(lastPos[0]);

            // if (Pos1 < 60 && lastPos[0] > 300)
            // {
            //   counts[0]++;
            // }
            // lastPos[0] = Pos1;
            delay(10);
          }


          // for motor 2
          if (state[1] == 0) 
          {
            // finger.ReadTorque(7);
            // finger.ReadPOS(7);
            // int Tor7 = finger.getTorque(7);
            // float Pos7 = finger.getAngle(7); 
            
            //----------------------------------
            // Serial.print("ID: 2 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor2);
            // Serial.print("Position: ");
            // Serial.println(Pos2);

            if (Pos2 < 60 && lastPos[1] > 300)
            {
              counts[1]++;
            }
            lastPos[1] = Pos2;
            

            if (abs(Tor2) > maxTorque)
            {
              state[1] = 1;
              // Serial.print("ID: 2 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(2, Pos2 + (counts[1] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos2 < 60 && lastPos[1] > 300)
            // {
            //   counts[1]++;
            // }
            // lastPos[1] = Pos2;
            delay(10);
          }

          // for motor 3
          if (state[2] == 0) 
          {
            // finger.ReadTorque(8);
            // finger.ReadPOS(8);
            // int Tor8 = finger.getTorque(8);
            // float Pos8 = finger.getAngle(8); 

            //---------------------------------
            // Serial.print("ID: 3 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor3);
            // Serial.print("Position: ");
            // Serial.println(Pos3);

            if (Pos3 < 60 && lastPos[2] > 300)
            {
              counts[2]++;
            }
            lastPos[2] = Pos3;

            if (abs(Tor3) > maxTorque)
            {
              state[2] = 1;
              // Serial.print("ID: 3 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(3, Pos3 + (counts[2] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos3 < 60 && lastPos[2] > 300)
            // {
            //   counts[2]++;
            // }
            // lastPos[2] = Pos3;
            delay(10);
          }
         
          // for motor 4
          if (state[3] == 0) 
          {
            // finger.ReadTorque(9);
            // finger.ReadPOS(9);
            // int Tor9 = finger.getTorque(9);
            // float Pos9 = finger.getAngle(9); 

            //---------------------------------
            // Serial.print("ID: 4 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor4);
            // Serial.print("Position: ");
            // Serial.println(Pos4);

            if (Pos4 < 60 && lastPos[3] > 300)
            {
              counts[3]++;
            }
            lastPos[3] = Pos4;
            // delay(10);

            if (abs(Tor4) > maxTorque)
            {
              state[3] = 1;
              // Serial.print("ID: 4 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(4, Pos4 + (counts[3] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos4 < 60 && lastPos[3] > 300)
            // {
            //   counts[3]++;
            // }
            // lastPos[3] = Pos4;
            delay(10);
          }
          // else if (state[3] == 1)
          // {
          //   finger.ReadPOS(9);
          //   float position = finger.getAngle(9);

          //   finger.angle_control(9, position + (counts[3] * 360) + 30, 30);
          //   delay(3000);
          //   finger.angle_control(9, 5, 3000);
          //   state[3] = 2;
          // }


          // for motor 5
          if (state[4] == 0) 
          {
            // finger.ReadTorque(10);
            // finger.ReadPOS(10);
            // int Tor10 = finger.getTorque(10);
            // float Pos10 = finger.getAngle(10); 

            // ---------------------------------------
            // Serial.print("ID: 5 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor5);
            // Serial.print("Position: ");
            // Serial.println(Pos5);


            if (Pos5 < 60 && lastPos[4] > 300)
            {
              counts[4]++;
            }
            lastPos[4] = Pos5;
            // delay(10);

            if (abs(Tor5) > maxTorque2)
            {
              state[4] = 1;
              // Serial.print("ID: 5 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(5, Pos5 + (counts[4] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos5 < 60 && lastPos[4] > 300)
            // {
            //   counts[4]++;
            // }
            // lastPos[4] = Pos5;
            delay(10);
          }
          // else if (state[4] == 1)
          // {
          //   finger.ReadPOS(10);
          //   float position = finger.getAngle(10);
          //   finger.angle_control(10, position + (counts[4] * 360) + 30, 30);
          //   delay(1000);
          //   finger.angle_control(10, 5, 3000);
          //   state[4] = 2;
          // }
        }
        //write more tighter 
      }
      break;

      case 7:  // 右手扭矩控制 五隻手指 大拇指的扭矩偏大
      {
        const int maxTorque = 650;
        const int maxTorque2 = 800;
        int counts[5] = {0};  // 每个马达的旋转次数
        float lastPos[5] = {0};  // 每个马达的最后位置
        int state[5] = {0};  // 0: 运行，1: 达到扭矩限制 

        finger.angle_control(1, 2600, 2400);
        delay(1);
        finger.angle_control(2, 2600, 2400);
        delay(1);
        finger.angle_control(3, 2600, 2400);
        delay(1);
        finger.angle_control(4, 2600, 2400);
        delay(1);
        finger.angle_control(5, 2600, 2000);

        while (true)
        {
          // bool allMotorsDone = true;
          // for (int i = 0; i < 5; i++)
          // {
          //   if (state[i] != 1)
          //   {
          //     allMotorsDone = false;
          //     break;
          //   }
          // }
          // if (allMotorsDone)
          // {
          //   break;
          // }
          int check = 1;

          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(state[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(counts[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          for(int i = 0; i < 5; i++)
          {
            if (state[i] == 1)
            {
              check += 1;
            }
          }
          if (check >= 5) break;
          // Serial.println(check);

          finger.ReadTorque(1);
          finger.ReadPOS(1);
          int Tor1 = finger.getTorque(1);
          float Pos1 = finger.getAngle(1); 

          finger.ReadTorque(2);
          finger.ReadPOS(2);
          int Tor2 = finger.getTorque(2);
          float Pos2 = finger.getAngle(2); 

          finger.ReadTorque(3);
          finger.ReadPOS(3);
          int Tor3 = finger.getTorque(3);
          float Pos3 = finger.getAngle(3); 

          finger.ReadTorque(4);
          finger.ReadPOS(4);
          int Tor4 = finger.getTorque(4);
          float Pos4 = finger.getAngle(4); 

          finger.ReadTorque(5);
          finger.ReadPOS(5);
          int Tor5 = finger.getTorque(5);
          float Pos5 = finger.getAngle(5);

          
          // for motor 1
          if (state[0] == 0) 
          {
            // finger.ReadTorque(6);
            // finger.ReadPOS(6);
            // int Tor6 = finger.getTorque(6);
            // float Pos6 = finger.getAngle(6); 

            //-------------------------------------------
            // Serial.print("ID: 1 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor1);
            // Serial.print("Position: ");
            // Serial.println(Pos1);

            if (Pos1 < 60 && lastPos[0] > 300)
            {
              counts[0]++;
            }
            lastPos[0] = Pos1;
            // delay(10);

            if (abs(Tor1) > maxTorque)
            {
              state[0] = 1;
              // Serial.print("ID: 1 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(1, Pos1 + (counts[0] * 360 + 13) , 10);  
              // continue;
            }
            // Serial.print(Pos6);
            // Serial.print("  ");
            // Serial.println(lastPos[0]);

            // if (Pos1 < 60 && lastPos[0] > 300)
            // {
            //   counts[0]++;
            // }
            // lastPos[0] = Pos1;
            delay(10);
          }


          // for motor 2
          if (state[1] == 0) 
          {
            // finger.ReadTorque(7);
            // finger.ReadPOS(7);
            // int Tor7 = finger.getTorque(7);
            // float Pos7 = finger.getAngle(7); 
            
            //----------------------------------
            // Serial.print("ID: 2 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor2);
            // Serial.print("Position: ");
            // Serial.println(Pos2);

            if (Pos2 < 60 && lastPos[1] > 300)
            {
              counts[1]++;
            }
            lastPos[1] = Pos2;
            

            if (abs(Tor2) > maxTorque)
            {
              state[1] = 1;
              // Serial.print("ID: 2 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(2, Pos2 + (counts[1] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos2 < 60 && lastPos[1] > 300)
            // {
            //   counts[1]++;
            // }
            // lastPos[1] = Pos2;
            delay(10);
          }

          // for motor 3
          if (state[2] == 0) 
          {
            // finger.ReadTorque(8);
            // finger.ReadPOS(8);
            // int Tor8 = finger.getTorque(8);
            // float Pos8 = finger.getAngle(8); 

            //---------------------------------
            // Serial.print("ID: 3 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor3);
            // Serial.print("Position: ");
            // Serial.println(Pos3);

            if (Pos3 < 60 && lastPos[2] > 300)
            {
              counts[2]++;
            }
            lastPos[2] = Pos3;

            if (abs(Tor3) > maxTorque)
            {
              state[2] = 1;
              // Serial.print("ID: 3 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(3, Pos3 + (counts[2] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos3 < 60 && lastPos[2] > 300)
            // {
            //   counts[2]++;
            // }
            // lastPos[2] = Pos3;
            delay(10);
          }
         
          // for motor 4
          if (state[3] == 0) 
          {
            // finger.ReadTorque(9);
            // finger.ReadPOS(9);
            // int Tor9 = finger.getTorque(9);
            // float Pos9 = finger.getAngle(9); 

            //---------------------------------
            // Serial.print("ID: 4 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor4);
            // Serial.print("Position: ");
            // Serial.println(Pos4);

            if (Pos4 < 60 && lastPos[3] > 300)
            {
              counts[3]++;
            }
            lastPos[3] = Pos4;
            // delay(10);

            if (abs(Tor4) > maxTorque)
            {
              state[3] = 1;
              // Serial.print("ID: 4 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(4, Pos4 + (counts[3] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos4 < 60 && lastPos[3] > 300)
            // {
            //   counts[3]++;
            // }
            // lastPos[3] = Pos4;
            delay(10);
          }
          // else if (state[3] == 1)
          // {
          //   finger.ReadPOS(9);
          //   float position = finger.getAngle(9);

          //   finger.angle_control(9, position + (counts[3] * 360) + 30, 30);
          //   delay(3000);
          //   finger.angle_control(9, 5, 3000);
          //   state[3] = 2;
          // }


          // for motor 5
          if (state[4] == 0) 
          {
            // finger.ReadTorque(10);
            // finger.ReadPOS(10);
            // int Tor10 = finger.getTorque(10);
            // float Pos10 = finger.getAngle(10); 

            // ---------------------------------------
            // Serial.print("ID: 5 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor5);
            // Serial.print("Position: ");
            // Serial.println(Pos5);


            if (Pos5 < 60 && lastPos[4] > 300)
            {
              counts[4]++;
            }
            lastPos[4] = Pos5;
            // delay(10);

            if (abs(Tor5) > maxTorque2)
            {
              state[4] = 1;
              // Serial.print("ID: 5 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(5, Pos5 + (counts[4] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos5 < 60 && lastPos[4] > 300)
            // {
            //   counts[4]++;
            // }
            // lastPos[4] = Pos5;
            delay(10);
          }
          // else if (state[4] == 1)
          // {
          //   finger.ReadPOS(10);
          //   float position = finger.getAngle(10);
          //   finger.angle_control(10, position + (counts[4] * 360) + 30, 30);
          //   delay(1000);
          //   finger.angle_control(10, 5, 3000);
          //   state[4] = 2;
          // }
        }
        //write more tighter 
      }
      break;


      case 8:  // 左手 五指控制 大拇指扭矩偏大
      {
        const int maxTorque = 650;
        const int maxTorque2 = 800;
        int counts[5] = {0};  // 每个马达的旋转次数
        float lastPos[5] = {0};  // 每个马达的最后位置
        int state[5] = {0};  // 0: 运行，1: 达到扭矩限制 

        finger.angle_control(6, 2600, 2400);  //小拇指 
        delay(1);
        finger.angle_control(7, 2600, 2400);  //无名指
        delay(1);
        finger.angle_control(8, 2600, 2400);  //中指
        delay(1);
        finger.angle_control(9, 2600, 2400);  // 食指
        delay(1);
        finger.angle_control(10, 2600, 2400); // 大拇指

        while (true)
        {
          // bool allMotorsDone = true;
          // for (int i = 0; i < 5; i++)
          // {
          //   if (state[i] != 1)
          //   {
          //     allMotorsDone = false;
          //     break;
          //   }
          // }
          // if (allMotorsDone)
          // {
          //   break;
          // }
          int check = 0;
          
          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(state[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(counts[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          for(int i = 0; i < 5; i++)
          {
            if (state[i] == 1)
            {
              check += 1;
            }
          }
          if (check >= 5) break;
          // Serial.println(check); 

          finger.ReadTorque(6);
          finger.ReadPOS(6);
          int Tor6 = finger.getTorque(6);
          float Pos6 = finger.getAngle(6); 

          finger.ReadTorque(7);
          finger.ReadPOS(7);
          int Tor7 = finger.getTorque(7);
          float Pos7 = finger.getAngle(7); 

          finger.ReadTorque(8);
          finger.ReadPOS(8);
          int Tor8 = finger.getTorque(8);
          float Pos8 = finger.getAngle(8); 

          finger.ReadTorque(9);
          finger.ReadPOS(9);
          int Tor9 = finger.getTorque(9);
          float Pos9 = finger.getAngle(9); 

          finger.ReadTorque(10);
          finger.ReadPOS(10);
          int Tor10 = finger.getTorque(10);
          float Pos10 = finger.getAngle(10);

          
          // for motor 6
          if (state[0] == 0) 
          {
            // finger.ReadTorque(6);
            // finger.ReadPOS(6);
            // int Tor6 = finger.getTorque(6);
            // float Pos6 = finger.getAngle(6); 
            //------------------------------------------
            // Serial.print("ID: 6 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor6);
            // Serial.print("Position: ");
            // Serial.println(Pos6);

            if (Pos6 < 60 && lastPos[0] > 300)
            {
              counts[0]++;
            }
            lastPos[0] = Pos6;

            if (abs(Tor6) > maxTorque)
            {
              state[0] = 1;
              // Serial.print("ID: 6 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(6, Pos6 + (counts[0] * 360 + 13) , 10);    // 20, 30
              // continue;
            }
            // Serial.print(Pos6);
            // Serial.print("  ");
            // Serial.println(lastPos[0]);

            // if (Pos6 < 60 && lastPos[0] > 300)
            // {
            //   counts[0]++;
            // }
            // lastPos[0] = Pos6;
            delay(10);
          }


          // for motor 7
          if (state[1] == 0) 
          {
            // finger.ReadTorque(7);
            // finger.ReadPOS(7);
            // int Tor7 = finger.getTorque(7);
            // float Pos7 = finger.getAngle(7); 
            //----------------------------------
            // Serial.print("ID: 7 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor7);
            // Serial.print("Position: ");
            // Serial.println(Pos7);

            if (Pos7 < 60 && lastPos[1] > 300)
            {
              counts[1]++;
            }
            lastPos[1] = Pos7;

            if (abs(Tor7) > maxTorque)
            {
              state[1] = 1;
              // Serial.print("ID: 7 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(7, Pos7 + (counts[1] * 360 + 13 ) , 10);
              // continue;
            }

            // if (Pos7 < 60 && lastPos[1] > 300)
            // {
            //   counts[1]++;
            // }
            // lastPos[1] = Pos7;
            delay(10);
          }

          // for motor 8
          if (state[2] == 0) 
          {
            // finger.ReadTorque(8);
            // finger.ReadPOS(8);
            // int Tor8 = finger.getTorque(8);
            // float Pos8 = finger.getAngle(8); 
            //-----------------------------------
            // Serial.print("ID: 8 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor8);
            // Serial.print("Position: ");
            // Serial.println(Pos8);

            if (Pos8 < 60 && lastPos[2] > 300)
            {
              counts[2]++;
            }
            lastPos[2] = Pos8;

            if (abs(Tor8) > maxTorque)
            {
              state[2] = 1;
              // Serial.print("ID: 8 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(8, Pos8 + (counts[2] * 360 + 13 ) , 10);
              // continue;
            }

            // if (Pos8 < 60 && lastPos[2] > 300)
            // {
            //   counts[2]++;
            // }
            // lastPos[2] = Pos8;
            delay(10);
          }
         
          // for motor 9
          if (state[3] == 0) 
          {
            // finger.ReadTorque(9);
            // finger.ReadPOS(9);
            // int Tor9 = finger.getTorque(9);
            // float Pos9 = finger.getAngle(9); 
            //---------------------------------
            // Serial.print("ID: 9 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor9);
            // Serial.print("Position: ");
            // Serial.println(Pos9);

            if (Pos9 < 60 && lastPos[3] > 300)
            {
              counts[3]++;
            }
            lastPos[3] = Pos9;

            if (abs(Tor9) > maxTorque)
            {
              state[3] = 1;
              // Serial.print("ID: 9 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(9, Pos9 + (counts[3] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos9 < 60 && lastPos[3] > 300)
            // {
            //   counts[3]++;
            // }
            // lastPos[3] = Pos9;
            delay(10);
          }
          // else if (state[3] == 1)
          // {
          //   finger.ReadPOS(9);
          //   float position = finger.getAngle(9);

          //   finger.angle_control(9, position + (counts[3] * 360) + 30, 30);
          //   delay(3000);
          //   finger.angle_control(9, 5, 3000);
          //   state[3] = 2;
          // }


          // for motor 10
          if (state[4] == 0) 
          {
            // finger.ReadTorque(10);
            // finger.ReadPOS(10);
            // int Tor10 = finger.getTorque(10);
            // float Pos10 = finger.getAngle(10); 
            //--------------------------------------
            // Serial.print("ID: 10 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor10);
            // Serial.print("Position: ");
            // Serial.println(Pos10);

            if (Pos10 < 60 && lastPos[4] > 300)
            {
              counts[4]++;
            }
            lastPos[4] = Pos10;

            if (abs(Tor10) > maxTorque2)
            {
              state[4] = 1;
              // Serial.print("ID: 10 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(10, Pos10 + (counts[4] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos10 < 60 && lastPos[4] > 300)
            // {
            //   counts[4]++;
            // }
            // lastPos[4] = Pos10;

            delay(10);
          }
          // else if (state[4] == 1)
          // {
          //   finger.ReadPOS(10);
          //   float position = finger.getAngle(10);
          //   finger.angle_control(10, position + (counts[4] * 360) + 30, 30);
          //   delay(1000);
          //   finger.angle_control(10, 5, 3000);
          //   state[4] = 2;
          // }
        }
        //write more tighter 
      }
      break;

      case 9:  // 左手空抓
      {
        finger.angle_control(6, 365, 2500);
        // delay(1);
        finger.angle_control(7, 590, 2500);
        // delay(1);
        finger.angle_control(8, 560, 2500);
        // delay(1);
        finger.angle_control(9, 510, 2500);
        // delay(1);
        // finger.angle_control(10, 320, 1500);
        // delay(1);
      }
      break;

      case 10:  // 左手回原点.    motor 10 : 230 fin: 780.   motor 9: 400 fin:850.   motor 8: 150 fin: 600. motor 7: 50 fin: 500. motor 6: 310 fin:760
      {         
        finger.angle_control(6, 5, 6000);   // 小拇指: 310改460     10改100  100改160      
        delay(1);
        finger.angle_control(7, 230, 6000);    // 无名指:  50改80.  80改130
        delay(1);
        finger.angle_control(8, 200, 6000);   // 中指: 150 改 220    220改 250
        delay(1);
        finger.angle_control(9, 150, 6000);    // 食指: 400 改 130  
        delay(1);
        finger.angle_control(10, 130, 5000);   // 大拇指: 230 改 60.  60改100 
        delay(1);
      }
      break;

      case 11: // 右手空抓
      {
        finger.angle_control(1, 440, 2400);
        delay(1);
        finger.angle_control(2, 365, 2400);
        delay(1);
        finger.angle_control(3, 410, 2400);
        delay(1);
        finger.angle_control(4, 560, 2400);
        delay(1);
        // finger.angle_control(5, 365, 1500);
        // delay(1);
      }
      break;

      case 12: //右手回原点 
      {
        finger.angle_control(1, 80, 6000);  // 30 to 400.
        delay(1);
        finger.angle_control(2, 5, 6000);  
        delay(1);
        finger.angle_control(3, 50, 6000);  
        delay(1);
        finger.angle_control(4, 200, 6000);  
        delay(1);
        finger.angle_control(5, 300, 5000); 
        delay(1);
      }
      break;

      case 13: // 双手回原点
      {
        finger.angle_control(1, 80, 2000);  // 右手小拇指
        delay(1);
        finger.angle_control(2, 5, 2000);  // 右手无名指
        delay(1);
        finger.angle_control(3, 50, 2000);  // 右手中指
        delay(1);
        finger.angle_control(4, 200, 2000);  // 右手食指
        delay(1);
        finger.angle_control(5, 300, 2000);  // 右手大拇指 
        delay(1);
        finger.angle_control(6, 5, 2000);   // 左手小拇指: 310改460     10改100  100改160     
        delay(1);
        finger.angle_control(7, 230, 2000);    // 左手无名指:  50改80.  80改130
        delay(1);
        finger.angle_control(8, 200, 2000);   // 左手中指: 150 改 220    220改 250
        delay(1);
        finger.angle_control(9, 150, 2000);    // 左手食指: 400 改 130  
        delay(1);
        finger.angle_control(10, 130, 2000);   // 左手大拇指: 230 改 60.  60改100 
      }
      break;

      case 14: // 左手 三只手指 文件夹 
      {
        const int maxTorque = 550;
        const int maxTorque2 = 750;
        int counts[5] = {0};  // 每个马达的旋转次数
        float lastPos[5] = {0};  // 每个马达的最后位置
        int state[5] = {0};  // 0: 运行，1: 达到扭矩限制 

        // finger.angle_control(6, 2600, 2400);  //小拇指 
        // delay(1);
        // finger.angle_control(7, 2600, 2400);  //无名指
        // delay(1);
        finger.angle_control(8, 2600, 2000);  //中指
        delay(1);
        finger.angle_control(9, 2600, 2000);  // 食指
        delay(1);
        finger.angle_control(10, 2600, 2400); // 大拇指
        delay(1);

        while (true)
        {
          // bool allMotorsDone = true;
          // for (int i = 0; i < 5; i++)
          // {
          //   if (state[i] != 1)
          //   {
          //     allMotorsDone = false;
          //     break;
          //   }
          // }
          // if (allMotorsDone)
          // {
          //   break;
          // }
          int check = 2;
          
          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(state[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          // for (int i = 0; i < 5; i++)
          // {
          //   Serial.print(counts[i]);
          //   Serial.print("  ");
          // }
          // Serial.println();

          for(int i = 0; i < 5; i++)
          {
            if (state[i] == 1)
            {
              check += 1;
            }
          }
          if (check >= 5) break;
          // Serial.println(check); 

          finger.ReadTorque(6);
          finger.ReadPOS(6);
          int Tor6 = finger.getTorque(6);
          float Pos6 = finger.getAngle(6); 

          finger.ReadTorque(7);
          finger.ReadPOS(7);
          int Tor7 = finger.getTorque(7);
          float Pos7 = finger.getAngle(7); 

          finger.ReadTorque(8);
          finger.ReadPOS(8);
          int Tor8 = finger.getTorque(8);
          float Pos8 = finger.getAngle(8); 

          finger.ReadTorque(9);
          finger.ReadPOS(9);
          int Tor9 = finger.getTorque(9);
          float Pos9 = finger.getAngle(9); 

          finger.ReadTorque(10);
          finger.ReadPOS(10);
          int Tor10 = finger.getTorque(10);
          float Pos10 = finger.getAngle(10);

          
          // for motor 6
          if (state[0] == 0) 
          {
            // finger.ReadTorque(6);
            // finger.ReadPOS(6);
            // int Tor6 = finger.getTorque(6);
            // float Pos6 = finger.getAngle(6); 
            //------------------------------------------
            // Serial.print("ID: 6 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor6);
            // Serial.print("Position: ");
            // Serial.println(Pos6);

            if (Pos6 < 60 && lastPos[0] > 300)
            {
              counts[0]++;
            }
            lastPos[0] = Pos6;

            if (abs(Tor6) > maxTorque)
            {
              state[0] = 1;
              // Serial.print("ID: 6 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(6, Pos6 + (counts[0] * 360 + 13) , 10);    // 20, 30
              // continue;
            }
            // Serial.print(Pos6);
            // Serial.print("  ");
            // Serial.println(lastPos[0]);

            // if (Pos6 < 60 && lastPos[0] > 300)
            // {
            //   counts[0]++;
            // }
            // lastPos[0] = Pos6;
            delay(10);
          }


          // for motor 7
          if (state[1] == 0) 
          {
            // finger.ReadTorque(7);
            // finger.ReadPOS(7);
            // int Tor7 = finger.getTorque(7);
            // float Pos7 = finger.getAngle(7); 
            //----------------------------------
            // Serial.print("ID: 7 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor7);
            // Serial.print("Position: ");
            // Serial.println(Pos7);

            if (Pos7 < 60 && lastPos[1] > 300)
            {
              counts[1]++;
            }
            lastPos[1] = Pos7;

            if (abs(Tor7) > maxTorque)
            {
              state[1] = 1;
              // Serial.print("ID: 7 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(7, Pos7 + (counts[1] * 360 + 13 ) , 10);
              // continue;
            }

            // if (Pos7 < 60 && lastPos[1] > 300)
            // {
            //   counts[1]++;
            // }
            // lastPos[1] = Pos7;
            delay(10);
          }

          // for motor 8
          if (state[2] == 0) 
          {
            // finger.ReadTorque(8);
            // finger.ReadPOS(8);
            // int Tor8 = finger.getTorque(8);
            // float Pos8 = finger.getAngle(8); 
            //-----------------------------------
            // Serial.print("ID: 8 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor8);
            // Serial.print("Position: ");
            // Serial.println(Pos8);

            if (Pos8 < 60 && lastPos[2] > 300)
            {
              counts[2]++;
            }
            lastPos[2] = Pos8;

            if (abs(Tor8) > maxTorque)
            {
              state[2] = 1;
              // Serial.print("ID: 8 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(8, Pos8 + (counts[2] * 360 + 13 ) , 10);
              // continue;
            }

            // if (Pos8 < 60 && lastPos[2] > 300)
            // {
            //   counts[2]++;
            // }
            // lastPos[2] = Pos8;
            delay(10);
          }
         
          // for motor 9
          if (state[3] == 0) 
          {
            // finger.ReadTorque(9);
            // finger.ReadPOS(9);
            // int Tor9 = finger.getTorque(9);
            // float Pos9 = finger.getAngle(9); 
            //---------------------------------
            // Serial.print("ID: 9 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor9);
            // Serial.print("Position: ");
            // Serial.println(Pos9);

            if (Pos9 < 60 && lastPos[3] > 300)
            {
              counts[3]++;
            }
            lastPos[3] = Pos9;

            if (abs(Tor9) > maxTorque)
            {
              state[3] = 1;
              // Serial.print("ID: 9 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(9, Pos9 + (counts[3] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos9 < 60 && lastPos[3] > 300)
            // {
            //   counts[3]++;
            // }
            // lastPos[3] = Pos9;
            delay(10);
          }
          // else if (state[3] == 1)
          // {
          //   finger.ReadPOS(9);
          //   float position = finger.getAngle(9);

          //   finger.angle_control(9, position + (counts[3] * 360) + 30, 30);
          //   delay(3000);
          //   finger.angle_control(9, 5, 3000);
          //   state[3] = 2;
          // }


          // for motor 10
          if (state[4] == 0) 
          {
            // finger.ReadTorque(10);
            // finger.ReadPOS(10);
            // int Tor10 = finger.getTorque(10);
            // float Pos10 = finger.getAngle(10); 
            //--------------------------------------
            // Serial.print("ID: 10 ");
            // Serial.print("Torque: ");
            // Serial.println(Tor10);
            // Serial.print("Position: ");
            // Serial.println(Pos10);

            if (Pos10 < 60 && lastPos[4] > 300)
            {
              counts[4]++;
            }
            lastPos[4] = Pos10;

            if (abs(Tor10) > maxTorque2)
            {
              state[4] = 1;
              // Serial.print("ID: 10 ");
              // Serial.println("Exceeded limit torque, stopping motor !!");
              finger.angle_control(10, Pos10 + (counts[4] * 360 + 13) , 10);
              // continue;
            }

            // if (Pos10 < 60 && lastPos[4] > 300)
            // {
            //   counts[4]++;
            // }
            // lastPos[4] = Pos10;

            delay(10);
          }
          // else if (state[4] == 1)
          // {
          //   finger.ReadPOS(10);
          //   float position = finger.getAngle(10);
          //   finger.angle_control(10, position + (counts[4] * 360) + 30, 30);
          //   delay(1000);
          //   finger.angle_control(10, 5, 3000);
          //   state[4] = 2;
          // }
        }
        //write more tighter 
      }
      break;


    }
  }
}
