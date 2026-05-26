#include "HardwareSerial.h"
#include "Print.h"
#include "Arduino.h"
#include "Finger_servo.h"

FINGER::FINGER()
{
    _serial = 0;
    _one_spin = 0;
}

FINGER::FINGER(int serial, int one_spin)
{
    _serial = serial;
    _one_spin = one_spin;
}

// FINGER::set_baudrate(int serial, int baudrate)
// {
//     _baudrate = baudrate;
//     if(serial == 1)
//     {
//         Serial1.begin(_baudrate);
//         Serial.println("1111");
//     }
//     else if(serial == 2)
//     {
//         Serial2.begin(_baudrate);
//     }
//     else if(serial == 3)
//     {
//         Serial3.begin(_baudrate);
//     }
// }

int FINGER::getTorque(int id) 
{
    if (id < 1 || id > 10) return 0;
    return torques[id-1];
}

float FINGER::getAngle(int id) 
{
    if (id < 1 || id > 10) return 0.0;
    return angles[id-1];
}

void FINGER::angle_control(int id, float angle, int speed)
{
    checksum = 0;
    int pos = round((angle / 360.0)*_one_spin);   //4095
    // if (pos < 0 )
    // {
    //   pos = 4095 + pos;
    // }
    Serial.print("pos : ");
    Serial.println(pos);
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 0x09;
    buf[4] = 0x03;
    buf[5] = 0x2A;
    buf[6] = (pos >> 0) & 0xFF;
    buf[7] = (pos >> 8) & 0xFF;
    buf[8] = 0x00;
    buf[9] = 0x00;
    buf[10] = (speed >> 0) & 0xFF;
    buf[11] = (speed >> 8) & 0xFF;
    for(int i = 2; i < 12; i++)
    {
        checksum += buf[i];
    }
    buf[12] = Cal_chechsum_v2(checksum);
    // for (int i = 0; i < 13; i++) 
    // {
    //     Serial.print(buf[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();
    Serial1.write(buf, 13);
    delay(10);

    if (Serial1.available())
  {
    while (Serial1.available()) 
    {
      byte c = Serial1.read();
      // Serial.print(c, HEX);
      // Serial.print(" ");
    }
    // Serial.println();
  }
  else 
  {
  Serial.println("No data received");
  }
}

void FINGER::angle_control_acc(int id, float angle, int speed, int acc)
{
    checksum = 0;
    int pos = round((angle / 360.0)*_one_spin);
    Serial.print("pos : ");
    Serial.println(pos);
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 0x0A;
    buf[4] = 0x03;
    buf[5] = 0x29;
    buf[6] = acc;
    buf[7] = (pos >> 0) & 0xFF;
    buf[8] = (pos >> 8) & 0xFF;
    buf[9] = 0x00;
    buf[10] = 0x00;
    buf[11] = (speed >> 0) & 0xFF;
    buf[12] = (speed >> 8) & 0xFF;
    for(int i = 2; i < 13; i++)
    {
        checksum += buf[i];
    }
    buf[13] = Cal_chechsum_v2(checksum);
    for (int i = 0; i < 14; i++) 
    {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial1.write(buf, 14);
    delay(10);
}

// void FINGER::change_mode(int id)
// {
//     checksum = 0;

//     buf[0] = 0xFF;
//     buf[1] = 0xFF;
//     buf[2] = 0x01;
//     buf[3] = 0x04;
//     buf[4] = 0x03;
//     buf[5] = 0x4D;
//     buf[6] = 0x01;

//     for(int i = 2; i < 7; i++)
//     {
//         checksum += buf[i];
//     }
//     buf[7] = Cal_chechsum_v2(checksum);
//     for (int i = 0; i < 8; i++) 
//     {
//         Serial.print(buf[i], HEX);
//         Serial.print(" ");
//     }
//     Serial.println();
//     Serial1.write(buf, 8);
//     delay(10);
// }

// void FINGER::CCW(int id, int speed)
// {
//     checksum = 0;
//     //int pos = round((angle / 360.0)*_one_spin);   //4095
//     // if (pos < 0 ){
//     //   pos = 4095 + pos;
//     // }
//     // Serial.print("pos : ");
//     // Serial.println(pos);
//     buf[0] = 0xFF;
//     buf[1] = 0xFF;
//     buf[2] = 0x01;
//     buf[3] = 0x05;
//     buf[4] = 0x03;
//     buf[5] = 0x4E;
//     buf[6] = (speed >> 0) & 0xFF;
//     buf[7] = (speed >> 8) & 0xFF;
//     for(int i = 2; i < 8; i++)
//     {
//         checksum += buf[i];
//     }
//     buf[8] = Cal_chechsum_v2(checksum);
//     for (int i = 0; i < 9; i++) 
//     {
//         Serial.print(buf[i], HEX);
//         Serial.print(" ");
//     }
//     Serial.println();
//     Serial1.write(buf, 9);
//     delay(10);
// }



void FINGER::ReadPOS(int id)
{
  if (id < 1 || id > 10) return;
  checksum = 0;
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = id;
  buf[3] = 0X04;
  buf[4] = 0x02;
  buf[5] = 0x38;
  buf[6] = 0x02;
  for (int i = 2; i < 7; i++)
  {
    checksum += buf[i];
  }
  buf[7] = Cal_chechsum_v2(checksum);
  // Serial.print("send : ");
  // for (int i = 0; i < 8; i++)
  // {
  //   Serial.print(buf[i],HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  Serial1.write(buf,8);
  delay(10);

  if (Serial1.available())
  {
     Serial1.readBytes(readin, 8);
        // Serial.print("Receive : ");
        if(readin[0] == 0xFF)
        {
            if(readin[1] == 0xFF)
            {
                for(int i = 0; i < 8; i++)
                {
                    // Serial.print(readin[i], HEX);
                    // Serial.print(" ");
                }
                // Serial.println();
            }
            
        }
        angles[id-1] = (readin[5] + (readin[6] << 8))*360.0 / (_one_spin);
        // Serial.print("angle = ");
        Serial.println(angles[id-1], 5);
  }
  else 
  {
    Serial.println("No data received");
  }
}

void FINGER::ReadTorque(int id)
{
  if (id < 1 || id > 10) return;
  checksum = 0;
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = id;
  buf[3] = 0X04;
  buf[4] = 0x02;
  buf[5] = 0x3C;
  buf[6] = 0x02;
  for (int i = 2; i < 7; i++)
  {
    checksum += buf[i];
  }
  buf[7] = Cal_chechsum_v2(checksum);
  // Serial.print("send : ");
  // for (int i = 0; i < 8; i++)
  // {
  //   Serial.print(buf[i],HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  Serial1.write(buf,8);
  delay(10);

  if (Serial1.available())
  {
      Serial1.readBytes(readin, 8);
      // Serial.print("Receive : ");
      if(readin[0] == 0xFF)
        {
            if(readin[1] == 0xFF)
            {
                for(int i = 0; i < 8; i++)
                {
                    // Serial.print(readin[i], HEX);
                    // Serial.print(" ");
                }
                // Serial.println();
            }
            
        }
        if (readin[6] == 0x00)
        {
            torques[id-1] = readin[5];
        }
        else if (readin[6] >= 0x01 && readin[6] <= 0x03)
        {
            torques[id-1] = readin[5] + (readin[6] << 8);
        }
        else if (readin[6] == 0x04)
        {
            torques[id-1] = readin[5] * (-1);
        }
        else if (readin[6] >= 0x05 && readin[6] <= 0x07)
        {
            torques[id-1] = (readin[5] + ((readin[6] & 0x03) << 8)) * (-1);
        }
        // Serial.print("Torque:");
        // Serial.println(torques[id-1]);
  }

        
}

// if (readin[6] == 0x00)
//     {
//         torques[id] = readin[5];
//     }
//     else if (readin[6] >= 0x01 && readin[6] <= 0x03)
//     {
//         torques[id] = readin[5] + (readin[6] << 8);
//     }
//     else if (readin[6] == 0x04)
//     {
//         torques[id] = readin[5] * (-1);
//     }
//     else if (readin[6] >= 0x05 && readin[6] <= 0x07)
//     {
//         torques[id] = (readin[5] + ((readin[6] & 0x03) << 8)) * (-1);
//     }
//     Serial.print("Torque:");
//     Serial.println(torques[id-1]);
// }

uint16_t FINGER::Cal_chechsum_v2(uint16_t checknum)
{
    // Serial.println(checksum, HEX);
    checksum &= 0xFF;
    // Serial.println(checksum, HEX);
    checksum ^= 0xFF;
    // Serial.println(checksum, HEX);

  return checksum;
}