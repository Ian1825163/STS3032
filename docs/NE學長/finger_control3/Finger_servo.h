#ifndef Finger_H
#define Finger_H

#include "Arduino.h"
#define BAUDRATE 1000000

class FINGER
{
    public:
        FINGER();
        FINGER(int serial, int one_spin);
        // set_baudrate(int serial, int baudrate);
        void angle_control(int id, float angle, int speed);
        void angle_control_acc(int id, float angle, int speed, int acc);
        void ReadPOS(int id);
        void ReadTorque(int id);
        int getTorque(int id);
        float getAngle(int id);
        // void change_mode(int id);
        // void CCW(int id, int speed);

    private:
        int _serial = 0;
        int _baudrate = BAUDRATE;
        int _one_spin = 0;
        byte buf[20];
        uint16_t checksum = 0;
        uint16_t Cal_chechsum_v2(uint16_t checksum);
        byte readin[20] = {0};
        // float angle = 0;
        // int torque = 0;
        float angles[11] = {0};  // 0 1 2 3 4 5 6 7 8 9 10 
        int torques[11] = {0};

};

#endif