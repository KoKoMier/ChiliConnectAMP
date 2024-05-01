#pragma once
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <iomanip>
#include <cmath>

class MTF02
{
public:
    struct MTF02Data
    {
        float Speed_X;
        float Speed_Y;
        float Pos_X;
        float Pos_Y;
        float Distance;
        int Distance_Strength;
        int Pos_Strength;
    };

    inline MTF02(const char *i2cDevice, uint8_t i2caddr)
    {

        if ((MTF02FD = open(i2cDevice, O_RDWR)) < 0)
            throw -1;
        if (ioctl(MTF02FD, I2C_SLAVE, i2caddr) < 0)
            throw -2;
        if (ioctl(MTF02FD, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
            throw -2;

        char input = 0x00;
        ssize_t bytesWritten = write(MTF02FD, &input, 1);
        if (bytesWritten < 1)
        {
            std::cerr << "write error\n";
        }
        read(MTF02FD, &ID, 1);
        if (ID == 0x0F)
        {
            std::cout << "ID " << std::dec << (unsigned int)ID << "\r\n";
        }
    }

    inline MTF02Data MTF02DataGet()
    {
        MTF02DataRead();
        Data.Pos_X += Data.Speed_X / 200;
        Data.Pos_Y += Data.Speed_Y / 200;
        return Data;
    }

private:
    inline void MTF02DataRead()
    {
        // Height
        {
            uint8_t D[] = {0, 0, 0};
            char input = 0x01;
            ssize_t bytesWritten = write(MTF02FD, &input, 1);
            read(MTF02FD, &D, 3);
            Data.Distance = (uint32_t)D[0] << 8 | (uint32_t)D[1];
            Data.Distance_Strength = D[2];
        }
        // Pos
        {
            uint8_t D[] = {0, 0, 0, 0, 0};
            char input = 0x06;
            ssize_t bytesWritten = write(MTF02FD, &input, 1);
            read(MTF02FD, &D, 5);
            float Data_X = (uint32_t)D[0] << 8 | (uint32_t)D[1];
            float Data_Y = (uint32_t)D[2] << 8 | (uint32_t)D[3];
            Data.Pos_Strength = D[4];
            if (Data_X > 32768)
                Data_X -= 65535;
            if (Data_Y > 32768)
                Data_Y -= 65535;
            Data.Speed_X = Data_X * Data.Distance / 1000;
            Data.Speed_Y = Data_Y * Data.Distance / 1000;
        }
    }

    uint8_t ID;
    int MTF02FD;
    float Distance;
    MTF02Data Data;
};