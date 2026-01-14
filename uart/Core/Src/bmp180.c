#include "stm32f1xx_hal.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
#define BMP180_I2C &hi2c1

#define BMP180_ADDRESS (0x77 << 1)

short AC1=0;
short AC2=0;
short AC3=0;
unsigned short AC4=0;
unsigned short AC5=0;
unsigned short AC6=0;   
short B1=0;
short B2=0;
short MB=0;
short MC=0;
short MD=0;

long UT=0;
short OSS=0;
long UP=0;
long X1=0;
long X2=0;
long X3=0;
long B3 =0;
long B5 =0;
unsigned long B4 =0;
long B6 =0;
unsigned long B7 =0;

long press =0;
long temp =0;

#define atmPress 101325 

void readCalibrationData()
{
    uint8_t callib_data[22] = {0};
    uint8_t callib_start = 0xAA;
    HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, callib_start, 1, callib_data, 22, HAL_MAX_DELAY);
    
    AC1 = (callib_data[0] << 8) | callib_data[1];
    AC2 = (callib_data[2] << 8) | callib_data[3];
    AC3 = (callib_data[4] << 8) | callib_data[5];
    AC4 = (callib_data[6] << 8) | callib_data[7];
    AC5 = (callib_data[8] << 8) | callib_data[9];
    AC6 = (callib_data[10] << 8) | callib_data[11];
    B1  = (callib_data[12] << 8) | callib_data[13];
    B2  = (callib_data[14] << 8) | callib_data[15];
    MB  = (callib_data[16] << 8) | callib_data[17];
    MC  = (callib_data[18] << 8) | callib_data[19];
    MD  = (callib_data[20] << 8) | callib_data[21];
}

uint16_t Get_UTemp(void)
{
    uint8_t datatowrite = 0x2E;
    uint8_t Temp_RAW[2] = {0};
    
    HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
    HAL_Delay(5);
    HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Temp_RAW, 2, 1000);
    
    return (Temp_RAW[0] << 8) + Temp_RAW[1];
}
float BMP180_GetTemp(void)
{
    UT = Get_UTemp();
    
    X1 = ((UT - AC6) * AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    temp = (B5 + 8) >> 4;
    
    return temp / 10.0;
}

uint16_t Get_UPress(void)
{
    uint8_t datatowrite = 0x34 + (OSS << 6);
    uint8_t Press_RAW[3] = {0};
    
    HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
    switch (OSS)
    {
        case 0: HAL_Delay(5);  break;
        case 1: HAL_Delay(8);  break;
        case 2: HAL_Delay(14); break;
        case 3: HAL_Delay(26); break;
        default: HAL_Delay(26); break;
    }
    HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Press_RAW, 3, 1000);
    
    return ((Press_RAW[0] << 16) + (Press_RAW[1] << 8) + Press_RAW[2]) >> (8 - OSS);
}

float BMP180_GetPress(void)
{
    UP = Get_UPress();
    
    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6 >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((((long)AC1) * 4 + X3) << OSS) + 2) >> 2;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
    B7 = ((unsigned long)(UP - B3) * (50000 >> OSS));
    
    if (B7 < 0x80000000)
        press = (B7 << 1) / B4;
    else
        press = (B7 / B4) << 1;
    
    X1 = (press >> 8) * (press >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * press) >> 16;
    press = press + ((X1 + X2 + 3791) >> 4);
    
    return press / 100.0;
}

float BMP180_GetAltitude(void)
{
    float altitude = 44330.0 * (1.0 - pow((BMP180_GetPress() * 100) / atmPress, 0.1903));
    return altitude;
}

void BMP180_Start(void)
{
    readCalibrationData();
}
