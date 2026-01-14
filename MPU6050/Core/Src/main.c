#include "main.h"
#include <string.h>
#include <stdio.h>

/* ================= MPU6050 REGISTERS ================= */
#define MPU6050_ADDR        (0x68 << 1)
#define WHO_AM_I            0x75
#define PWR_MGMT_1          0x6B
#define ACCEL_XOUT_H        0x3B

/* ================= HANDLES ================= */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* ================= CALIBRATION OFFSETS ================= */
float acc_off_x = 0, acc_off_y = 0, acc_off_z = 0;
float gyr_off_x = 0, gyr_off_y = 0, gyr_off_z = 0;

/* ================= STATE FLAGS ================= */
uint8_t mpu_ready = 0;
uint8_t calibrated = 0;

/* ================= PROTOTYPES ================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

void printu(const char *msg);
uint8_t MPU6050_Detect(void);
void MPU6050_Init(void);
void MPU6050_Calibrate(void);
void MPU6050_Read(void);
void I2C_Recover(void);

/* ================= MAIN ================= */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  HAL_Delay(200);

  printu("\r\n===== MPU6050 FULL TEST =====\r\n");

  if (MPU6050_Detect())
  {
    MPU6050_Init();
    HAL_Delay(500);        // allow sensor to stabilize
    MPU6050_Calibrate();
    calibrated = 1;
    mpu_ready = 1;
  }
  else
  {
    printu("MPU6050 NOT AVAILABLE ❌\r\n");
  }

  while (1)
  {
    if (mpu_ready && calibrated)
    {
      MPU6050_Read();
    }
    HAL_Delay(200);
  }
}

/* ================= UART ================= */
void printu(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* ================= I2C RECOVERY ================= */
void I2C_Recover(void)
{
  HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(10);
  MX_I2C1_Init();
}

/* ================= DETECT ================= */
uint8_t MPU6050_Detect(void)
{
  uint8_t id = 0;
  char buf[40];

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                       WHO_AM_I, 1,
                       &id, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    printu("I2C ERROR (WHO_AM_I)\r\n");
    I2C_Recover();
    return 0;
  }

  sprintf(buf, "WHO_AM_I = 0x%02X\r\n", id);
  printu(buf);

  return (id == 0x68);
}

/* ================= INIT ================= */
void MPU6050_Init(void)
{
  uint8_t data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,
                    PWR_MGMT_1, 1,
                    &data, 1, HAL_MAX_DELAY);
  printu("MPU6050 INITIALIZED ✅\r\n");
}

/* ================= CALIBRATION ================= */
void MPU6050_Calibrate(void)
{
  uint8_t raw[14];
  int16_t ax, ay, az, gx, gy, gz;
  const int samples = 500;

  printu("Calibrating... KEEP SENSOR STILL\r\n");

  for (int i = 0; i < samples; i++)
  {
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                         ACCEL_XOUT_H, 1,
                         raw, 14, HAL_MAX_DELAY) != HAL_OK)
    {
      I2C_Recover();
      i--;
      continue;
    }

    ax = (raw[0] << 8) | raw[1];
    ay = (raw[2] << 8) | raw[3];
    az = (raw[4] << 8) | raw[5];
    gx = (raw[8] << 8) | raw[9];
    gy = (raw[10] << 8) | raw[11];
    gz = (raw[12] << 8) | raw[13];

    acc_off_x += ax;
    acc_off_y += ay;
    acc_off_z += (az - 16384);   // gravity removed

    gyr_off_x += gx;
    gyr_off_y += gy;
    gyr_off_z += gz;

    HAL_Delay(2);
  }

  acc_off_x /= samples;
  acc_off_y /= samples;
  acc_off_z /= samples;
  gyr_off_x /= samples;
  gyr_off_y /= samples;
  gyr_off_z /= samples;

  printu("Calibration DONE ✅\r\n\r\n");
}

/* ================= READ DATA ================= */
void MPU6050_Read(void)
{
  uint8_t raw[14];
  int16_t axr, ayr, azr, gxr, gyr, gzr;
  float ax, ay, az, gx, gy, gz;
  char buf[160];

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
                       ACCEL_XOUT_H, 1,
                       raw, 14, HAL_MAX_DELAY) != HAL_OK)
  {
    printu("I2C READ ERROR\r\n");
    I2C_Recover();
    return;
  }

  axr = (raw[0] << 8) | raw[1];
  ayr = (raw[2] << 8) | raw[3];
  azr = (raw[4] << 8) | raw[5];
  gxr = (raw[8] << 8) | raw[9];
  gyr = (raw[10] << 8) | raw[11];
  gzr = (raw[12] << 8) | raw[13];

  ax = (axr - acc_off_x) / 16384.0f;
  ay = (ayr - acc_off_y) / 16384.0f;
  az = (azr - acc_off_z) / 16384.0f;

  gx = (gxr - gyr_off_x) / 131.0f;
  gy = (gyr - gyr_off_y) / 131.0f;
  gz = (gzr - gyr_off_z) / 131.0f;

  sprintf(buf,
          "ACC[g] X:% .2f Y:% .2f Z:% .2f | "
          "GYRO[dps] X:% .2f Y:% .2f Z:% .2f\r\n",
          ax, ay, az, gx, gy, gz);

  printu(buf);
}

/* ================= CLOCK ================= */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o = {0};
  RCC_ClkInitTypeDef c = {0};

  o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  o.HSEState = RCC_HSE_ON;
  o.PLL.PLLState = RCC_PLL_ON;
  o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  o.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&o);

  c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV2;
  c.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2);
}

/* ================= I2C ================= */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  HAL_I2C_Init(&hi2c1);
}

/* ================= UART ================= */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart1);
}

/* ================= GPIO ================= */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* ================= ERROR ================= */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
