/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Game Controller — STM32 B-U585I-IOT02A
  *
  *  Architecture:
  *   - Sensor pipeline : ISM330DHCX component driver (Project 2)
  *   - Mouse-look      : gyro yaw/pitch → smoothed cursor deltas, ALWAYS on
  *                       (same math as the old annotation cursor, just no
  *                       longer gated behind a steady-hold entry state)
  *   - Button mapping  : dual-purpose, switches with "drawing mode"
  *
  *  Controls:
  *   Tilt / rotate board                → mouse look (always active, in
  *                                         BOTH modes — it doubles as the
  *                                         draw cursor in drawing mode)
  *
  *   NORMAL MODE (default):
  *     Hold button                      → W held (move forward)
  *     Release button                   → W released
  *
  *   Double-tap button                  → SPACE tapped once (host)
  *                                         + toggles into DRAWING MODE
  *                                         (green LED turns on)
  *
  *   DRAWING MODE:
  *     Hold button                      → Left mouse button held (draw)
  *     Release button                   → Left mouse button released
  *
  *   Double-tap button again            → SPACE tapped once (host)
  *                                         + toggles back to NORMAL MODE
  *                                         (green LED turns off)
  *
  *  Note: the FIRST tap of a double-tap is indistinguishable from a normal
  *  single press until the second tap arrives, so it will fire a brief
  *  W-down/W-up (or click-down/click-up) blip before the mode toggles.
  *  This trades a tiny blip for zero added latency on real single presses,
  *  which matters much more for FPS-style forward movement.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "b_u585i_iot02a_bus.h"
#include "ism330dhcx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── UART commands ── */
#define DEBOUNCE_MS                   50
#define DOUBLE_CLICK_MS               400   /* ms — max gap between two presses to count as a double-tap */

#define CMD_W_DOWN                    "W_DOWN\r\n"
#define CMD_W_UP                      "W_UP\r\n"
#define CMD_LCLICK_DOWN               "LCLICK_DOWN\r\n"
#define CMD_LCLICK_UP                 "LCLICK_UP\r\n"
#define CMD_SPACE_TAP                 "SPACE_TAP\r\n"

/* ── Mouse-look cursor ── */
#define MOUSE_SEND_INTERVAL           20     /* ms — cursor update rate */
#define GYRO_CURSOR_DEADZONE          500.0f /* mdps — ignore tremor/noise */
#define CURSOR_SENSITIVITY            22     /* pixels per (mdps x iteration), tune as needed */

/* ── ISM330DHCX I2C address (SA0 high on B-U585I-IOT02A = 0x6B) ── */
#define ISM330DHCX_I2C_ADDR           (0x6B << 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef  hcrc;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* ── ISM330DHCX driver object ── */
static ISM330DHCX_Object_t IsmObj;
static ISM330DHCX_Axes_t   AccAxes;    /* mg   — raw from driver (unused by game logic, kept for future use) */
static ISM330DHCX_Axes_t   GyroAxes;   /* mdps — raw from driver, drives mouse-look                          */

/* ── Mouse-look filter state (single EMA, same as old annotation cursor) ── */
static float    cursorFastX     = 0.0f;
static float    cursorFastZ     = 0.0f;
static uint32_t lastMouseSend   = 0;

/* ── Button state ── */
static uint8_t  buttonHeld       = 0;   /* debounced current button state       */
static uint8_t  lastButtonState  = 0;   /* previous DEBOUNCED state — for edge detection */
static uint8_t  lastRawButton    = 0;   /* previous RAW pin reading — for debounce timer  */
static uint32_t lastDebounceTime = 0;

/* ── Mode + double-tap tracking ── */
static uint8_t  drawMode         = 0;   /* 0 = normal (W), 1 = drawing (LClick) */
static uint32_t lastClickTime    = 0;
static uint8_t  pressWasDoubleTap = 0;  /* true if the current press was consumed as a double-tap toggle */
static uint8_t  pressModeWasDraw  = 0;  /* mode that was active when the current press started            */

/* ── UART TX buffer ── */
static char txBuf[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
static void IMU_Init(void);
static void IMU_ReadSensors(void);
static void MouseLook_Update(void);
static void SendCommand(const char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Send UART string and blink red LED */
static void SendCommand(const char *cmd)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 100);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

/* ============================================================
   IMU INIT — ISM330DHCX component driver
   ODR: 104 Hz, ±4 g accel, ±500 dps gyro
   ============================================================ */
static void IMU_Init(void)
{
    ISM330DHCX_IO_t io;
    uint8_t id = 0;

    io.BusType  = ISM330DHCX_I2C_BUS;
    io.Address  = ISM330DHCX_I2C_ADDR;
    io.Init     = BSP_I2C2_Init;
    io.DeInit   = BSP_I2C2_DeInit;
    io.WriteReg = BSP_I2C2_WriteReg;
    io.ReadReg  = BSP_I2C2_ReadReg;
    io.GetTick  = BSP_GetTick;
    io.Delay    = HAL_Delay;

    HAL_UART_Transmit(&huart1, (uint8_t *)"IMU_CHECKING\r\n", 14, 100);

    if (ISM330DHCX_RegisterBusIO(&IsmObj, &io) != ISM330DHCX_OK)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"IMU_BUS_FAIL\r\n", 14, 100);
        while (1) { HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); HAL_Delay(200); }
    }

    ISM330DHCX_ReadID(&IsmObj, &id);
    snprintf(txBuf, sizeof(txBuf), "WHO_AM_I:0x%02X\r\n", id);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), 100);

    if (id != 0x6Bu)
    {
        snprintf(txBuf, sizeof(txBuf), "IMU_FAIL:0x%02X\r\n", id);
        HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), 100);
        while (1) { HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); HAL_Delay(200); }
    }

    if (ISM330DHCX_Init(&IsmObj) != ISM330DHCX_OK)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"IMU_INIT_FAIL\r\n", 15, 100);
        while (1) { HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); HAL_Delay(200); }
    }

    /* Accel: 104 Hz, ±4 g (kept enabled, not currently used by game logic) */
    ISM330DHCX_ACC_SetOutputDataRate(&IsmObj, 104.0f);
    ISM330DHCX_ACC_SetFullScale(&IsmObj, 4);
    ISM330DHCX_ACC_Enable(&IsmObj);

    /* Gyro: 104 Hz, ±500 dps — drives mouse-look */
    ISM330DHCX_GYRO_SetOutputDataRate(&IsmObj, 104.0f);
    ISM330DHCX_GYRO_SetFullScale(&IsmObj, 500);
    ISM330DHCX_GYRO_Enable(&IsmObj);

    HAL_Delay(100);
    HAL_UART_Transmit(&huart1, (uint8_t *)"IMU_OK\r\n", 8, 100);
}

/* ============================================================
   IMU READ
   ============================================================ */
static void IMU_ReadSensors(void)
{
    ISM330DHCX_ACC_GetAxes(&IsmObj,  &AccAxes);
    ISM330DHCX_GYRO_GetAxes(&IsmObj, &GyroAxes);
}

/* ============================================================
   MOUSE LOOK — always active, in both normal and drawing mode.
   Same gyro EMA cursor math as the old annotation cursor.
   ============================================================ */
static void MouseLook_Update(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lastMouseSend) < MOUSE_SEND_INTERVAL)
        return;
    lastMouseSend = now;

    /* GyroAxes.z (mdps) = yaw   → horizontal
       GyroAxes.x (mdps) = pitch → vertical
       Gyro reads near zero when still → instant stop, no drift */
    float gyroX = -(float)GyroAxes.z;   /* yaw   → horizontal (flipped) */
    float gyroY = -(float)GyroAxes.x;   /* pitch → vertical             */

    if (gyroX > -GYRO_CURSOR_DEADZONE && gyroX < GYRO_CURSOR_DEADZONE) gyroX = 0.0f;
    if (gyroY > -GYRO_CURSOR_DEADZONE && gyroY < GYRO_CURSOR_DEADZONE) gyroY = 0.0f;

    /* Single EMA (alpha = 0.4) for smoothing */
    cursorFastX = (4.0f * gyroX + 6.0f * cursorFastX) / 10.0f;
    cursorFastZ = (4.0f * gyroY + 6.0f * cursorFastZ) / 10.0f;

    int32_t dx =  (int32_t)(cursorFastX / (float)(CURSOR_SENSITIVITY * 30));
    int32_t dy = -(int32_t)(cursorFastZ / (float)(CURSOR_SENSITIVITY * 30));

    if (dx != 0 || dy != 0)
    {
        snprintf(txBuf, sizeof(txBuf), "MOUSE_MOVE:%ld,%ld\r\n", (long)dx, (long)dy);
        HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), 50);
    }
}

/* ============================================================
   BUTTON HANDLING — press/release maps to W or Left-click
   depending on drawMode; double-tap toggles drawMode + taps space.
   ============================================================ */
static void Button_Update(void)
{
    uint8_t currentButton =
        (HAL_GPIO_ReadPin(USER_Button_GPIO_Port, USER_Button_Pin)
         == GPIO_PIN_SET) ? 1u : 0u;

    if (currentButton != lastRawButton)
        lastDebounceTime = HAL_GetTick();
    lastRawButton = currentButton;

    if ((HAL_GetTick() - lastDebounceTime) > DEBOUNCE_MS)
        buttonHeld = currentButton;

    /* Rising edge */
    if (buttonHeld && !lastButtonState)
    {
        uint32_t now = HAL_GetTick();

        if ((now - lastClickTime) < DOUBLE_CLICK_MS)
        {
            /* Double-tap → toggle mode, tap space, consume this press
               (no W/LClick down for this particular press)            */
            drawMode = !drawMode;
            SendCommand(CMD_SPACE_TAP);
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
                              drawMode ? GPIO_PIN_SET : GPIO_PIN_RESET);
            lastClickTime      = 0;   /* prevent a 3rd quick press chaining into another toggle */
            pressWasDoubleTap  = 1;
        }
        else
        {
            pressWasDoubleTap = 0;
            pressModeWasDraw  = drawMode;
            SendCommand(pressModeWasDraw ? CMD_LCLICK_DOWN : CMD_W_DOWN);
            lastClickTime = now;
        }
    }

    /* Falling edge */
    if (!buttonHeld && lastButtonState)
    {
        if (!pressWasDoubleTap)
            SendCommand(pressModeWasDraw ? CMD_LCLICK_UP : CMD_W_UP);
        pressWasDoubleTap = 0;
    }

    lastButtonState = buttonHeld;
}

/* USER CODE END 0 */

/* ============================================================
   APPLICATION ENTRY POINT
   ============================================================ */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemPower_Config();
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_ICACHE_Init();
    MX_USART1_UART_Init();
    MX_CRC_Init();

    /* USER CODE BEGIN 2 */

    HAL_UART_Transmit(&huart1, (uint8_t *)"BOOT\r\n", 6, 100);
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart1, (uint8_t *)"STARTING_IMU\r\n", 14, 100);

    /* BSP I2C2 init */
    int32_t bspRet = BSP_I2C2_Init();
    snprintf(txBuf, sizeof(txBuf), "BSP_I2C2_INIT:%s\r\n",
             bspRet == BSP_ERROR_NONE ? "OK" : "FAIL");
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, strlen(txBuf), 100);

    /* IMU init via ISM330DHCX component driver */
    IMU_Init();

    /* Seed sensor read */
    IMU_ReadSensors();

    /* State reset */
    buttonHeld        = 0;
    lastButtonState    = 0;
    drawMode           = 0;
    lastClickTime       = 0;
    pressWasDoubleTap   = 0;
    lastMouseSend       = HAL_GetTick();
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)"READY - GAME CONTROL ACTIVE\r\n", 30, 100);

    /* USER CODE END 2 */

    /* ============================================================
       MAIN LOOP
       ============================================================ */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        Button_Update();

        IMU_ReadSensors();
        MouseLook_Update();

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* ============================================================
   PERIPHERAL INITIALISATION — unchanged from CubeMX generation
   ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_4;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLMBOOST      = RCC_PLLMBOOST_DIV1;
    RCC_OscInitStruct.PLL.PLLM           = 1;
    RCC_OscInitStruct.PLL.PLLN           = 80;
    RCC_OscInitStruct.PLL.PLLP           = 2;
    RCC_OscInitStruct.PLL.PLLQ           = 2;
    RCC_OscInitStruct.PLL.PLLR           = 2;
    RCC_OscInitStruct.PLL.PLLRGE         = RCC_PLLVCIRANGE_0;
    RCC_OscInitStruct.PLL.PLLFRACN       = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                     | RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

static void SystemPower_Config(void)
{
    HAL_PWREx_EnableVddIO2();
    if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
        Error_Handler();
}

static void MX_CRC_Init(void)
{
    hcrc.Instance                     = CRC;
    hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
        Error_Handler();
}

static void MX_ICACHE_Init(void)
{
    if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
        Error_Handler();
    if (HAL_ICACHE_Enable() != HAL_OK)
        Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 115200;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    HAL_GPIO_WritePin(UCPD_PWR_GPIO_Port, UCPD_PWR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, LED_RED_Pin | LED_GREEN_Pin | Mems_VL53_xshut_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WRLS_WKUP_B_GPIO_Port, WRLS_WKUP_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, Mems_STSAFE_RESET_Pin | WRLS_WKUP_W_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = WRLS_FLOW_Pin | Mems_VLX_GPIO_Pin | Mems_INT_LPS22HH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = WRLS_UART4_RX_Pin | WRLS_UART4_TX_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = USB_UCPD_CC1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_UCPD_CC1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_F_NCS_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
    HAL_GPIO_Init(OCTOSPI_F_NCS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_R_IO5_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO5_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_F_IO7_Pin | OCTOSPI_F_IO5_Pin
                              | OCTOSPI_F_IO6_Pin | OCTOSPI_F_IO4_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = PH3_BOOT0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PH3_BOOT0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = UCPD_PWR_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(UCPD_PWR_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = WRLS_SPI2_MOSI_Pin | WRLS_SPI2_MISO_Pin | WRLS_SPI2_SCK_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_R_DQS_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_DQS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_R_IO7_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO7_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_F_IO0_Pin | OCTOSPI_F_IO1_Pin
                              | OCTOSPI_F_IO2_Pin  | OCTOSPI_F_IO3_Pin
                              | OCTOSPI_F_CLK_P_Pin | OCTOSPI_F_DQS_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = USER_Button_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_Button_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LED_RED_Pin | LED_GREEN_Pin | Mems_VL53_xshut_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_R_IO0_Pin | OCTOSPI_R_IO2_Pin
                              | OCTOSPI_R_IO1_Pin  | OCTOSPI_R_IO3_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_R_IO4_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = USB_C_P_Pin | USB_C_PA11_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_USB;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = MIC_CCK1_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
    HAL_GPIO_Init(MIC_CCK1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = MIC_SDINx_Pin | MIC_CCK0_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_ADF1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = WRLS_WKUP_B_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WRLS_WKUP_B_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = WRLS_NOTIFY_Pin | Mems_INT_IIS2MDC_Pin | USB_IANA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_R_IO6_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO6_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = USB_UCPD_FLT_Pin | Mems_ISM330DLC_INT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = OCTOSPI_R_CLK_P_Pin | OCTOSPI_R_NCS_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = USB_VBUS_SENSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = WRLS_SPI2_NSS_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(WRLS_SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = USB_UCPD_CC2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_UCPD_CC2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = Mems_STSAFE_RESET_Pin | WRLS_WKUP_W_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = MIC_SDIN0_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
    HAL_GPIO_Init(MIC_SDIN0_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    (void)file;
    (void)line;
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
