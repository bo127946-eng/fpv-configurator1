/*
 * Project: FPV Flight Controller Firmware (ST Bare-Metal / C++17)
 * Version: 4.40
 * Target: STM32F103CBT6 (128K Flash / 20K SRAM)
 * Peripherals: TIM2 (PA0-PA3), ADC1 (PA4, PA5), USART1 (PA9/PA10),
 *              USART3 (PB10/PB11 VTX), SPI2 (PB12-PB15, PB0, PB1 RTX),
 *              I2C1 (PB6/PB7), Flash page 127 (0x0801FC00) config storage.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ==============================================================================
// 1. РЕГИСТРЫ STM32F103
// ==============================================================================
namespace Reg {
  struct GPIO_TypeDef {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
  };

  struct RCC_TypeDef {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
  };

  struct FLASH_TypeDef {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t AR;
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;
    volatile uint32_t WRPR;
  };

  struct TIM_TypeDef {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
  };

  struct USART_TypeDef {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
  };

  struct SPI_TypeDef {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
  };

  struct I2C_TypeDef {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
  };

  struct ADC_TypeDef {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
  };

  struct STK_TypeDef {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
  };

  constexpr uintptr_t PERIPH_BASE = 0x40000000UL;
  constexpr uintptr_t APB1PERIPH_BASE = PERIPH_BASE;
  constexpr uintptr_t APB2PERIPH_BASE = PERIPH_BASE + 0x00010000UL;
  constexpr uintptr_t AHBPERIPH_BASE  = PERIPH_BASE + 0x00020000UL;

  inline RCC_TypeDef*    const RCC    = reinterpret_cast<RCC_TypeDef*>(AHBPERIPH_BASE + 0x1000UL);
  inline FLASH_TypeDef*  const FLASH  = reinterpret_cast<FLASH_TypeDef*>(AHBPERIPH_BASE + 0x2000UL);
  inline GPIO_TypeDef*   const GPIOA  = reinterpret_cast<GPIO_TypeDef*>(APB2PERIPH_BASE + 0x0800UL);
  inline GPIO_TypeDef*   const GPIOB  = reinterpret_cast<GPIO_TypeDef*>(APB2PERIPH_BASE + 0x0C00UL);
  inline ADC_TypeDef*    const ADC1   = reinterpret_cast<ADC_TypeDef*>(APB2PERIPH_BASE + 0x2400UL);
  inline USART_TypeDef*  const USART1 = reinterpret_cast<USART_TypeDef*>(APB2PERIPH_BASE + 0x3800UL);
  inline TIM_TypeDef*    const TIM2   = reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x0000UL);
  inline SPI_TypeDef*    const SPI2   = reinterpret_cast<SPI_TypeDef*>(APB1PERIPH_BASE + 0x3800UL);
  inline USART_TypeDef*  const USART3 = reinterpret_cast<USART_TypeDef*>(APB1PERIPH_BASE + 0x4800UL);
  inline I2C_TypeDef*    const I2C1   = reinterpret_cast<I2C_TypeDef*>(APB1PERIPH_BASE + 0x5400UL);
  inline STK_TypeDef*    const STK    = reinterpret_cast<STK_TypeDef*>(0xE000E010UL);
}

// ==============================================================================
// 2. СИСТЕМНОЕ ТАКТИРОВАНИЕ (72 MHz HSE), ЗАДЕРЖКИ И АППАРАТНЫЕ ИНТЕРФЕЙСЫ
// ==============================================================================
static volatile uint32_t g_ms_ticks = 0;
constexpr uint32_t SYSCLK_HZ = 72000000UL;
constexpr uint32_t FLASH_CFG_ADDR = 0x0801FC00UL;

extern "C" void SystemInit(void) {
  Reg::RCC->CR |= (1U << 16);
  while (!(Reg::RCC->CR & (1U << 17))) {}

  Reg::FLASH->ACR = (1U << 4) | 0x02U;

  Reg::RCC->CFGR &= ~((0xFUL << 18) | (1U << 16) | (0xFUL << 4) | (0x3UL << 14));
  Reg::RCC->CFGR |= (1U << 16) | (7U << 18) | (4U << 8) | (2U << 14);

  Reg::RCC->CR |= (1U << 24);
  while (!(Reg::RCC->CR & (1U << 25))) {}

  Reg::RCC->CFGR &= ~0x3UL;
  Reg::RCC->CFGR |= 0x2UL;
  while ((Reg::RCC->CFGR & 0xCUL) != 0x8UL) {}

  Reg::STK->LOAD = (SYSCLK_HZ / 1000U) - 1U;
  Reg::STK->VAL = 0U;
  Reg::STK->CTRL = 0x07U;
}

extern "C" void SysTick_Handler(void) {
  g_ms_ticks++;
}

static inline uint32_t millis() {
  return g_ms_ticks;
}

static inline uint32_t micros() {
  uint32_t ms, val;
  do {
    ms = g_ms_ticks;
    val = Reg::STK->VAL;
  } while (ms != g_ms_ticks);
  uint32_t load = Reg::STK->LOAD;
  return ms * 1000UL + ((load - val) * 1000UL) / (load + 1UL);
}

static void delay(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms) {}
}

static inline float constrainF(float v, float lo, float hi) {
  if (!isfinite(v)) return lo;
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline long constrainL(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int constrainI(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ==============================================================================
// 3. GPIO, UART, SPI2, I2C1, ADC1, TIM2
// ==============================================================================
enum GpioMode : uint8_t {
  GPIO_IN_FLOATING = 0x4,
  GPIO_IN_PULL     = 0x8,
  GPIO_OUT_PP_50M  = 0x3,
  GPIO_OUT_OD_50M  = 0x7,
  GPIO_AF_PP_50M   = 0xB,
  GPIO_AF_OD_50M   = 0xF
};

static void gpioConfigPin(Reg::GPIO_TypeDef* port, uint8_t pin, GpioMode mode, bool pull_up = true) {
  if (pin < 8) {
    uint32_t shift = pin * 4U;
    port->CRL &= ~(0xFUL << shift);
    port->CRL |= ((uint32_t)mode << shift);
  } else {
    uint32_t shift = (pin - 8U) * 4U;
    port->CRH &= ~(0xFUL << shift);
    port->CRH |= ((uint32_t)mode << shift);
  }
  if (mode == GPIO_IN_PULL) {
    if (pull_up) port->BSRR = (1U << pin);
    else port->BRR = (1U << pin);
  }
}

static inline void gpioWrite(Reg::GPIO_TypeDef* port, uint8_t pin, bool high) {
  if (high) port->BSRR = (1U << pin);
  else port->BRR = (1U << pin);
}

static inline bool gpioRead(Reg::GPIO_TypeDef* port, uint8_t pin) {
  return (port->IDR & (1U << pin)) != 0;
}

class UartPort {
public:
  explicit UartPort(Reg::USART_TypeDef* dev, uint32_t pclk_hz) : dev_(dev), pclk_(pclk_hz) {}

  void begin(uint32_t baud) {
    if (baud == 0) baud = 115200;
    dev_->CR1 &= ~(1U << 13);
    dev_->BRR = (pclk_ + (baud / 2U)) / baud;
    dev_->CR1 = (1U << 2) | (1U << 3) | (1U << 13);
  }

  void end() {
    dev_->CR1 &= ~(1U << 13);
  }

  void flush() {
    while (!(dev_->SR & (1U << 6))) {}
  }

  int available() {
    return (dev_->SR & (1U << 5)) ? 1 : 0;
  }

  int read() {
    if (!(dev_->SR & (1U << 5))) return -1;
    return (int)(dev_->DR & 0xFFU);
  }

  size_t write(uint8_t b) {
    uint32_t s = millis();
    while (!(dev_->SR & (1U << 7))) {
      if ((millis() - s) > 20) return 0;
    }
    dev_->DR = b;
    return 1;
  }

  size_t write(const uint8_t* buf, size_t len) {
    size_t n = 0;
    for (size_t i = 0; i < len; i++) {
      if (write(buf[i]) == 1) n++;
      else break;
    }
    return n;
  }

  void print(const char* s) {
    while (s && *s) write((uint8_t)*s++);
  }

  void print(int v) {
    char b[16]; snprintf(b, sizeof(b), "%d", v); print(b);
  }

  void print(unsigned int v) {
    char b[16]; snprintf(b, sizeof(b), "%u", v); print(b);
  }

  void print(float v, int dec = 2) {
    if (!isfinite(v)) { print("nan"); return; }
    char fmt[12]; snprintf(fmt, sizeof(fmt), "%%.%df", dec);
    char b[24]; snprintf(b, sizeof(b), fmt, (double)v); print(b);
  }

  void println(const char* s = "") { print(s); print("\r\n"); }
  void println(int v) { print(v); print("\r\n"); }
  void println(unsigned int v) { print(v); print("\r\n"); }
  void println(float v, int dec = 2) { print(v, dec); print("\r\n"); }

private:
  Reg::USART_TypeDef* dev_;
  uint32_t pclk_;
};

static UartPort uart1(Reg::USART1, SYSCLK_HZ);
static UartPort uart3(Reg::USART3, SYSCLK_HZ / 2U);

static void initSpi2() {
  gpioConfigPin(Reg::GPIOB, 13, GPIO_AF_PP_50M);
  gpioConfigPin(Reg::GPIOB, 14, GPIO_IN_FLOATING);
  gpioConfigPin(Reg::GPIOB, 15, GPIO_AF_PP_50M);
  gpioConfigPin(Reg::GPIOB, 12, GPIO_OUT_PP_50M);
  gpioConfigPin(Reg::GPIOB, 0,  GPIO_OUT_PP_50M);
  gpioConfigPin(Reg::GPIOB, 1,  GPIO_IN_PULL, false);
  gpioWrite(Reg::GPIOB, 12, true);
  gpioWrite(Reg::GPIOB, 0, true);

  Reg::SPI2->CR1 = (1U << 2) | (2U << 3) | (1U << 8) | (1U << 9) | (1U << 6);
}

static inline void spi2BeginTx() {
  gpioWrite(Reg::GPIOB, 12, false);
}

static inline void spi2EndTx() {
  while (Reg::SPI2->SR & (1U << 7)) {}
  gpioWrite(Reg::GPIOB, 12, true);
}

static inline uint8_t spi2Transfer(uint8_t b) {
  uint32_t s = micros();
  while (!(Reg::SPI2->SR & (1U << 1))) { if ((micros() - s) > 100) return 0; }
  Reg::SPI2->DR = b;
  s = micros();
  while (!(Reg::SPI2->SR & (1U << 0))) { if ((micros() - s) > 100) return 0; }
  return (uint8_t)(Reg::SPI2->DR & 0xFFU);
}

static void initI2c1() {
  gpioConfigPin(Reg::GPIOB, 6, GPIO_AF_OD_50M);
  gpioConfigPin(Reg::GPIOB, 7, GPIO_AF_OD_50M);

  Reg::I2C1->CR1 |= (1U << 15);
  Reg::I2C1->CR1 &= ~(1U << 15);

  Reg::I2C1->CR2 = 36U;
  Reg::I2C1->CCR = (1U << 15) | 45U;
  Reg::I2C1->TRISE = 12U;
  Reg::I2C1->CR1 |= 0x01U;
}

static bool i2c1WaitFlag(volatile uint32_t* reg, uint32_t mask, bool set, uint32_t timeout_us = 1000) {
  uint32_t s = micros();
  while ((((*reg) & mask) != 0) != set) {
    if ((micros() - s) > timeout_us) return false;
  }
  return true;
}

static bool i2c1WriteRead(uint8_t addr7, const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
  uint32_t s = micros();
  while (Reg::I2C1->SR2 & (1U << 1)) { if ((micros() - s) > 1000) return false; }

  Reg::I2C1->CR1 |= (1U << 8);
  if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 0), true)) return false;

  Reg::I2C1->DR = (uint8_t)((addr7 << 1) | 0U);
  if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 1), true)) return false;
  (void)Reg::I2C1->SR2;

  for (size_t i = 0; i < tx_len; i++) {
    if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 7), true)) return false;
    Reg::I2C1->DR = tx[i];
  }
  if (tx_len > 0 && !i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 2), true)) return false;

  if (rx_len > 0) {
    Reg::I2C1->CR1 |= (1U << 8);
    if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 0), true)) return false;
    Reg::I2C1->DR = (uint8_t)((addr7 << 1) | 1U);
    if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 1), true)) return false;
    (void)Reg::I2C1->SR2;

    if (rx_len == 1) {
      Reg::I2C1->CR1 &= ~(1U << 10);
      Reg::I2C1->CR1 |= (1U << 9);
      if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 6), true)) return false;
      rx[0] = (uint8_t)(Reg::I2C1->DR & 0xFFU);
    } else {
      Reg::I2C1->CR1 |= (1U << 10);
      for (size_t i = 0; i < rx_len; i++) {
        if (i == rx_len - 1) {
          Reg::I2C1->CR1 &= ~(1U << 10);
          Reg::I2C1->CR1 |= (1U << 9);
        }
        if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 6), true)) return false;
        rx[i] = (uint8_t)(Reg::I2C1->DR & 0xFFU);
      }
    }
  } else {
    Reg::I2C1->CR1 |= (1U << 9);
  }
  return true;
}

static void initAdc1() {
  gpioConfigPin(Reg::GPIOA, 4, GPIO_IN_FLOATING);
  gpioConfigPin(Reg::GPIOA, 5, GPIO_IN_FLOATING);

  Reg::ADC1->SMPR2 = (7U << (4U * 3U)) | (7U << (5U * 3U));
  Reg::ADC1->SQR1 = 0U;
  Reg::ADC1->CR2 = 0x01U;
  delay(1);
  Reg::ADC1->CR2 |= (1U << 2);
  while (Reg::ADC1->CR2 & (1U << 2)) {}
}

static uint16_t adc1ReadChannel(uint8_t ch) {
  Reg::ADC1->SQR3 = (ch & 0x1FU);
  Reg::ADC1->CR2 |= (1U << 0);
  uint32_t s = micros();
  while (!(Reg::ADC1->SR & (1U << 1))) { if ((micros() - s) > 200) return 0; }
  return (uint16_t)(Reg::ADC1->DR & 0x0FFFU);
}

// ==============================================================================
// 4. КОНСТАНТЫ, СТРУКТУРЫ И СОХРАНЕНИЕ В FLASH (СТРАНИЦА 127: 0x0801FC00)
// ==============================================================================
constexpr uint32_t LOOP_INTERVAL_US = 4000;
constexpr uint32_t LOOP_OVERRUN_MARGIN_US = 200;
constexpr float DT = 0.004f;
constexpr uint32_t AUX_SENSOR_PERIOD_MS = 40;
constexpr uint32_t MS5611_CONV_DELAY_MS = 10;
constexpr uint32_t BMP180_TEMP_DELAY_MS = 5;
constexpr uint32_t BMP180_PRESS_DELAY_MS = 8;
constexpr uint32_t RTX_TX_TIMEOUT_PRI_MS = 50;
constexpr uint32_t RTX_TX_TIMEOUT_BAK_MS = 220;
constexpr uint8_t RTX_SCAN_CHANNELS_PER_TICK = 10;
constexpr uint32_t RTX_SCAN_BUSY_TIMEOUT_US = 40;
constexpr uint32_t RTX_BUSY_SHORT_TIMEOUT_US = 30;
constexpr uint8_t RTX_RESP_QUEUE_SIZE = 4;
constexpr uint32_t ERR_202_REPEAT_MS = 1000;
constexpr uint32_t VTX_CMD_INTERVAL_MS = 70;
constexpr uint32_t VTX_PROBE_INTERVAL_MS = 2000;
constexpr uint32_t VTX_PROBE_TIMEOUT_MS = 250;
constexpr uint8_t RTX_RX_MAX_PAYLOAD_BYTES = 32;
constexpr uint32_t MAG_CAL_DURATION_MS = 8000;
constexpr uint32_t MAG_CAL_SAMPLE_MS = 20;
constexpr uint32_t MON_STREAM_PERIOD_MS = 80;
constexpr uint32_t BENCH_TEST_DURATION_MS = 10000;
constexpr float BENCH_TEST_THROTTLE = 100.0f;

constexpr float SHUNT_RESISTANCE_OHM = 0.01f;
constexpr uint32_t EEPROM_MAGIC_MARKER = 0xFACE1371UL;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint32_t SMARTAUDIO_BAUD_RATE = 4800;
constexpr uint32_t TRAMP_BAUD_RATE = 9600;

constexpr float STICK_CENTER = 500.0f;
constexpr float INITIAL_BASE_HOVER = 150.0f;
constexpr float OUTPUT_MIN = 0.0f;
constexpr float OUTPUT_MAX = 1000.0f;
constexpr float MAX_RATE_DPS_PER_UNIT = 500.0f;
constexpr float DEFAULT_FLT_AMB = 0.05f;
constexpr float DEFAULT_FLT_AMB_MIN = 0.003f;
constexpr float PID_D_ALPHA = 0.4f;
constexpr float FC_DEG_TO_RAD = 0.0174532925f;
constexpr float FC_RAD_TO_DEG = 57.29577951f;
constexpr float SIN_45 = 0.70710678f;
constexpr float COS_45 = 0.70710678f;
constexpr uint32_t FAILSAFE_TIMEOUT_MS = 1000;
constexpr uint8_t I2C_MAX_ERRORS = 10;
constexpr uint8_t RTX_MAX_ERRORS = 10;

// Безопасные лимиты параметров конфигурации (единый источник истины)
constexpr float CFG_ANGLE_MAX_DEG = 85.0f;
constexpr float CFG_VZ_MAX_MPS = 10.0f;
constexpr float CFG_RATE_MAX = 2.5f;
constexpr float CFG_EXPO_MAX = 1.0f;
constexpr float CFG_DEADBAND_MAX = 200.0f;
constexpr float CFG_ALPHA_MIN = 0.001f;
constexpr float CFG_ALPHA_MAX = 1.0f;
constexpr float CFG_NOTCH_FREQ_MIN = 20.0f;
constexpr float CFG_NOTCH_FREQ_MAX = 120.0f;
constexpr float CFG_NOTCH_BW_MIN = 1.0f;
constexpr float CFG_NOTCH_BW_MAX = 100.0f;
constexpr float CFG_ESC_LIMIT_MIN = 1.0f;
constexpr float CFG_ESC_LIMIT_MAX = 250.0f;
constexpr float CFG_BAT_V_MAX_MIN = 7.0f;
constexpr float CFG_BAT_V_MAX_MAX = 25.2f;
constexpr float CFG_BAT_FS_V_MIN = 5.0f;
constexpr float CFG_BAT_FS_V_MAX = 25.0f;
constexpr float CFG_BAT_SCALE_V_MIN = 0.1f;
constexpr float CFG_BAT_SCALE_V_MAX = 50.0f;
constexpr float CFG_BAT_OFFSET_A_MIN = -200.0f;
constexpr float CFG_BAT_OFFSET_A_MAX = 200.0f;
constexpr float CFG_BAT_SCALE_A_MIN = 0.05f;
constexpr float CFG_BAT_SCALE_A_MAX = 20.0f;
constexpr float CFG_LIDAR_OFFSET_MIN = -5.0f;
constexpr float CFG_LIDAR_OFFSET_MAX = 5.0f;
constexpr float CFG_MAG_BIAS_MIN = -2000.0f;
constexpr float CFG_MAG_BIAS_MAX = 2000.0f;
constexpr float CFG_RATE_KP_MAX = 10.0f;
constexpr float CFG_RATE_KI_MAX = 5.0f;
constexpr float CFG_RATE_KD_MAX = 1.0f;
constexpr float CFG_VZ_KP_MAX = 50.0f;
constexpr float CFG_VZ_KI_MAX = 20.0f;
constexpr float CFG_VZ_KD_MAX = 10.0f;

enum FcErrorCode : uint8_t {
  FC_ERR_NONE = 0, FC_ERR_LOOP_OVERRUN = 1, FC_ERR_IMU_FAIL = 2, FC_ERR_I2C_BUS = 3,
  FC_ERR_SPI_RTX = 4, FC_ERR_RC_LOSS = 5, FC_ERR_BAT_LOW = 6, FC_ERR_TILT_DISARM = 7,
  FC_ERR_RTX_TX_TIMEOUT = 8, FC_ERR_NAN_INF_MATH = 9, FC_ERR_BARO_FAIL = 10,
  FC_ERR_LIDAR_FAIL = 11, FC_ERR_MAG_FAIL = 12, FC_ERR_EEPROM_RESET = 13,
  FC_ERR_BAT_SENSOR_FAIL = 14, FC_ERR_SA_VTX_FAIL = 16, FC_ERR_CALIB_MISSING = 18
};

enum VtxProtSetting : uint8_t { VTX_PROT_AUTO = 0, VTX_PROT_TRAMP = 1, VTX_PROT_SMARTAUDIO = 2 };
enum VtxActiveProto : uint8_t { VTX_PROTO_NONE = 0, VTX_PROTO_TRAMP = 1, VTX_PROTO_SMARTAUDIO = 2 };
enum RtxChipId : uint8_t { RTX_CHIP_AUTO = 0, RTX_CHIP_SX1262 = 1, RTX_CHIP_SX1268 = 2, RTX_CHIP_LLCC68 = 3 };
enum VtxProbeStep : uint8_t { VTX_STEP_IDLE = 0, VTX_STEP_WAIT_SA, VTX_STEP_WAIT_TRAMP, VTX_STEP_WAIT_CMD_ACK };
enum DelayedRtxRespType : uint8_t {
  RTX_RESP_NONE = 0, RTX_RESP_202_ERR_EVENT = 202, RTX_RESP_230_SCAN_BEST = 230,
  RTX_RESP_240_CHAN_SW = 240, RTX_RESP_250_VTX_ACK = 250
};
enum BenchTestMode : uint8_t { BENCH_TEST_NONE = 0, BENCH_TEST_MOTORS = 1, BENCH_TEST_PID = 2 };

struct DelayedRtxRespItem { DelayedRtxRespType type = RTX_RESP_NONE; uint16_t vtx_mhz = 0; uint8_t tier = 0; uint16_t vtx_mw = 0; };
struct PIDConfig { float kp, ki, kd; };

struct __attribute__((aligned(4))) FlightConfig {
  uint32_t magic_marker;
  float gyro_bias_x, gyro_bias_y, gyro_bias_z;
  float base_temperature;
  float temp_coeff_x, temp_coeff_y, temp_coeff_z;
  float acc_bias_x, acc_bias_y, acc_bias_z;
  float mag_bias_x, mag_bias_y, mag_bias_z;
  float lidar_offset_m, bat_scale_v, bat_offset_a, bat_scale_a, bat_v_max, flt_esc_limit;
  float fm_roll_max, fm_pitch_max, fm_vz_speed_max;
  float gyro_filter_alpha, acc_filter_alpha, filter_notch_freq, filter_notch_bw, flt_amb, flt_amb_min;
  float fs_vz_speed;
  float rc_rate, rc_super_expo, rc_stick_expo;
  float rc_yaw_rate, rc_yaw_super_expo, rc_yaw_expo;
  float rc_deadband, rc_horizon_transition, rc_filter_smoothing;
  float bat_fs_v, bat_filter_alpha, lidar_filter_alpha;
  PIDConfig roll_pid, pitch_pid, yaw_pid, vz_pid;
  uint16_t vtx_mhz, vtx_mw, rtx_addr, rtx_sync_word;
  uint8_t bat_comp_en, vtx_en, vtx_prot, esc_prot, imu_align, com_align, fc_error, fm_mode;
  uint8_t flt_amb_en, flt_lpf_en, flt_not_en, fs_bat_en, fs_rtx_en, lidar_en, baro_en, com_en;
  uint8_t rtx_en, rtx_band, rtx_hw_crc_en, rtx_addr_en, rtx_iq_inv;
  uint8_t rtx_pri_chan, rtx_pri_sf, rtx_pri_bw, rtx_pri_cr;
  int8_t rtx_pri_power;
  uint8_t rtx_bak_chan, rtx_bak_sf, rtx_bak_bw, rtx_bak_cr;
  int8_t rtx_bak_power;
  uint8_t rtx_id, imu_id, bar_id, com_id, lid_id;
  uint8_t _pad[2];
};

static_assert((sizeof(FlightConfig) % 4) == 0, "FlightConfig must be 4-byte aligned");

struct Quaternion {
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  void normalize() {
    float n2 = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (n2 > 1e-12f && isfinite(n2)) {
      float inv = 1.0f / sqrtf(n2);
      if (isfinite(inv)) { q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv; return; }
    }
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  }
};

struct LowPassFilter {
  float alpha = 0.1f, output = 0.0f;
  void reset(float v = 0.0f) { output = v; }
  float update(float in) {
    if (!isfinite(in)) return output;
    float a = constrainF(alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
    output = a * in + (1.0f - a) * output;
    return isfinite(output) ? output : 0.0f;
  }
};

struct BiquadNotchFilter {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
  float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
  float last_freq = -1.0f, last_bw = -1.0f;
  void reset() { x1 = x2 = y1 = y2 = 0.0f; }
  void setParameters(float sr, float freq, float bw, bool force = false) {
    if (!force && fabsf(freq - last_freq) < 1.5f && fabsf(bw - last_bw) < 0.5f) return;
    last_freq = freq; last_bw = bw;
    if (freq <= 0.0f || bw <= 0.0f || sr <= 0.0f) { b0 = 1.0f; b1 = b2 = a1 = a2 = 0.0f; return; }
    float w = 2.0f * (float)M_PI * freq / sr;
    float sn = sinf(w), cs = cosf(w);
    float sn_s = (fabsf(sn) < 1e-6f) ? ((sn >= 0.0f) ? 1e-6f : -1e-6f) : sn;
    float al = sn * sinhf(0.34657359f * bw * w / sn_s);
    float a0 = 1.0f + al;
    b0 = 1.0f / a0; b1 = (-2.0f * cs) / a0; b2 = (1.0f - al) / a0;
    a1 = b1; a2 = (1.0f - al) / a0;
  }
  float update(float in) {
    if (b0 == 1.0f && b1 == 0.0f && a1 == 0.0f && a2 == 0.0f) return in;
    float out = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = in; y2 = y1; y1 = out;
    return isfinite(out) ? out : in;
  }
};

struct AdaptiveLidarFilter {
  float output = 0.0f, h[3] = {0.0f, 0.0f, 0.0f};
  bool init_ok = false;
  void reset() { output = h[0] = h[1] = h[2] = 0.0f; init_ok = false; }
  float update(float raw_v, float alpha) {
    if (!isfinite(raw_v)) return output;
    float a = constrainF(alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
    if (!init_ok) { output = h[0] = h[1] = h[2] = raw_v; init_ok = true; return output; }
    h[0] = h[1]; h[1] = h[2]; h[2] = raw_v;
    float med = h[1];
    if ((h[0] >= h[1] && h[0] <= h[2]) || (h[0] <= h[1] && h[0] >= h[2])) med = h[0];
    else if ((h[2] >= h[0] && h[2] <= h[1]) || (h[2] <= h[0] && h[2] >= h[1])) med = h[2];
    output = a * med + (1.0f - a) * output;
    return isfinite(output) ? output : raw_v;
  }
};

struct PID {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float integral = 0.0f, last_meas = 0.0f, last_deriv = 0.0f, int_limit = 200.0f;
  void init(const PIDConfig &cfg, float lim) { kp = cfg.kp; ki = cfg.ki; kd = cfg.kd; int_limit = lim; reset(); }
  void reset() { integral = last_meas = last_deriv = 0.0f; }
  void updateConfig(const PIDConfig &cfg) { kp = cfg.kp; ki = cfg.ki; kd = cfg.kd; }
  float update(float err, float meas, float dt, bool sat = false) {
    if (!isfinite(err) || !isfinite(meas) || !isfinite(dt)) return 0.0f;
    if (!isfinite(integral)) integral = 0.0f;
    if (!sat && int_limit > 0.0f && dt > 0.0f) integral = constrainF(integral + err * dt, -int_limit, int_limit);
    float raw_d = (dt > 0.0f) ? -(meas - last_meas) / dt : 0.0f;
    if (!isfinite(raw_d)) raw_d = 0.0f;
    last_meas = meas;
    last_deriv = PID_D_ALPHA * raw_d + (1.0f - PID_D_ALPHA) * last_deriv;
    if (!isfinite(last_deriv)) last_deriv = 0.0f;
    float out = kp * err + ki * integral + kd * last_deriv;
    return isfinite(out) ? out : 0.0f;
  }
};

// ==============================================================================
// 5. ГЛОБАЛЬНОЕ СОСТОЯНИЕ И ОПЕРЕЖАЮЩИЕ ОБЪЯВЛЕНИЯ
// ==============================================================================
FlightConfig currentConfig;
Quaternion orientation;

uint8_t fc_error_ram = FC_ERR_NONE, fc_latched_error_ram = FC_ERR_NONE, last_sent_202_err = FC_ERR_NONE;
uint32_t last_202_send_ms = 0;
bool prev_armed_state = false;

DelayedRtxRespItem rtx_resp_queue[RTX_RESP_QUEUE_SIZE];
uint8_t rtx_resp_head = 0, rtx_resp_tail = 0, rtx_resp_count = 0;

float cpu_load_pct = 0.0f, cpu_max_load_pct = 0.0f, vibration_index = 0.0f;
bool mon_stream_active = false;
uint32_t mon_last_send_ms = 0;
BenchTestMode bench_test_mode = BENCH_TEST_NONE;
uint32_t bench_test_start_ms = 0;
float prev_m1_out = 0.0f, prev_m2_out = 0.0f, prev_m3_out = 0.0f, prev_m4_out = 0.0f;

LowPassFilter lpf_ax, lpf_ay, lpf_az, lpf_gx, lpf_gy, lpf_gz;
LowPassFilter lpf_xy_hp_x{0.15f}, lpf_xy_hp_y{0.15f};
LowPassFilter lpf_temp{0.05f}, lpf_battery{0.05f}, lpf_current{0.15f};
LowPassFilter lpf_baro_alt{0.12f}, lpf_baro_vz{0.25f};
AdaptiveLidarFilter adaptive_lidar;
BiquadNotchFilter notch_gx, notch_gy, notch_gz;

float base_hover_throttle = INITIAL_BASE_HOVER, current_vz = 0.0f;
float baro_alt_m = 0.0f, baro_ground_alt_m = 0.0f, baro_vz_mps = 0.0f;
bool baro_cal_ready = false;

float v_batt = 14.8f, current_a = 0.0f, power_w = 0.0f, consumed_wh = 0.0f;
bool batt_valid = false;
uint8_t current_link_quality = 100;
float current_chip_temp = 25.0f;

bool vtx_uart_ready = false, vtx_hw_unavailable = false, vtx_initial_probe_done = false;
bool vtx_send_freq_req = false, vtx_send_power_req = false, vtx_link_ok = false;
uint8_t vtx_detected_proto = VTX_PROTO_NONE;
uint16_t vtx_actual_mw = 0;
int8_t vtx_actual_dbm = 0;
uint32_t vtx_last_tx_ms = 0, vtx_last_probe_ms = 0, vtx_baud = 0, vtx_step_start_ms = 0;
VtxProbeStep vtx_probe_step = VTX_STEP_IDLE;
uint8_t tramp_rx_buf[16], tramp_rx_idx = 0, sa_rx_buf[16], sa_rx_idx = 0;

uint8_t rtx_active_chip_id = RTX_CHIP_SX1262;
bool rtx_recovering = false;
bool mag_cal_active = false;
uint32_t mag_cal_start_ms = 0, mag_cal_last_sample_ms = 0;
float mag_cal_min_x = 1e9f, mag_cal_max_x = -1e9f, mag_cal_min_y = 1e9f, mag_cal_max_y = -1e9f, mag_cal_min_z = 1e9f, mag_cal_max_z = -1e9f;
float compass_heading_deg = 0.0f;
bool compass_valid = false;

uint8_t i2c_error_counter = 0, rtx_error_counter = 0;
uint8_t rtx_noise_floor[100], rtx_scan_cursor = 0, rtx_best_clean_chan = 15;
bool rtx_scan_active = false, rtx_switched_flag = false;
uint8_t rtx_new_switched_chan = 15, main_rtx_chan = 15, active_rtx_chan = 15;
bool is_on_backup = false, pending_channel_switch = false;
uint8_t target_pending_chan = 15;

bool rtx_tx_in_flight = false, rtx_apply_switch_after_tx = false, rtx_post_tx_backup = false;
uint32_t rtx_tx_start_ms = 0, rtx_tx_timeout_ms = RTX_TX_TIMEOUT_PRI_MS;
uint8_t rtx_post_tx_chan = 15;

PID vzPID, rollRatePID, pitchRatePID, yawRatePID;
bool is_armed = false, is_failsafe = false;
float arm_ramp_factor = 0.0f;
float smooth_throttle = 0.0f, smooth_roll = STICK_CENTER, smooth_pitch = STICK_CENTER, smooth_yaw = STICK_CENTER;
uint8_t mpu_dma_buffer[14];
uint32_t last_rc_packet_ms = 0;
bool send_telemetry_pending = false;
bool err_rc_loss = false, err_batt_low = false, err_imu_fail = false, err_aux_i2c_fail = false;
float dbg_lidar_m = 0.0f;
uint32_t last_lidar_packet_ms = 0;

void loadSettings();
void saveSettings();
void sanitizeConfig();
void resetToFactoryDefaults();
void applyFilterConfig();
void updatePidTunings();
void applyRtxConfiguration(bool use_backup_settings);
void rtx_set_rx_mode();
void startChannelScanNonBlocking();
void stepChannelScanNonBlocking();
void startTelemetryNonBlocking();
void pollTelemetryTxState();
void queueDelayedRtxResponse(DelayedRtxRespType type, uint16_t vtx_mhz, uint8_t tier, uint16_t vtx_mw);
bool hasQueuedRtxResponse();
bool popDelayedRtxResponse(DelayedRtxRespItem &out);
void setupHardwareTimers();
void setMotorsPower(float m1, float m2, float m3, float m4, float max_cap = OUTPUT_MAX);
void resetMotorSlewState();
void flightControlTask();
void parseCli();
float calculateSuperExpoRate(float in, float r, float s, float e);
void setFcErrorRam(uint8_t err_code);
void vtxInit();
void vtxSetFrequencyMhz(uint16_t mhz, bool send_now, bool save_eeprom);
void vtxSetPowerMw(uint16_t mw, bool send_now, bool save_eeprom);
void vtxSetFrequencyAndPower(uint16_t mhz, uint16_t power_mw, bool save_eeprom);
void vtxStepNonBlocking(uint32_t now_ms);
void stopBenchTest();

namespace CliAutoCal {
  void init();
  void pollButton(uint32_t now_ms);
  void start();
  void stop(uint8_t reason_code, uint8_t fc_err_code);
  bool isFlightActive();
  void onDisarmRequested();
  void onArmRequested();
  void onAttitudeAndMotion(float ax_g, float ay_g, float az_g, float roll_deg, float pitch_deg, float gx_dps, float gy_dps);
  bool stepFlight(uint32_t now_ms, float gx_dps, float gy_dps, float gz_dps,
                  float gx_rad, float gy_rad, float gz_rad,
                  float roll_deg, float pitch_deg,
                  float &m1, float &m2, float &m3, float &m4);
}

// ==============================================================================
// 6. FLASH STORAGE (СТРАНИЦА 127: 0x0801FC00) И КОНФИГУРАЦИЯ
// ==============================================================================
static void flashUnlock() {
  if (Reg::FLASH->CR & (1U << 7)) {
    Reg::FLASH->KEYR = 0x45670123UL;
    Reg::FLASH->KEYR = 0xCDEF89ABUL;
  }
}

static void flashLock() {
  Reg::FLASH->CR |= (1U << 7);
}

static void flashErasePage(uint32_t page_addr) {
  while (Reg::FLASH->SR & (1U << 0)) {}
  Reg::FLASH->CR |= (1U << 1);
  Reg::FLASH->AR = page_addr;
  Reg::FLASH->CR |= (1U << 6);
  while (Reg::FLASH->SR & (1U << 0)) {}
  Reg::FLASH->CR &= ~(1U << 1);
  Reg::FLASH->SR = (1U << 5) | (1U << 2);
}

static void flashWriteHalfWord(uint32_t addr, uint16_t data) {
  while (Reg::FLASH->SR & (1U << 0)) {}
  Reg::FLASH->CR |= (1U << 0);
  *reinterpret_cast<volatile uint16_t*>(addr) = data;
  while (Reg::FLASH->SR & (1U << 0)) {}
  Reg::FLASH->CR &= ~(1U << 0);
  Reg::FLASH->SR = (1U << 5) | (1U << 2);
}

static inline float roundToTenthVolt(float v) { return (float)lroundf(v * 10.0f) * 0.1f; }
static inline float getEffectiveEscDeltaLimit() {
  return constrainF(currentConfig.flt_esc_limit, CFG_ESC_LIMIT_MIN, CFG_ESC_LIMIT_MAX);
}
static inline float getEffectiveBatVMax() {
  return roundToTenthVolt(constrainF(currentConfig.bat_v_max, CFG_BAT_V_MAX_MIN, CFG_BAT_V_MAX_MAX));
}

void sanitizeConfig() {
  currentConfig.fm_mode = (uint8_t)constrainI(currentConfig.fm_mode, 0, 3);
  currentConfig.fm_roll_max = constrainF(currentConfig.fm_roll_max, 1.0f, CFG_ANGLE_MAX_DEG);
  currentConfig.fm_pitch_max = constrainF(currentConfig.fm_pitch_max, 1.0f, CFG_ANGLE_MAX_DEG);
  currentConfig.fm_vz_speed_max = constrainF(currentConfig.fm_vz_speed_max, 0.1f, CFG_VZ_MAX_MPS);

  currentConfig.rc_rate = constrainF(currentConfig.rc_rate, 0.01f, CFG_RATE_MAX);
  currentConfig.rc_super_expo = constrainF(currentConfig.rc_super_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_stick_expo = constrainF(currentConfig.rc_stick_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_yaw_rate = constrainF(currentConfig.rc_yaw_rate, 0.01f, CFG_RATE_MAX);
  currentConfig.rc_yaw_super_expo = constrainF(currentConfig.rc_yaw_super_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_yaw_expo = constrainF(currentConfig.rc_yaw_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_deadband = constrainF(currentConfig.rc_deadband, 0.0f, CFG_DEADBAND_MAX);
  currentConfig.rc_horizon_transition = constrainF(currentConfig.rc_horizon_transition, 0.05f, 1.0f);
  currentConfig.rc_filter_smoothing = constrainF(currentConfig.rc_filter_smoothing, CFG_ALPHA_MIN, CFG_ALPHA_MAX);

  currentConfig.esc_prot = (uint8_t)constrainI(currentConfig.esc_prot, 0, 1);
  currentConfig.flt_esc_limit = getEffectiveEscDeltaLimit();
  currentConfig.flt_lpf_en = (uint8_t)constrainI(currentConfig.flt_lpf_en, 0, 1);
  currentConfig.flt_not_en = (uint8_t)constrainI(currentConfig.flt_not_en, 0, 1);
  currentConfig.flt_amb_en = (uint8_t)constrainI(currentConfig.flt_amb_en, 0, 1);
  currentConfig.gyro_filter_alpha = constrainF(currentConfig.gyro_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.acc_filter_alpha = constrainF(currentConfig.acc_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.filter_notch_freq = constrainF(currentConfig.filter_notch_freq, CFG_NOTCH_FREQ_MIN, CFG_NOTCH_FREQ_MAX);
  currentConfig.filter_notch_bw = constrainF(currentConfig.filter_notch_bw, CFG_NOTCH_BW_MIN, CFG_NOTCH_BW_MAX);
  currentConfig.flt_amb = constrainF(currentConfig.flt_amb, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.flt_amb_min = constrainF(currentConfig.flt_amb_min, 0.0005f, currentConfig.flt_amb);

  currentConfig.imu_id = (uint8_t)constrainI(currentConfig.imu_id, 0, 6);
  currentConfig.imu_align = (uint8_t)constrainI(currentConfig.imu_align, 0, 3);
  currentConfig.baro_en = (uint8_t)constrainI(currentConfig.baro_en, 0, 1);
  currentConfig.bar_id = (uint8_t)constrainI(currentConfig.bar_id, 0, 7);
  currentConfig.com_en = (uint8_t)constrainI(currentConfig.com_en, 0, 1);
  currentConfig.com_id = (uint8_t)constrainI(currentConfig.com_id, 0, 6);
  currentConfig.com_align = (uint8_t)constrainI(currentConfig.com_align, 0, 3);
  currentConfig.mag_bias_x = constrainF(currentConfig.mag_bias_x, CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
  currentConfig.mag_bias_y = constrainF(currentConfig.mag_bias_y, CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
  currentConfig.mag_bias_z = constrainF(currentConfig.mag_bias_z, CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
  currentConfig.lidar_en = (uint8_t)constrainI(currentConfig.lidar_en, 0, 1);
  currentConfig.lid_id = (uint8_t)constrainI(currentConfig.lid_id, 0, 4);
  currentConfig.lidar_filter_alpha = constrainF(currentConfig.lidar_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.lidar_offset_m = constrainF(currentConfig.lidar_offset_m, CFG_LIDAR_OFFSET_MIN, CFG_LIDAR_OFFSET_MAX);

  currentConfig.bat_scale_v = constrainF(currentConfig.bat_scale_v, CFG_BAT_SCALE_V_MIN, CFG_BAT_SCALE_V_MAX);
  currentConfig.bat_v_max = getEffectiveBatVMax();
  currentConfig.bat_comp_en = (uint8_t)constrainI(currentConfig.bat_comp_en, 0, 1);
  currentConfig.bat_filter_alpha = constrainF(currentConfig.bat_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.bat_offset_a = constrainF(currentConfig.bat_offset_a, CFG_BAT_OFFSET_A_MIN, CFG_BAT_OFFSET_A_MAX);
  currentConfig.bat_scale_a = constrainF(currentConfig.bat_scale_a, CFG_BAT_SCALE_A_MIN, CFG_BAT_SCALE_A_MAX);
  currentConfig.bat_fs_v = constrainF(currentConfig.bat_fs_v, CFG_BAT_FS_V_MIN, CFG_BAT_FS_V_MAX);
  currentConfig.fs_bat_en = (uint8_t)constrainI(currentConfig.fs_bat_en, 0, 1);
  currentConfig.fs_rtx_en = (uint8_t)constrainI(currentConfig.fs_rtx_en, 0, 1);
  currentConfig.fs_vz_speed = constrainF(currentConfig.fs_vz_speed, 0.05f, CFG_VZ_MAX_MPS);

  currentConfig.roll_pid.kp = constrainF(currentConfig.roll_pid.kp, 0.0f, CFG_RATE_KP_MAX);
  currentConfig.roll_pid.ki = constrainF(currentConfig.roll_pid.ki, 0.0f, CFG_RATE_KI_MAX);
  currentConfig.roll_pid.kd = constrainF(currentConfig.roll_pid.kd, 0.0f, CFG_RATE_KD_MAX);
  currentConfig.pitch_pid.kp = constrainF(currentConfig.pitch_pid.kp, 0.0f, CFG_RATE_KP_MAX);
  currentConfig.pitch_pid.ki = constrainF(currentConfig.pitch_pid.ki, 0.0f, CFG_RATE_KI_MAX);
  currentConfig.pitch_pid.kd = constrainF(currentConfig.pitch_pid.kd, 0.0f, CFG_RATE_KD_MAX);
  currentConfig.yaw_pid.kp = constrainF(currentConfig.yaw_pid.kp, 0.0f, CFG_RATE_KP_MAX);
  currentConfig.yaw_pid.ki = constrainF(currentConfig.yaw_pid.ki, 0.0f, CFG_RATE_KI_MAX);
  currentConfig.yaw_pid.kd = constrainF(currentConfig.yaw_pid.kd, 0.0f, CFG_RATE_KD_MAX);
  currentConfig.vz_pid.kp = constrainF(currentConfig.vz_pid.kp, 0.0f, CFG_VZ_KP_MAX);
  currentConfig.vz_pid.ki = constrainF(currentConfig.vz_pid.ki, 0.0f, CFG_VZ_KI_MAX);
  currentConfig.vz_pid.kd = constrainF(currentConfig.vz_pid.kd, 0.0f, CFG_VZ_KD_MAX);

  currentConfig.vtx_en = (uint8_t)constrainI(currentConfig.vtx_en, 0, 1);
  currentConfig.vtx_prot = (uint8_t)constrainI(currentConfig.vtx_prot, 0, 2);
  currentConfig.vtx_mhz = (uint16_t)constrainI(currentConfig.vtx_mhz, 5000, 6000);
  currentConfig.vtx_mw = (currentConfig.vtx_mw == 0) ? 25U : (uint16_t)constrainI(currentConfig.vtx_mw, 1, 10000);

  currentConfig.rtx_en = (uint8_t)constrainI(currentConfig.rtx_en, 0, 1);
  currentConfig.rtx_id = (uint8_t)constrainI(currentConfig.rtx_id, 0, 3);
  currentConfig.rtx_band = (uint8_t)constrainI(currentConfig.rtx_band, 1, 9);
  currentConfig.rtx_hw_crc_en = (uint8_t)constrainI(currentConfig.rtx_hw_crc_en, 0, 1);
  currentConfig.rtx_addr_en = (uint8_t)constrainI(currentConfig.rtx_addr_en, 0, 1);
  currentConfig.rtx_iq_inv = (uint8_t)constrainI(currentConfig.rtx_iq_inv, 0, 1);
  currentConfig.rtx_pri_chan = (uint8_t)constrainI(currentConfig.rtx_pri_chan, 0, 99);
  currentConfig.rtx_pri_sf = (uint8_t)constrainI(currentConfig.rtx_pri_sf, 5, 12);
  currentConfig.rtx_pri_bw = (uint8_t)constrainI(currentConfig.rtx_pri_bw, 0, 9);
  currentConfig.rtx_pri_cr = (uint8_t)constrainI(currentConfig.rtx_pri_cr, 1, 4);
  currentConfig.rtx_pri_power = (int8_t)constrainI(currentConfig.rtx_pri_power, -9, 22);
  currentConfig.rtx_bak_chan = (uint8_t)constrainI(currentConfig.rtx_bak_chan, 0, 99);
  currentConfig.rtx_bak_sf = (uint8_t)constrainI(currentConfig.rtx_bak_sf, 5, 12);
  currentConfig.rtx_bak_bw = (uint8_t)constrainI(currentConfig.rtx_bak_bw, 0, 9);
  currentConfig.rtx_bak_cr = (uint8_t)constrainI(currentConfig.rtx_bak_cr, 1, 4);
  currentConfig.rtx_bak_power = (int8_t)constrainI(currentConfig.rtx_bak_power, -9, 22);
}

static inline float getBatteryCompensationFactor() {
  if (!currentConfig.bat_comp_en || !batt_valid || v_batt <= 2.0f || !isfinite(v_batt)) return 1.0f;
  return constrainF(getEffectiveBatVMax() / v_batt, 0.95f, 1.35f);
}
static inline bool isCriticalErrorCode(uint8_t e) {
  return e == FC_ERR_IMU_FAIL || e == FC_ERR_RC_LOSS || e == FC_ERR_BAT_LOW || e == FC_ERR_TILT_DISARM || e == FC_ERR_NAN_INF_MATH;
}
static inline bool hasBlockingArmError(uint8_t e) {
  return e == FC_ERR_IMU_FAIL || e == FC_ERR_CALIB_MISSING || e == FC_ERR_NAN_INF_MATH;
}

void setFcErrorRam(uint8_t err_code) {
  if (err_code == FC_ERR_NONE) return;
  fc_error_ram = err_code;
  fc_latched_error_ram = err_code;
  if (err_code != last_sent_202_err) {
    uint8_t flags = (isCriticalErrorCode(err_code) ? 0x01 : 0x00) | (is_armed ? 0x02 : 0x00);
    queueDelayedRtxResponse(RTX_RESP_202_ERR_EVENT, err_code, flags, 0);
    last_sent_202_err = err_code;
    last_202_send_ms = millis();
  }
}

void serviceError202Event(uint32_t now_ms) {
  if (fc_error_ram == FC_ERR_NONE) { last_sent_202_err = FC_ERR_NONE; return; }
  if (isCriticalErrorCode(fc_error_ram) && (now_ms - last_202_send_ms) >= ERR_202_REPEAT_MS) {
    queueDelayedRtxResponse(RTX_RESP_202_ERR_EVENT, fc_error_ram, (uint8_t)(0x01 | (is_armed ? 0x02 : 0x00)), 0);
    last_sent_202_err = fc_error_ram;
    last_202_send_ms = now_ms;
  }
}

void handleMotorArmStateAndEepromError() {
  if (!prev_armed_state && is_armed) {
    stopBenchTest();
    CliAutoCal::onArmRequested();
    if (hasBlockingArmError(fc_error_ram) || hasBlockingArmError(fc_latched_error_ram)) {
      is_armed = false; resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
      prev_armed_state = false; return;
    }
    fc_error_ram = fc_latched_error_ram = last_sent_202_err = FC_ERR_NONE;
    consumed_wh = 0.0f;
    resetMotorSlewState();
  } else if (prev_armed_state && !is_armed) {
    resetMotorSlewState();
    if (fc_latched_error_ram != FC_ERR_NONE) {
      currentConfig.fc_error = fc_latched_error_ram;
      saveSettings();
    }
  }
  prev_armed_state = is_armed;
}

void applyFilterConfig() {
  sanitizeConfig();
  lpf_gx.alpha = lpf_gy.alpha = lpf_gz.alpha = currentConfig.gyro_filter_alpha;
  lpf_ax.alpha = lpf_ay.alpha = lpf_az.alpha = currentConfig.acc_filter_alpha;
  if (!currentConfig.flt_lpf_en) {
    lpf_gx.reset(); lpf_gy.reset(); lpf_gz.reset();
    lpf_ax.reset(); lpf_ay.reset(); lpf_az.reset();
  }
  lpf_battery.alpha = currentConfig.bat_filter_alpha;
  notch_gx.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  notch_gy.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  notch_gz.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  if (!currentConfig.flt_not_en) { notch_gx.reset(); notch_gy.reset(); notch_gz.reset(); }
}

void updatePidTunings() {
  sanitizeConfig();
  rollRatePID.updateConfig(currentConfig.roll_pid);
  pitchRatePID.updateConfig(currentConfig.pitch_pid);
  yawRatePID.updateConfig(currentConfig.yaw_pid);
  vzPID.updateConfig(currentConfig.vz_pid);
}

void resetToFactoryDefaults() {
  memset(&currentConfig, 0, sizeof(currentConfig));
  currentConfig.magic_marker = EEPROM_MAGIC_MARKER;
  currentConfig.vtx_en = 1; currentConfig.vtx_prot = VTX_PROT_AUTO;
  currentConfig.vtx_mhz = 5658; currentConfig.vtx_mw = 25;
  currentConfig.base_temperature = 25.0f;
  currentConfig.temp_coeff_x = currentConfig.temp_coeff_y = currentConfig.temp_coeff_z = 0.0005f;
  currentConfig.bat_scale_v = 4.85f; currentConfig.bat_scale_a = 1.0f; currentConfig.bat_comp_en = 1;
  currentConfig.bat_v_max = 16.8f; currentConfig.flt_esc_limit = 45.0f;
  currentConfig.fm_mode = 1; currentConfig.fm_roll_max = 25.0f; currentConfig.fm_pitch_max = 25.0f; currentConfig.fm_vz_speed_max = 1.2f;
  currentConfig.rc_rate = 0.35f; currentConfig.rc_super_expo = 0.03f; currentConfig.rc_stick_expo = 0.45f;
  currentConfig.rc_yaw_rate = 0.30f; currentConfig.rc_yaw_super_expo = 0.03f; currentConfig.rc_yaw_expo = 0.35f;
  currentConfig.rc_deadband = 15.0f; currentConfig.rc_horizon_transition = 0.75f; currentConfig.rc_filter_smoothing = 0.35f;
  currentConfig.fs_bat_en = 1; currentConfig.fs_rtx_en = 1; currentConfig.fs_vz_speed = 0.5f;
  currentConfig.bat_fs_v = 13.8f; currentConfig.bat_filter_alpha = 0.05f;
  currentConfig.lidar_en = 1; currentConfig.lidar_filter_alpha = 0.15f;
  currentConfig.baro_en = 1; currentConfig.com_en = 0;
  currentConfig.flt_amb_en = 1; currentConfig.flt_lpf_en = 1; currentConfig.flt_not_en = 1;
  currentConfig.gyro_filter_alpha = 0.10f; currentConfig.acc_filter_alpha = 0.10f;
  currentConfig.filter_notch_freq = 120.0f; currentConfig.filter_notch_bw = 20.0f;
  currentConfig.flt_amb = DEFAULT_FLT_AMB; currentConfig.flt_amb_min = DEFAULT_FLT_AMB_MIN;
  currentConfig.rtx_en = 1; currentConfig.rtx_band = 9; currentConfig.rtx_hw_crc_en = 1;
  currentConfig.rtx_addr_en = 1; currentConfig.rtx_sync_word = 0x1424;
  currentConfig.rtx_pri_chan = 15; currentConfig.rtx_pri_sf = 7; currentConfig.rtx_pri_bw = 7; currentConfig.rtx_pri_cr = 1; currentConfig.rtx_pri_power = 22;
  currentConfig.rtx_bak_chan = 25; currentConfig.rtx_bak_sf = 9; currentConfig.rtx_bak_bw = 7; currentConfig.rtx_bak_cr = 1; currentConfig.rtx_bak_power = 22;
  currentConfig.roll_pid = {0.45f, 0.010f, 0.018f};
  currentConfig.pitch_pid = {0.45f, 0.010f, 0.018f};
  currentConfig.yaw_pid = {0.60f, 0.010f, 0.0f};
  currentConfig.vz_pid = {7.0f, 1.2f, 0.30f};

  sanitizeConfig();
  vtx_link_ok = false; vtx_detected_proto = VTX_PROTO_NONE;
  main_rtx_chan = active_rtx_chan = currentConfig.rtx_pri_chan;
  is_on_backup = false;

  rollRatePID.init(currentConfig.roll_pid, 200.0f);
  pitchRatePID.init(currentConfig.pitch_pid, 200.0f);
  yawRatePID.init(currentConfig.yaw_pid, 200.0f);
  vzPID.init(currentConfig.vz_pid, 300.0f);
  adaptive_lidar.reset();
  resetMotorSlewState();
  applyFilterConfig();
  saveSettings();
}

void loadSettings() {
  memcpy(&currentConfig, reinterpret_cast<const void*>(FLASH_CFG_ADDR), sizeof(FlightConfig));
  if (currentConfig.magic_marker != EEPROM_MAGIC_MARKER) {
    resetToFactoryDefaults();
    setFcErrorRam(FC_ERR_EEPROM_RESET);
    return;
  }
  sanitizeConfig();
  rollRatePID.init(currentConfig.roll_pid, 200.0f);
  pitchRatePID.init(currentConfig.pitch_pid, 200.0f);
  yawRatePID.init(currentConfig.yaw_pid, 200.0f);
  vzPID.init(currentConfig.vz_pid, 300.0f);
  if (currentConfig.fc_error != FC_ERR_NONE) fc_latched_error_ram = currentConfig.fc_error;
}

void saveSettings() {
  if (is_armed || CliAutoCal::isFlightActive()) return;
  sanitizeConfig();
  currentConfig.magic_marker = EEPROM_MAGIC_MARKER;

  const uint8_t* src = reinterpret_cast<const uint8_t*>(&currentConfig);
  flashUnlock();
  flashErasePage(FLASH_CFG_ADDR);
  for (size_t i = 0; i < sizeof(FlightConfig); i += 2) {
    uint16_t hw = (uint16_t)src[i] | ((uint16_t)src[i + 1] << 8);
    flashWriteHalfWord(FLASH_CFG_ADDR + i, hw);
  }
  flashLock();
}

// ==============================================================================
// 7. РАДИОКАНАЛ RTX (SPI2)
// ==============================================================================
inline bool hasQueuedRtxResponse() { return (rtx_resp_count > 0) && (rtx_resp_queue[rtx_resp_head].type != RTX_RESP_NONE); }

void queueDelayedRtxResponse(DelayedRtxRespType type, uint16_t vtx_mhz, uint8_t tier, uint16_t vtx_mw) {
  if (type == RTX_RESP_NONE) return;
  if (rtx_resp_count >= RTX_RESP_QUEUE_SIZE) {
    if (type != RTX_RESP_202_ERR_EVENT) return;
    rtx_resp_head = (rtx_resp_head + 1) % RTX_RESP_QUEUE_SIZE;
    rtx_resp_count--;
  }
  rtx_resp_queue[rtx_resp_tail] = {type, vtx_mhz, tier, vtx_mw};
  rtx_resp_tail = (rtx_resp_tail + 1) % RTX_RESP_QUEUE_SIZE;
  rtx_resp_count++;
}

bool popDelayedRtxResponse(DelayedRtxRespItem &out) {
  if (rtx_resp_count == 0) { out.type = RTX_RESP_NONE; return false; }
  out = rtx_resp_queue[rtx_resp_head];
  rtx_resp_queue[rtx_resp_head].type = RTX_RESP_NONE;
  rtx_resp_head = (rtx_resp_head + 1) % RTX_RESP_QUEUE_SIZE;
  rtx_resp_count--;
  return true;
}

uint32_t getTelemetryTxTimeoutMs(bool backup_mode) {
  uint8_t sf = backup_mode ? (uint8_t)constrainI(currentConfig.rtx_bak_sf, 5, 12) : (uint8_t)constrainI(currentConfig.rtx_pri_sf, 5, 12);
  uint32_t t = backup_mode ? RTX_TX_TIMEOUT_BAK_MS : RTX_TX_TIMEOUT_PRI_MS;
  if (sf >= 11) t += 140; else if (sf >= 10) t += 70;
  return t;
}

void checkRtxBusAndRecover() {
  if (rtx_recovering || rtx_error_counter < RTX_MAX_ERRORS) return;
  rtx_recovering = true;
  setFcErrorRam(FC_ERR_SPI_RTX);
  gpioWrite(Reg::GPIOB, 0, false);
  uint32_t s = micros(); while ((micros() - s) < 2000) {}
  gpioWrite(Reg::GPIOB, 0, true);
  rtx_tx_in_flight = rtx_apply_switch_after_tx = rtx_scan_active = false;
  rtx_error_counter = 0;
  applyRtxConfiguration(is_on_backup);
  rtx_recovering = false;
}

inline bool rtx_try_ready_short(uint32_t timeout_us = RTX_BUSY_SHORT_TIMEOUT_US) {
  uint32_t s = micros();
  while (gpioRead(Reg::GPIOB, 1)) {
    if ((micros() - s) >= timeout_us) {
      rtx_error_counter++; setFcErrorRam(FC_ERR_SPI_RTX); checkRtxBusAndRecover(); return false;
    }
  }
  return true;
}

void rtx_write_command(uint8_t cmd, const uint8_t *data, uint8_t len) {
  if (!rtx_try_ready_short()) return;
  spi2BeginTx(); spi2Transfer(cmd);
  for (uint8_t i = 0; i < len; i++) spi2Transfer(data[i]);
  spi2EndTx(); rtx_error_counter = 0;
}

void rtx_read_command(uint8_t cmd, uint8_t *data, uint8_t len) {
  if (!rtx_try_ready_short()) return;
  spi2BeginTx(); spi2Transfer(cmd); spi2Transfer(0x00);
  for (uint8_t i = 0; i < len; i++) data[i] = spi2Transfer(0x00);
  spi2EndTx(); rtx_error_counter = 0;
}

static uint8_t detectOrSelectRtxChip() {
  uint8_t req = currentConfig.rtx_id;
  if (req == RTX_CHIP_SX1262 || req == RTX_CHIP_SX1268 || req == RTX_CHIP_LLCC68) return req;
  uint8_t st[2] = {0, 0}; rtx_read_command(0xC0, st, 2);
  return (currentConfig.rtx_band <= 5) ? RTX_CHIP_SX1268 : RTX_CHIP_SX1262;
}

void rtx_init_hardware() {
  initSpi2();
  gpioWrite(Reg::GPIOB, 0, false); delay(5); gpioWrite(Reg::GPIOB, 0, true); delay(10);
  rtx_tx_in_flight = rtx_apply_switch_after_tx = rtx_scan_active = false;
  rtx_active_chip_id = detectOrSelectRtxChip();
}

static uint32_t rtxFreqToReg(uint32_t freq_hz) {
  return (uint32_t)(((uint64_t)freq_hz << 25) / 32000000UL);
}

void applyRtxConfiguration(bool use_backup_settings) {
  if (!currentConfig.rtx_en || !rtx_try_ready_short()) return;
  rtx_tx_in_flight = rtx_apply_switch_after_tx = rtx_scan_active = false;
  rtx_active_chip_id = detectOrSelectRtxChip();

  uint8_t stby = 0x00; rtx_write_command(0x80, &stby, 1);
  uint8_t reg_mode = 0x01; rtx_write_command(0x96, &reg_mode, 1);
  uint8_t pkt_type = 0x01; rtx_write_command(0x8A, &pkt_type, 1);

  if (currentConfig.rtx_addr_en) {
    uint8_t ab[2] = {(uint8_t)(currentConfig.rtx_addr >> 8), (uint8_t)(currentConfig.rtx_addr & 0xFF)};
    rtx_write_command(0x6C, ab, 2);
  }
  uint8_t sb[2] = {(uint8_t)(currentConfig.rtx_sync_word >> 8), (uint8_t)(currentConfig.rtx_sync_word & 0xFF)};
  rtx_write_command(0x74, sb, 2);

  uint8_t band = (uint8_t)constrainI(currentConfig.rtx_band, 1, 9);
  uint8_t chan = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_chan, 0, 99) : (uint8_t)constrainI(active_rtx_chan, 0, 99);
  uint32_t freq_reg = rtxFreqToReg((uint32_t)band * 100000000UL + (uint32_t)chan * 1000000UL);
  uint8_t fb[4] = {(uint8_t)(freq_reg >> 24), (uint8_t)(freq_reg >> 16), (uint8_t)(freq_reg >> 8), (uint8_t)freq_reg};
  rtx_write_command(0x86, fb, 4);

  uint8_t sf = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_sf, 5, 12) : (uint8_t)constrainI(currentConfig.rtx_pri_sf, 5, 12);
  uint8_t bw = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_bw, 0, 9) : (uint8_t)constrainI(currentConfig.rtx_pri_bw, 0, 9);
  if (rtx_active_chip_id == RTX_CHIP_LLCC68) sf = (uint8_t)constrainI(sf, 5, 11);
  uint8_t cr = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_cr, 1, 4) : (uint8_t)constrainI(currentConfig.rtx_pri_cr, 1, 4);
  uint8_t mp[4] = {sf, bw, cr, 0x00};
  rtx_write_command(0x8B, mp, 4);

  uint8_t pp[6] = {0x00, 0x08, 0x00, RTX_RX_MAX_PAYLOAD_BYTES, (uint8_t)(currentConfig.rtx_hw_crc_en ? 1 : 0), (uint8_t)(currentConfig.rtx_iq_inv ? 1 : 0)};
  rtx_write_command(0x8C, pp, 6);

  int8_t pwr = use_backup_settings ? (int8_t)constrainI(currentConfig.rtx_bak_power, -9, 22) : (int8_t)constrainI(currentConfig.rtx_pri_power, -9, 22);
  uint8_t pa[4] = {0x04, 0x07, 0x00, 0x01}; rtx_write_command(0x95, pa, 4);
  uint8_t tp[2] = {(uint8_t)pwr, 0x02}; rtx_write_command(0x8E, tp, 2);

  rtx_set_rx_mode();
}

void rtx_set_rx_mode() {
  if (!currentConfig.rtx_en) return;
  uint8_t p[3] = {0xFF, 0xFF, 0xFF};
  rtx_write_command(0x82, p, 3);
}

void startChannelScanNonBlocking() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_recovering) return;
  rtx_scan_active = true; rtx_scan_cursor = 0;
}

void stepChannelScanNonBlocking() {
  if (!rtx_scan_active || !currentConfig.rtx_en || rtx_tx_in_flight || rtx_recovering) return;
  if (gpioRead(Reg::GPIOB, 1)) return;

  uint32_t base_reg = rtxFreqToReg((uint32_t)constrainI(currentConfig.rtx_band, 1, 9) * 100000000UL);
  uint32_t step_reg = rtxFreqToReg(1000000UL);
  uint8_t end_chan = (uint8_t)(rtx_scan_cursor + RTX_SCAN_CHANNELS_PER_TICK);
  if (end_chan > 100) end_chan = 100;
  uint32_t freq_reg = base_reg + ((uint32_t)rtx_scan_cursor * step_reg);

  for (uint8_t c = rtx_scan_cursor; c < end_chan; c++) {
    uint32_t w = micros();
    while (gpioRead(Reg::GPIOB, 1)) {
      if ((micros() - w) > RTX_SCAN_BUSY_TIMEOUT_US) { setFcErrorRam(FC_ERR_SPI_RTX); rtx_scan_active = false; applyRtxConfiguration(is_on_backup); return; }
    }
    uint8_t fb[4] = {(uint8_t)(freq_reg >> 24), (uint8_t)(freq_reg >> 16), (uint8_t)(freq_reg >> 8), (uint8_t)freq_reg};
    spi2BeginTx(); spi2Transfer(0x86); for (uint8_t i = 0; i < 4; i++) spi2Transfer(fb[i]); spi2EndTx();

    w = micros();
    while (gpioRead(Reg::GPIOB, 1)) {
      if ((micros() - w) > RTX_SCAN_BUSY_TIMEOUT_US) { setFcErrorRam(FC_ERR_SPI_RTX); rtx_scan_active = false; applyRtxConfiguration(is_on_backup); return; }
    }
    spi2BeginTx(); spi2Transfer(0x15); spi2Transfer(0x00); rtx_noise_floor[c] = spi2Transfer(0x00); spi2EndTx();
    freq_reg += step_reg;
  }

  rtx_scan_cursor = end_chan;
  if (rtx_scan_cursor >= 100) {
    uint8_t best = 0, min_n = 255;
    for (uint8_t i = 0; i < 100; i++) {
      if (rtx_noise_floor[i] < min_n) { min_n = rtx_noise_floor[i]; best = i; }
    }
    rtx_best_clean_chan = best; rtx_scan_active = false;
    queueDelayedRtxResponse(RTX_RESP_230_SCAN_BEST, rtx_best_clean_chan, 0, 0);

    uint32_t wr = base_reg + ((uint32_t)constrainI(active_rtx_chan, 0, 99) * step_reg);
    uint8_t wb[4] = {(uint8_t)(wr >> 24), (uint8_t)(wr >> 16), (uint8_t)(wr >> 8), (uint8_t)wr};
    rtx_write_command(0x86, wb, 4);
    rtx_set_rx_mode();
  }
}

void readRx() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_scan_active || rtx_recovering) return;
  if (gpioRead(Reg::GPIOB, 1)) return;

  uint8_t irq_st[2] = {0, 0}; rtx_read_command(0x12, irq_st, 2);
  uint16_t irq = ((uint16_t)irq_st[0] << 8) | irq_st[1];
  if (irq & 0x02) {
    uint8_t clr[2] = {0x00, 0x02}; rtx_write_command(0x02, clr, 2);
    if (!rtx_try_ready_short()) { rtx_set_rx_mode(); return; }

    uint8_t bs[2] = {0, 0}; rtx_read_command(0x13, bs, 2);
    uint8_t plen = bs[0], sptr = bs[1];
    if (plen == 0 || plen > RTX_RX_MAX_PAYLOAD_BYTES) { rtx_set_rx_mode(); return; }

    spi2BeginTx(); spi2Transfer(0x1E); spi2Transfer(sptr);
    uint8_t raw[RTX_RX_MAX_PAYLOAD_BYTES] = {0};
    for (uint8_t i = 0; i < plen; i++) raw[i] = spi2Transfer(0x00);
    spi2EndTx();

    uint8_t idx = 0;
    if (currentConfig.rtx_addr_en) {
      if (plen < (idx + 2)) { rtx_set_rx_mode(); return; }
      uint16_t addr = ((uint16_t)raw[idx] << 8) | raw[idx + 1]; idx += 2;
      if (addr != currentConfig.rtx_addr) { rtx_set_rx_mode(); return; }
    }
    if (plen < (uint8_t)(idx + 2)) { rtx_set_rx_mode(); return; }

    uint8_t cmd = raw[idx++], lq = raw[idx++], rem = (uint8_t)(plen - idx);
    if (cmd == 110 && rem >= 4) {
      current_link_quality = lq;
      float t_v = (float)raw[idx++] * 10.0f, r_v = (float)raw[idx++] * 10.0f;
      float p_v = (float)raw[idx++] * 10.0f, y_v = (float)raw[idx++] * 10.0f;
      float a = constrainF(currentConfig.rc_filter_smoothing, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
      smooth_throttle = a * t_v + (1.0f - a) * smooth_throttle;
      smooth_roll = a * r_v + (1.0f - a) * smooth_roll;
      smooth_pitch = a * p_v + (1.0f - a) * smooth_pitch;
      smooth_yaw = a * y_v + (1.0f - a) * smooth_yaw;
      last_rc_packet_ms = millis(); send_telemetry_pending = true;
    } else if (cmd == 120 && rem >= 1) {
      current_link_quality = lq;
      if (raw[idx++] == 0) { CliAutoCal::onDisarmRequested(); is_armed = false; }
      else if (!is_failsafe) is_armed = true;
      last_rc_packet_ms = millis();
    } else if (cmd == 130) {
      current_link_quality = lq; startChannelScanNonBlocking(); last_rc_packet_ms = millis();
    } else if (cmd == 140 && rem >= 1) {
      current_link_quality = lq;
      uint8_t tc = (uint8_t)constrainI(raw[idx++], 0, 99);
      target_pending_chan = tc; pending_channel_switch = true;
      rtx_switched_flag = true; rtx_new_switched_chan = tc;
      queueDelayedRtxResponse(RTX_RESP_240_CHAN_SW, tc, 0, 0);
      last_rc_packet_ms = millis();
    } else if (cmd == 150 && rem >= 4) {
      current_link_quality = lq;
      uint16_t mhz = ((uint16_t)raw[idx] << 8) | raw[idx + 1];
      uint16_t mw = ((uint16_t)raw[idx + 2] << 8) | raw[idx + 3];
      vtxSetFrequencyAndPower(mhz, mw, true);
      queueDelayedRtxResponse(RTX_RESP_250_VTX_ACK, currentConfig.vtx_mhz, 0, currentConfig.vtx_mw);
      last_rc_packet_ms = millis();
    }
  }
  rtx_set_rx_mode();
}

static uint8_t getActiveStatusErrorCode() {
  return (fc_error_ram != FC_ERR_NONE) ? fc_error_ram : fc_latched_error_ram;
}

void startTelemetryNonBlocking() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_scan_active || rtx_recovering) return;
  if (gpioRead(Reg::GPIOB, 1)) return;

  uint8_t stby = 0x00; rtx_write_command(0x80, &stby, 1);
  if (!rtx_try_ready_short()) return;

  uint8_t txp[32]; uint8_t p = 0;
  if (currentConfig.rtx_addr_en) {
    txp[p++] = (uint8_t)(currentConfig.rtx_addr >> 8); txp[p++] = (uint8_t)(currentConfig.rtx_addr & 0xFF);
  }

  DelayedRtxRespItem qi;
  if (hasQueuedRtxResponse() && popDelayedRtxResponse(qi)) {
    txp[p++] = (uint8_t)qi.type; txp[p++] = current_link_quality;
    if (qi.type == RTX_RESP_202_ERR_EVENT) { txp[p++] = (uint8_t)qi.vtx_mhz; txp[p++] = qi.tier; }
    else if (qi.type == RTX_RESP_230_SCAN_BEST || qi.type == RTX_RESP_240_CHAN_SW) txp[p++] = (uint8_t)qi.vtx_mhz;
    else if (qi.type == RTX_RESP_250_VTX_ACK) {
      txp[p++] = (uint8_t)(qi.vtx_mhz >> 8); txp[p++] = (uint8_t)(qi.vtx_mhz & 0xFF);
      txp[p++] = (uint8_t)(qi.vtx_mw >> 8);  txp[p++] = (uint8_t)(qi.vtx_mw & 0xFF);
    }
  } else {
    txp[p++] = 210; txp[p++] = current_link_quality;
    uint16_t vc = (uint16_t)constrainL(lroundf(v_batt * 100.0f), 0L, 65535L);
    uint16_t ic = (uint16_t)constrainL(lroundf(current_a * 100.0f), 0L, 65535L);
    uint16_t wd = (uint16_t)constrainL(lroundf(power_w * 10.0f), 0L, 65535L);
    uint16_t whm = (uint16_t)constrainL(lroundf(consumed_wh * 1000.0f), 0L, 65535L);
    int16_t acm = (int16_t)constrainL(lroundf(baro_alt_m * 100.0f), -32768L, 32767L);
    uint16_t hd = (uint16_t)constrainL(lroundf(compass_heading_deg * 10.0f), 0L, 3600L);
    uint16_t lm = (uint16_t)constrainL(lroundf(dbg_lidar_m * 1000.0f), 0L, 65535L);
    int16_t tc10 = (int16_t)constrainL(lroundf(current_chip_temp * 10.0f), -32768L, 32767L);
    txp[p++] = (uint8_t)(vc >> 8);  txp[p++] = (uint8_t)(vc & 0xFF);
    txp[p++] = (uint8_t)(ic >> 8);  txp[p++] = (uint8_t)(ic & 0xFF);
    txp[p++] = (uint8_t)(wd >> 8);  txp[p++] = (uint8_t)(wd & 0xFF);
    txp[p++] = (uint8_t)(whm >> 8); txp[p++] = (uint8_t)(whm & 0xFF);
    txp[p++] = (uint8_t)((uint16_t)acm >> 8); txp[p++] = (uint8_t)((uint16_t)acm & 0xFF);
    txp[p++] = (uint8_t)(hd >> 8);  txp[p++] = (uint8_t)(hd & 0xFF);
    txp[p++] = (uint8_t)(lm >> 8);  txp[p++] = (uint8_t)(lm & 0xFF);
    txp[p++] = (uint8_t)((uint16_t)tc10 >> 8); txp[p++] = (uint8_t)((uint16_t)tc10 & 0xFF);
    txp[p++] = (uint8_t)(is_armed ? 1 : 0);
    txp[p++] = getActiveStatusErrorCode();
  }

  uint8_t bc[2] = {0x00, 0x00}; rtx_write_command(0x8F, bc, 2);
  spi2BeginTx(); spi2Transfer(0x0E); spi2Transfer(0x00);
  for (uint8_t i = 0; i < p; i++) spi2Transfer(txp[i]);
  spi2EndTx();

  uint8_t pp[6] = {0x00, 0x08, 0x00, p, (uint8_t)(currentConfig.rtx_hw_crc_en ? 1 : 0), (uint8_t)(currentConfig.rtx_iq_inv ? 1 : 0)};
  rtx_write_command(0x8C, pp, 6);
  uint8_t txm[3] = {0x00, 0x00, 0x00}; rtx_write_command(0x83, txm, 3);

  rtx_tx_in_flight = true; rtx_tx_start_ms = millis();
  rtx_tx_timeout_ms = getTelemetryTxTimeoutMs(is_on_backup);
  send_telemetry_pending = false;

  if (pending_channel_switch) {
    pending_channel_switch = false; main_rtx_chan = target_pending_chan;
    rtx_apply_switch_after_tx = true; rtx_post_tx_chan = main_rtx_chan; rtx_post_tx_backup = false;
  }
}

void pollTelemetryTxState() {
  if (!rtx_tx_in_flight) return;
  uint32_t now_ms = millis();
  bool done = false, tout = false;

  if (!gpioRead(Reg::GPIOB, 1)) {
    uint8_t ist[2] = {0, 0}; rtx_read_command(0x12, ist, 2);
    if ((((uint16_t)ist[0] << 8) | ist[1]) & 0x01) {
      uint8_t clr[2] = {0x00, 0x01}; rtx_write_command(0x02, clr, 2); done = true;
    }
  }
  if (!done && ((now_ms - rtx_tx_start_ms) >= rtx_tx_timeout_ms)) {
    tout = true; rtx_error_counter++; setFcErrorRam(FC_ERR_RTX_TX_TIMEOUT);
    if (!is_on_backup) {
      is_on_backup = true; active_rtx_chan = (uint8_t)constrainI(currentConfig.rtx_bak_chan, 0, 99);
      rtx_apply_switch_after_tx = true; rtx_post_tx_chan = active_rtx_chan; rtx_post_tx_backup = true;
    }
  }
  if (done || tout) {
    rtx_tx_in_flight = false;
    if (rtx_apply_switch_after_tx) {
      rtx_apply_switch_after_tx = false; is_on_backup = rtx_post_tx_backup; active_rtx_chan = rtx_post_tx_chan;
      applyRtxConfiguration(is_on_backup);
    } else rtx_set_rx_mode();
  }
}

// ==============================================================================
// 8. УПРАВЛЕНИЕ VTX (USART3: PB10 TX / PB11 RX)
// ==============================================================================
static inline uint16_t vtxClampMw(uint16_t mw) {
  return (mw == 0) ? 25U : (uint16_t)constrainI((int)mw, 1, 10000);
}

static uint8_t vtxMwToDbmInt(uint16_t mw) {
  mw = vtxClampMw(mw);
  if (mw >= 10000) return 40;
  uint8_t db = 0; uint32_t v = mw;
  while (v >= 10) { v /= 10; db += 10; }
  if (v >= 8) db += 9; else if (v >= 6) db += 8; else if (v >= 5) db += 7;
  else if (v >= 4) db += 6; else if (v >= 3) db += 5; else if (v >= 2) db += 3;
  return (uint8_t)constrainI((int)db, 0, 40);
}

static uint16_t vtxDbmToMwInt(uint8_t dbm) {
  static const uint16_t kDec[4] = {1, 2, 4, 6};
  uint8_t d = (uint8_t)constrainI((int)dbm, 0, 40);
  uint32_t mw = kDec[d % 10 < 3 ? 0 : (d % 10 < 5 ? 1 : (d % 10 < 8 ? 2 : 3))];
  for (uint8_t b = 0; b < (d / 10); b++) mw *= 10;
  return (uint16_t)constrainL((long)mw, 1L, 10000L);
}

static inline uint8_t vtxEncodeSaDirectDbm(uint16_t mw) {
  return (uint8_t)(0x80 | (vtxMwToDbmInt(mw) & 0x7F));
}

static void vtxEnsureBaud(uint32_t baud) {
  if (!vtx_uart_ready || vtx_baud != baud) {
    if (vtx_uart_ready) { uart3.flush(); uart3.end(); }
    uart3.begin(baud);
    vtx_baud = baud; vtx_uart_ready = true; tramp_rx_idx = sa_rx_idx = 0;
    while (uart3.available() > 0) (void)uart3.read();
  }
}

static uint8_t smartAudioCrc8(const uint8_t *d, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t trampCrc8(const uint8_t *f) {
  uint16_t s = 0; for (uint8_t i = 1; i <= 13; i++) s += f[i]; return (uint8_t)(s & 0xFF);
}

static void vtxMarkFail() { vtx_link_ok = false; setFcErrorRam(FC_ERR_SA_VTX_FAIL); }

static void vtxMarkOk(uint8_t proto, uint32_t now_ms) {
  if (proto == VTX_PROTO_SMARTAUDIO || proto == VTX_PROTO_TRAMP) vtx_detected_proto = proto;
  vtx_link_ok = true; vtx_last_probe_ms = now_ms; vtx_initial_probe_done = true;
}

static uint8_t vtxResolveProto() {
  if (currentConfig.vtx_prot == VTX_PROT_TRAMP) return VTX_PROTO_TRAMP;
  if (currentConfig.vtx_prot == VTX_PROT_SMARTAUDIO) return VTX_PROTO_SMARTAUDIO;
  return vtx_detected_proto;
}

static bool pollSmartAudioRx() {
  bool seen = false;
  while (uart3.available() > 0) {
    uint8_t b = (uint8_t)uart3.read();
    if (sa_rx_idx == 0) { if (b == 0xAA) sa_rx_buf[sa_rx_idx++] = b; }
    else if (sa_rx_idx == 1) { if (b == 0x55) sa_rx_buf[sa_rx_idx++] = b; else sa_rx_idx = (b == 0xAA) ? 1 : 0; }
    else {
      sa_rx_buf[sa_rx_idx++] = b;
      if (sa_rx_idx >= 4) {
        uint8_t plen = sa_rx_buf[3];
        if (plen > 10) { sa_rx_idx = 0; continue; }
        uint8_t tlen = (uint8_t)(4 + plen + 1);
        if (sa_rx_idx == tlen) {
          if (smartAudioCrc8(sa_rx_buf, (uint8_t)(tlen - 1)) == sa_rx_buf[tlen - 1]) {
            if (plen >= 5) {
              uint8_t dbm = sa_rx_buf[8] & 0x7F;
              if (dbm <= 40) { vtx_actual_dbm = (int8_t)dbm; vtx_actual_mw = vtxDbmToMwInt(dbm); }
            }
            seen = true;
          }
          sa_rx_idx = 0;
        }
      }
    }
  }
  return seen;
}

static bool pollTrampRx() {
  bool seen = false;
  while (uart3.available() > 0) {
    uint8_t b = (uint8_t)uart3.read();
    if (tramp_rx_idx == 0) { if (b == '$') tramp_rx_buf[tramp_rx_idx++] = b; }
    else {
      tramp_rx_buf[tramp_rx_idx++] = b;
      if (tramp_rx_idx == 16) {
        if (tramp_rx_buf[15] == 0x00 && tramp_rx_buf[14] == trampCrc8(tramp_rx_buf)) {
          uint16_t v16 = (uint16_t)tramp_rx_buf[2] | ((uint16_t)tramp_rx_buf[3] << 8);
          if ((tramp_rx_buf[1] == 'r' || tramp_rx_buf[1] == 'v') && v16 > 0 && v16 <= 10000) {
            vtx_actual_mw = v16; vtx_actual_dbm = (int8_t)vtxMwToDbmInt(v16);
          }
          seen = true;
        }
        tramp_rx_idx = 0;
      }
    }
  }
  return seen;
}

static bool sendSmartAudioFrame(uint8_t cmd, const uint8_t *pl, uint8_t plen) {
  if (vtx_hw_unavailable) { setFcErrorRam(FC_ERR_SA_VTX_FAIL); return false; }
  vtxEnsureBaud(SMARTAUDIO_BAUD_RATE);
  sa_rx_idx = 0; while (uart3.available() > 0) (void)uart3.read();
  uint8_t f[12], idx = 0;
  f[idx++] = 0xAA; f[idx++] = 0x55; f[idx++] = cmd; f[idx++] = plen;
  for (uint8_t i = 0; i < plen && idx < 11; i++) f[idx++] = pl[i];
  f[idx] = smartAudioCrc8(f, idx); idx++;
  if (uart3.write(f, idx) != idx) { vtxMarkFail(); return false; }
  return true;
}

static bool sendTrampFrame(char cmd, uint16_t param) {
  if (vtx_hw_unavailable) { setFcErrorRam(FC_ERR_SA_VTX_FAIL); return false; }
  vtxEnsureBaud(TRAMP_BAUD_RATE);
  tramp_rx_idx = 0; while (uart3.available() > 0) (void)uart3.read();
  uint8_t f[16]; memset(f, 0, 16);
  f[0] = '$'; f[1] = (uint8_t)cmd; f[2] = (uint8_t)(param & 0xFF); f[3] = (uint8_t)((param >> 8) & 0xFF);
  f[14] = trampCrc8(f);
  if (uart3.write(f, 16) != 16) { vtxMarkFail(); return false; }
  return true;
}

static bool vtxSendFreqCmd(uint8_t proto) {
  if (proto == VTX_PROTO_TRAMP) return sendTrampFrame('F', currentConfig.vtx_mhz);
  uint8_t fb[2] = {(uint8_t)(currentConfig.vtx_mhz >> 8), (uint8_t)(currentConfig.vtx_mhz & 0xFF)};
  return sendSmartAudioFrame(0x09, fb, 2);
}

static bool vtxSendPowerCmd(uint8_t proto) {
  if (proto == VTX_PROTO_TRAMP) return sendTrampFrame('P', currentConfig.vtx_mw);
  uint8_t pb = vtxEncodeSaDirectDbm(currentConfig.vtx_mw);
  return sendSmartAudioFrame(0x05, &pb, 1);
}

static void vtxArmCmdAck(uint32_t now_ms) {
  vtx_last_tx_ms = vtx_step_start_ms = now_ms;
  vtx_probe_step = VTX_STEP_WAIT_CMD_ACK;
}

void vtxSetFrequencyMhz(uint16_t mhz, bool send_now, bool save_eeprom) {
  currentConfig.vtx_mhz = (uint16_t)constrainI((int)mhz, 5000, 6000);
  vtx_send_freq_req = true;
  if (send_now && currentConfig.vtx_en) {
    uint8_t proto = vtxResolveProto();
    if ((proto == VTX_PROTO_TRAMP || proto == VTX_PROTO_SMARTAUDIO) && vtxSendFreqCmd(proto)) {
      vtx_send_freq_req = false; vtxArmCmdAck(millis());
    }
  }
  if (save_eeprom) saveSettings();
}

void vtxSetPowerMw(uint16_t mw, bool send_now, bool save_eeprom) {
  currentConfig.vtx_mw = vtxClampMw(mw);
  vtx_send_power_req = true;
  if (send_now && currentConfig.vtx_en) {
    uint8_t proto = vtxResolveProto();
    if ((proto == VTX_PROTO_TRAMP || proto == VTX_PROTO_SMARTAUDIO) && vtxSendPowerCmd(proto)) {
      vtx_send_power_req = false; vtxArmCmdAck(millis());
    }
  }
  if (save_eeprom) saveSettings();
}

void vtxSetFrequencyAndPower(uint16_t mhz, uint16_t power_mw, bool save_eeprom) {
  currentConfig.vtx_mhz = (uint16_t)constrainI((int)mhz, 5000, 6000);
  currentConfig.vtx_mw = vtxClampMw(power_mw);
  vtx_send_freq_req = vtx_send_power_req = true;
  if (save_eeprom) saveSettings();
}

void vtxInit() {
  gpioConfigPin(Reg::GPIOB, 10, GPIO_AF_PP_50M);
  gpioConfigPin(Reg::GPIOB, 11, GPIO_IN_PULL, true);

  vtx_hw_unavailable = vtx_initial_probe_done = false;
  vtx_probe_step = VTX_STEP_IDLE;
  vtx_step_start_ms = vtx_baud = 0;
  vtx_link_ok = false; vtx_detected_proto = VTX_PROTO_NONE;
  vtx_actual_dbm = 0; vtx_actual_mw = 0; tramp_rx_idx = sa_rx_idx = 0;

  if (!currentConfig.vtx_en) {
    vtx_uart_ready = vtx_send_freq_req = vtx_send_power_req = false;
    return;
  }
  uint8_t proto = vtxResolveProto();
  vtxEnsureBaud((proto == VTX_PROTO_TRAMP) ? TRAMP_BAUD_RATE : SMARTAUDIO_BAUD_RATE);
  vtx_send_freq_req = vtx_send_power_req = true;
  vtx_last_probe_ms = millis();
}

static void vtxFinishProbeTimeout(uint32_t now_ms) {
  vtx_initial_probe_done = true; vtx_last_probe_ms = now_ms; vtxMarkFail(); vtx_probe_step = VTX_STEP_IDLE;
}

void vtxStepNonBlocking(uint32_t now_ms) {
  if (!currentConfig.vtx_en) { vtx_link_ok = false; vtx_probe_step = VTX_STEP_IDLE; return; }
  if (vtx_hw_unavailable) { setFcErrorRam(FC_ERR_SA_VTX_FAIL); return; }
  if (!vtx_uart_ready) { vtxInit(); if (!vtx_uart_ready) return; }

  uint8_t known = vtxResolveProto();
  bool auto_fb = (currentConfig.vtx_prot == VTX_PROT_AUTO) && (known == VTX_PROTO_NONE);

  if (vtx_probe_step == VTX_STEP_WAIT_SA) {
    if (pollSmartAudioRx()) { vtxMarkOk(VTX_PROTO_SMARTAUDIO, now_ms); vtx_probe_step = VTX_STEP_IDLE; return; }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) {
      if (auto_fb && sendTrampFrame('v', 0)) { vtx_last_tx_ms = vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_TRAMP; }
      else vtxFinishProbeTimeout(now_ms);
    }
    return;
  }
  if (vtx_probe_step == VTX_STEP_WAIT_TRAMP) {
    if (pollTrampRx()) { vtxMarkOk(VTX_PROTO_TRAMP, now_ms); vtx_probe_step = VTX_STEP_IDLE; return; }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) vtxFinishProbeTimeout(now_ms);
    return;
  }
  if (vtx_probe_step == VTX_STEP_WAIT_CMD_ACK) {
    bool ack = (known == VTX_PROTO_TRAMP) ? pollTrampRx() : pollSmartAudioRx();
    if (ack) {
      uint8_t ok_proto = (known != VTX_PROTO_NONE) ? known : (uint8_t)VTX_PROTO_SMARTAUDIO;
      vtxMarkOk(ok_proto, now_ms);
      vtx_probe_step = VTX_STEP_IDLE;
      return;
    }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) vtxFinishProbeTimeout(now_ms);
    return;
  }

  if (known == VTX_PROTO_TRAMP && pollTrampRx()) vtxMarkOk(VTX_PROTO_TRAMP, now_ms);
  else if (known == VTX_PROTO_SMARTAUDIO && pollSmartAudioRx()) vtxMarkOk(VTX_PROTO_SMARTAUDIO, now_ms);

  if ((now_ms - vtx_last_tx_ms) < VTX_CMD_INTERVAL_MS) return;

  if (vtx_send_freq_req || vtx_send_power_req) {
    uint8_t p = (known != VTX_PROTO_NONE) ? known : (uint8_t)VTX_PROTO_SMARTAUDIO;
    if (vtx_send_freq_req && vtxSendFreqCmd(p)) { vtx_send_freq_req = false; vtxArmCmdAck(now_ms); return; }
    if (vtx_send_power_req && vtxSendPowerCmd(p)) { vtx_send_power_req = false; vtxArmCmdAck(now_ms); return; }
  }

  if (!vtx_initial_probe_done || (now_ms - vtx_last_probe_ms) >= VTX_PROBE_INTERVAL_MS) {
    if (known == VTX_PROTO_TRAMP) {
      if (sendTrampFrame('v', 0)) { vtx_last_tx_ms = vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_TRAMP; }
    } else {
      if (sendSmartAudioFrame(0x03, nullptr, 0)) { vtx_last_tx_ms = vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_SA; }
    }
  }
}

// ==============================================================================
// 9. ДАТЧИКИ (I2C1 + АЦП)
// ==============================================================================
namespace SensorDriver {
  enum IMUType : uint8_t { IMU_UNKNOWN = 0, IMU_MPU6050 = 1, IMU_MPU6500 = 2, IMU_MPU9250 = 3, IMU_ICM20602 = 4, IMU_BMI160 = 5, IMU_QMI8658 = 6 };
  enum BaroType : uint8_t { BARO_UNKNOWN = 0, BARO_BMP280 = 1, BARO_BME280 = 2, BARO_MS5611 = 3, BARO_BMP180 = 4, BARO_BMP388 = 5, BARO_BMP390 = 6, BARO_DPS310 = 7 };
  enum MagType : uint8_t { MAG_UNKNOWN = 0, MAG_HMC5883L = 1, MAG_QMC5883L = 2, MAG_AK8963 = 3, MAG_IST8310 = 4, MAG_HSCDTD008A = 5, MAG_AK09911 = 6 };
  enum LidarType : uint8_t { LIDAR_NONE = 0, LIDAR_VL53L0X = 1, LIDAR_VL53L1X = 2, LIDAR_VL53L3CX = 3, LIDAR_VL53L4CX = 4 };
  enum Ms5611Phase : uint8_t { MS_IDLE = 0, MS_WAIT_D1, MS_WAIT_D2 };
  enum Bmp180Phase : uint8_t { B180_IDLE = 0, B180_WAIT_TEMP, B180_WAIT_PRESS };

  struct BaroCalib {
    uint16_t dig_T1 = 0; int16_t dig_T2 = 0, dig_T3 = 0;
    uint16_t dig_P1 = 0; int16_t dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0, dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;
    uint16_t ms_c[7] = {0};
    int16_t ac1 = 0, ac2 = 0, ac3 = 0; uint16_t ac4 = 0, ac5 = 0, ac6 = 0;
    int16_t b1 = 0, b2 = 0, mc = 0, md = 0;
    float b3_t1 = 0.0f, b3_t2 = 0.0f, b3_t3 = 0.0f;
    float b3_p1 = 0.0f, b3_p2 = 0.0f, b3_p3 = 0.0f, b3_p4 = 0.0f, b3_p5 = 0.0f;
    float b3_p6 = 0.0f, b3_p7 = 0.0f, b3_p8 = 0.0f, b3_p9 = 0.0f, b3_p10 = 0.0f, b3_p11 = 0.0f;
    int32_t dps_c00 = 0, dps_c10 = 0;
    int16_t dps_c01 = 0, dps_c11 = 0, dps_c20 = 0, dps_c21 = 0, dps_c30 = 0;
    float dps_kp = 524288.0f, dps_kt = 524288.0f;
    bool valid = false;
  } baroCal;

  struct MagCalib { float asa_x = 1.0f, asa_y = 1.0f, asa_z = 1.0f, scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f; } magCal;

  struct SensorStatus {
    IMUType imu_type = IMU_UNKNOWN; BaroType baro_type = BARO_UNKNOWN;
    MagType mag_type = MAG_UNKNOWN; LidarType lidar_type = LIDAR_NONE;
    uint8_t imu_addr = 0x68, baro_addr = 0x76, mag_addr = 0x0C, lidar_addr = 0x29;
    float gyro_scale = 131.0f, acc_scale = 16384.0f, temp_scale = 340.0f, temp_offset = 25.0f, lidar_max_m = 4.0f;
    bool imu_found = false, baro_found = false, mag_found = false, lidar_found = false;
  } activeSensors;

  Ms5611Phase ms5611_phase = MS_IDLE; uint32_t ms5611_conv_start_ms = 0, ms5611_d1_raw = 0, ms5611_d2_raw = 0;
  Bmp180Phase bmp180_phase = B180_IDLE; uint32_t bmp180_conv_start_ms = 0; int32_t bmp180_ut_raw = 0;

  void detectIMU(); void detectBaro(); void detectMag(); void detectLidar(); void initLidarHardware();
  bool calibrateGyro(bool silent = false); bool calibrateAcc(bool silent = false);
  void calibrateIMU(); void calibrateBaro(); void calibrateLidar();
  void calibrateBatVoltage(float actual_voltage); void calibrateCurrent(bool has_value, float actual_current_a);

  float getEffectiveLidarMaxM() { return activeSensors.lidar_max_m * 0.90f; }
  inline float getBatScaleV() { return constrainF(currentConfig.bat_scale_v, CFG_BAT_SCALE_V_MIN, CFG_BAT_SCALE_V_MAX); }
  inline float getBatScaleA() { return constrainF(currentConfig.bat_scale_a, CFG_BAT_SCALE_A_MIN, CFG_BAT_SCALE_A_MAX); }

  inline float computeCurrentAmpsFromAdc(uint16_t raw_adc) {
    float pin_v = ((float)raw_adc / 4095.0f) * 3.3f;
    float off_a = constrainF(currentConfig.bat_offset_a, CFG_BAT_OFFSET_A_MIN, CFG_BAT_OFFSET_A_MAX);
    float a = ((pin_v / SHUNT_RESISTANCE_OHM) - off_a) * getBatScaleA();
    return (a > 0.0f) ? a : 0.0f;
  }

  inline float wrapDeg0To360(float deg) {
    if (!isfinite(deg)) { setFcErrorRam(FC_ERR_NAN_INF_MATH); return 0.0f; }
    float w = fmodf(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    return (w >= 360.0f || w < 0.0f) ? 0.0f : w;
  }

  inline void rotateXYByYawAlign(float &x, float &y, uint8_t align) {
    float ox = x, oy = y;
    switch (align & 0x03) {
      case 1: x = oy; y = -ox; break;
      case 2: x = -ox; y = -oy; break;
      case 3: x = -oy; y = ox; break;
      default: break;
    }
  }

  inline float extractTempFromImuBuf(const uint8_t *buf) {
    return ((float)((int16_t)(buf[6] << 8 | buf[7])) / activeSensors.temp_scale) + activeSensors.temp_offset;
  }

  inline void packNormalizedImuToBuffer(uint8_t *out14, float ax_g, float ay_g, float az_g, float temp_c, float gx_dps, float gy_dps, float gz_dps) {
    int16_t iax = (int16_t)constrainF(ax_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t iay = (int16_t)constrainF(ay_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t iaz = (int16_t)constrainF(az_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t itm = (int16_t)constrainF((temp_c - activeSensors.temp_offset) * activeSensors.temp_scale, -32767.0f, 32767.0f);
    int16_t igx = (int16_t)constrainF(gx_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    int16_t igy = (int16_t)constrainF(gy_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    int16_t igz = (int16_t)constrainF(gz_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    out14[0] = (uint8_t)((uint16_t)iax >> 8); out14[1] = (uint8_t)((uint16_t)iax & 0xFF);
    out14[2] = (uint8_t)((uint16_t)iay >> 8); out14[3] = (uint8_t)((uint16_t)iay & 0xFF);
    out14[4] = (uint8_t)((uint16_t)iaz >> 8); out14[5] = (uint8_t)((uint16_t)iaz & 0xFF);
    out14[6] = (uint8_t)((uint16_t)itm >> 8); out14[7] = (uint8_t)((uint16_t)itm & 0xFF);
    out14[8] = (uint8_t)((uint16_t)igx >> 8); out14[9] = (uint8_t)((uint16_t)igx & 0xFF);
    out14[10] = (uint8_t)((uint16_t)igy >> 8); out14[11] = (uint8_t)((uint16_t)igy & 0xFF);
    out14[12] = (uint8_t)((uint16_t)igz >> 8); out14[13] = (uint8_t)((uint16_t)igz & 0xFF);
  }

  void checkBusAndRecover() {
    if (i2c_error_counter > I2C_MAX_ERRORS) {
      setFcErrorRam(FC_ERR_I2C_BUS);
      initI2c1();
      ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE; i2c_error_counter = 0;
    }
  }

  static bool recordI2cError() { i2c_error_counter++; setFcErrorRam(FC_ERR_I2C_BUS); return false; }

  bool readReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    checkBusAndRecover();
    if (i2c1WriteRead(addr, &reg, 1, buf, len)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  bool readReg16Addr(uint8_t addr, uint16_t reg16, uint8_t *buf, uint8_t len) {
    checkBusAndRecover();
    uint8_t r[2] = {(uint8_t)(reg16 >> 8), (uint8_t)(reg16 & 0xFF)};
    if (i2c1WriteRead(addr, r, 2, buf, len)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    checkBusAndRecover();
    uint8_t t[2] = {reg, val};
    if (i2c1WriteRead(addr, t, 2, nullptr, 0)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  bool writeReg16Addr8Val(uint8_t addr, uint16_t reg16, uint8_t val) {
    checkBusAndRecover();
    uint8_t t[3] = {(uint8_t)(reg16 >> 8), (uint8_t)(reg16 & 0xFF), val};
    if (i2c1WriteRead(addr, t, 3, nullptr, 0)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  void enableImuI2cBypass() {
    if (activeSensors.imu_found && (activeSensors.imu_type == IMU_MPU9250 || activeSensors.imu_type == IMU_MPU6500)) {
      writeReg(activeSensors.imu_addr, 0x6A, 0x00); delay(2);
      writeReg(activeSensors.imu_addr, 0x37, 0x02); delay(5);
    }
  }

  static void initSpecificImu(uint8_t addr, IMUType type) {
    activeSensors.imu_addr = addr; activeSensors.imu_type = type;
    if (type == IMU_BMI160) {
      activeSensors.acc_scale = 16384.0f; activeSensors.gyro_scale = 131.0f; activeSensors.temp_scale = 512.0f; activeSensors.temp_offset = 23.0f;
      writeReg(addr, 0x7E, 0x11); delay(5); writeReg(addr, 0x7E, 0x15); delay(10);
      writeReg(addr, 0x40, 0x16); writeReg(addr, 0x41, 0x03); writeReg(addr, 0x42, 0x17); writeReg(addr, 0x43, 0x03);
    } else if (type == IMU_QMI8658) {
      activeSensors.acc_scale = 8192.0f; activeSensors.gyro_scale = 128.0f; activeSensors.temp_scale = 256.0f; activeSensors.temp_offset = 0.0f;
      writeReg(addr, 0x02, 0x60); writeReg(addr, 0x03, 0x15); writeReg(addr, 0x04, 0x45); writeReg(addr, 0x05, 0x11); writeReg(addr, 0x07, 0x03);
    } else if (type == IMU_ICM20602) {
      activeSensors.acc_scale = 16384.0f; activeSensors.gyro_scale = 131.0f; activeSensors.temp_scale = 326.8f; activeSensors.temp_offset = 25.0f;
      writeReg(addr, 0x6B, 0x01); delay(10); writeReg(addr, 0x1A, 0x03); writeReg(addr, 0x1B, 0x00); writeReg(addr, 0x1C, 0x00); writeReg(addr, 0x1D, 0x03);
    } else {
      activeSensors.acc_scale = 16384.0f; activeSensors.gyro_scale = 131.0f; activeSensors.temp_scale = 340.0f; activeSensors.temp_offset = 25.0f;
      writeReg(addr, 0x6B, 0x00); delay(10); writeReg(addr, 0x1A, 0x03); writeReg(addr, 0x1B, 0x00); writeReg(addr, 0x1C, 0x00);
      if (type == IMU_MPU6500 || type == IMU_MPU9250) writeReg(addr, 0x1D, 0x03);
    }
    activeSensors.imu_found = true; err_imu_fail = false;
    if ((type == IMU_MPU9250 || type == IMU_MPU6500) && currentConfig.com_en) enableImuI2cBypass();
  }

  static bool probeImuOnce() {
    uint8_t id = 0, req = currentConfig.imu_id;
    const uint8_t addrs689[2] = {0x68, 0x69};
    const uint8_t addrsQmi[4] = {0x6A, 0x6B, 0x68, 0x69};

    if (req == IMU_BMI160 || req == IMU_UNKNOWN) {
      for (uint8_t i = 0; i < 2; i++) if (readReg(addrs689[i], 0x00, &id, 1) && id == 0xD1) { initSpecificImu(addrs689[i], IMU_BMI160); return true; }
      if (req == IMU_BMI160) return false;
    }
    if (req == IMU_QMI8658 || req == IMU_UNKNOWN) {
      for (uint8_t i = 0; i < 4; i++) if (readReg(addrsQmi[i], 0x00, &id, 1) && id == 0x05) { initSpecificImu(addrsQmi[i], IMU_QMI8658); return true; }
      if (req == IMU_QMI8658) return false;
    }
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t a = addrs689[i];
      if (!readReg(a, 0x75, &id, 1)) continue;
      if ((req == IMU_MPU6050 || req == IMU_UNKNOWN) && id == 0x68) { initSpecificImu(a, IMU_MPU6050); return true; }
      if ((req == IMU_MPU6500 || req == IMU_MPU9250 || req == IMU_UNKNOWN) && id == 0x70) {
        if (req == IMU_MPU6500) { initSpecificImu(a, IMU_MPU6500); return true; }
        uint8_t ak = 0; writeReg(a, 0x6A, 0x00); delay(2); writeReg(a, 0x37, 0x02); delay(5);
        initSpecificImu(a, (readReg(0x0C, 0x00, &ak, 1) && ak == 0x48) ? IMU_MPU9250 : IMU_MPU6500);
        return true;
      }
      if ((req == IMU_MPU9250 || req == IMU_UNKNOWN) && (id == 0x71 || id == 0x73)) { initSpecificImu(a, IMU_MPU9250); return true; }
      if ((req == IMU_ICM20602 || req == IMU_UNKNOWN) && id == 0x12) { initSpecificImu(a, IMU_ICM20602); return true; }
    }
    return false;
  }

  void detectIMU() {
    activeSensors.imu_found = false; activeSensors.imu_type = IMU_UNKNOWN; activeSensors.temp_offset = 25.0f;
    if (probeImuOnce()) return;
    err_imu_fail = true; setFcErrorRam(FC_ERR_IMU_FAIL);
  }

  bool readMPU6050(uint8_t *buffer) {
    if (!activeSensors.imu_found) return false;
    if (activeSensors.imu_type == IMU_BMI160) {
      uint8_t raw[14]; if (!readReg(activeSensors.imu_addr, 0x0C, raw, 14)) return false;
      packNormalizedImuToBuffer(buffer,
        (float)((int16_t)(raw[7] << 8 | raw[6])) / activeSensors.acc_scale,
        (float)((int16_t)(raw[9] << 8 | raw[8])) / activeSensors.acc_scale,
        (float)((int16_t)(raw[11] << 8 | raw[10])) / activeSensors.acc_scale,
        ((float)((int16_t)(raw[13] << 8 | raw[12])) / activeSensors.temp_scale) + activeSensors.temp_offset,
        (float)((int16_t)(raw[1] << 8 | raw[0])) / activeSensors.gyro_scale,
        (float)((int16_t)(raw[3] << 8 | raw[2])) / activeSensors.gyro_scale,
        (float)((int16_t)(raw[5] << 8 | raw[4])) / activeSensors.gyro_scale);
      return true;
    }
    if (activeSensors.imu_type == IMU_QMI8658) {
      uint8_t tb[2], xb[12];
      if (!readReg(activeSensors.imu_addr, 0x33, tb, 2) || !readReg(activeSensors.imu_addr, 0x35, xb, 12)) return false;
      packNormalizedImuToBuffer(buffer,
        (float)((int16_t)(xb[1] << 8 | xb[0])) / activeSensors.acc_scale,
        (float)((int16_t)(xb[3] << 8 | xb[2])) / activeSensors.acc_scale,
        (float)((int16_t)(xb[5] << 8 | xb[4])) / activeSensors.acc_scale,
        ((float)((int16_t)(tb[1] << 8 | tb[0])) / activeSensors.temp_scale) + activeSensors.temp_offset,
        (float)((int16_t)(xb[7] << 8 | xb[6])) / activeSensors.gyro_scale,
        (float)((int16_t)(xb[9] << 8 | xb[8])) / activeSensors.gyro_scale,
        (float)((int16_t)(xb[11] << 8 | xb[10])) / activeSensors.gyro_scale);
      return true;
    }
    return readReg(activeSensors.imu_addr, 0x3B, buffer, 14);
  }

  inline float pressureToAltitudeMeters(float p_pa) { return 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.1903f)); }
  inline bool isPressureValidPa(float p_pa) { return p_pa >= 30000.0f && p_pa <= 125000.0f && isfinite(p_pa); }
  static void markBaroError() { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_BARO_FAIL); }

  bool readBme280PressurePa(float &p_pa) {
    uint8_t raw[6]; if (!readReg(activeSensors.baro_addr, 0xF7, raw, 6)) return false;
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t v1t = ((((adc_T >> 3) - ((int32_t)baroCal.dig_T1 << 1))) * (int32_t)baroCal.dig_T2) >> 11;
    int32_t v2t = (((((adc_T >> 4) - (int32_t)baroCal.dig_T1) * ((adc_T >> 4) - (int32_t)baroCal.dig_T1)) >> 12) * (int32_t)baroCal.dig_T3) >> 14;
    int64_t v1p = ((int64_t)(v1t + v2t)) - 128000;
    int64_t v2p = v1p * v1p * (int64_t)baroCal.dig_P6 + ((v1p * (int64_t)baroCal.dig_P5) << 17) + (((int64_t)baroCal.dig_P4) << 35);
    v1p = ((v1p * v1p * (int64_t)baroCal.dig_P3) >> 8) + ((v1p * (int64_t)baroCal.dig_P2) << 12);
    v1p = (((((int64_t)1) << 47) + v1p)) * (int64_t)baroCal.dig_P1 >> 33;
    if (v1p == 0) return false;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2p) * 3125) / v1p;
    v1p = (((int64_t)baroCal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    v2p = (((int64_t)baroCal.dig_P8) * p) >> 19;
    p = ((p + v1p + v2p) >> 8) + (((int64_t)baroCal.dig_P7) << 4);
    p_pa = (float)p / 256.0f;
    return isPressureValidPa(p_pa);
  }

  bool computeMs5611AltitudeFromRaw(uint32_t d1, uint32_t d2, float &alt_m) {
    if (d1 == 0 || d2 == 0) return false;
    int32_t dT = (int32_t)d2 - ((int32_t)baroCal.ms_c[5] << 8);
    int64_t temp = 2000LL + (((int64_t)dT * baroCal.ms_c[6]) >> 23);
    int64_t off = ((int64_t)baroCal.ms_c[2] << 16) + (((int64_t)baroCal.ms_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)baroCal.ms_c[1] << 15) + (((int64_t)baroCal.ms_c[3] * dT) >> 8);
    if (temp < 2000) {
      int64_t t2 = ((int64_t)dT * dT) >> 31, dt = temp - 2000LL;
      int64_t o2 = (5LL * dt * dt) >> 1, s2 = (5LL * dt * dt) >> 2;
      if (temp < -1500) { int64_t d15 = temp + 1500LL; o2 += 7LL * d15 * d15; s2 += (11LL * d15 * d15) >> 1; }
      temp -= t2; off -= o2; sens -= s2;
    }
    float p_pa = (float)(((((int64_t)d1 * sens) >> 21) - off) >> 15) * 10.0f;
    if (!isPressureValidPa(p_pa)) { markBaroError(); return false; }
    alt_m = pressureToAltitudeMeters(p_pa); return true;
  }

  float computeBmp180PressurePa(int32_t ut, int32_t up, uint8_t oss = 1) {
    int32_t x1 = ((ut - (int32_t)baroCal.ac6) * (int32_t)baroCal.ac5) >> 15;
    int32_t den = x1 + (int32_t)baroCal.md; if (den == 0) den = 1;
    int32_t x2 = ((int32_t)baroCal.mc << 11) / den;
    int32_t b5 = x1 + x2, b6 = b5 - 4000;
    x1 = ((int32_t)baroCal.b2 * ((b6 * b6) >> 12)) >> 11; x2 = ((int32_t)baroCal.ac2 * b6) >> 11;
    int32_t b3 = (((((int32_t)baroCal.ac1) * 4 + (x1 + x2)) << oss) + 2) >> 2;
    x1 = ((int32_t)baroCal.ac3 * b6) >> 13; x2 = ((int32_t)baroCal.b1 * ((b6 * b6) >> 12)) >> 16;
    uint32_t b4 = ((uint32_t)baroCal.ac4 * (uint32_t)((((x1 + x2) + 2) >> 2) + 32768)) >> 15; if (b4 == 0) b4 = 1;
    uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * (50000UL >> oss);
    int32_t p = (b7 < 0x80000000UL) ? (int32_t)((b7 * 2UL) / b4) : (int32_t)((b7 / b4) * 2UL);
    x1 = (p >> 8) * (p >> 8); x1 = (x1 * 3038) >> 16; x2 = (-7357 * p) >> 16;
    return (float)(p + ((x1 + x2 + 3791) >> 4));
  }

  bool stepBmp180Phased(uint32_t now_ms, float &alt_m) {
    constexpr uint8_t oss = 1;
    if (bmp180_phase == B180_IDLE) {
      if (!writeReg(activeSensors.baro_addr, 0xF4, 0x2E)) { markBaroError(); return false; }
      bmp180_conv_start_ms = now_ms; bmp180_phase = B180_WAIT_TEMP; return false;
    }
    if (bmp180_phase == B180_WAIT_TEMP) {
      if ((now_ms - bmp180_conv_start_ms) < BMP180_TEMP_DELAY_MS) return false;
      uint8_t tb[2]; if (!readReg(activeSensors.baro_addr, 0xF6, tb, 2)) { bmp180_phase = B180_IDLE; return false; }
      bmp180_ut_raw = (int32_t)((uint16_t)tb[0] << 8 | tb[1]);
      if (!writeReg(activeSensors.baro_addr, 0xF4, (uint8_t)(0x34 + (oss << 6)))) { bmp180_phase = B180_IDLE; return false; }
      bmp180_conv_start_ms = now_ms; bmp180_phase = B180_WAIT_PRESS; return false;
    }
    if ((now_ms - bmp180_conv_start_ms) < BMP180_PRESS_DELAY_MS) return false;
    uint8_t pb[3]; if (!readReg(activeSensors.baro_addr, 0xF6, pb, 3)) { bmp180_phase = B180_IDLE; return false; }
    int32_t up = (int32_t)(((((uint32_t)pb[0] << 16) | ((uint32_t)pb[1] << 8) | pb[2])) >> (8 - oss));
    bmp180_phase = B180_IDLE;
    float p_pa = computeBmp180PressurePa(bmp180_ut_raw, up, oss);
    if (!isPressureValidPa(p_pa)) { markBaroError(); return false; }
    alt_m = pressureToAltitudeMeters(p_pa); return true;
  }

  bool readBmp388PressurePa(float &p_pa) {
    uint8_t d[6]; if (!readReg(activeSensors.baro_addr, 0x04, d, 6)) return false;
    uint32_t up = ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
    uint32_t ut = ((uint32_t)d[5] << 16) | ((uint32_t)d[4] << 8) | d[3];
    float pd1 = (float)ut - baroCal.b3_t1;
    float tl = pd1 * baroCal.b3_t2 + pd1 * pd1 * baroCal.b3_t3;
    float tl2 = tl * tl, tl3 = tl2 * tl, fp = (float)up, fp2 = fp * fp, fp3 = fp2 * fp;
    float o1 = baroCal.b3_p5 + baroCal.b3_p6 * tl + baroCal.b3_p7 * tl2 + baroCal.b3_p8 * tl3;
    float o2 = fp * (baroCal.b3_p1 + baroCal.b3_p2 * tl + baroCal.b3_p3 * tl2 + baroCal.b3_p4 * tl3);
    p_pa = o1 + o2 + fp2 * (baroCal.b3_p9 + baroCal.b3_p10 * tl) + fp3 * baroCal.b3_p11;
    return isPressureValidPa(p_pa);
  }

  bool readDps310PressurePa(float &p_pa) {
    uint8_t d[6]; if (!readReg(activeSensors.baro_addr, 0x00, d, 6)) return false;
    int32_t pr = (int32_t)(((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2]); if (pr & 0x00800000) pr |= 0xFF000000;
    int32_t tr = (int32_t)(((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5]); if (tr & 0x00800000) tr |= 0xFF000000;
    float ts = (float)tr / baroCal.dps_kt, ps = (float)pr / baroCal.dps_kp;
    p_pa = (float)baroCal.dps_c00 + ps * ((float)baroCal.dps_c10 + ps * ((float)baroCal.dps_c20 + ps * (float)baroCal.dps_c30)) +
           ts * ((float)baroCal.dps_c01 + ps * ((float)baroCal.dps_c11 + ps * (float)baroCal.dps_c21));
    return isPressureValidPa(p_pa);
  }

  bool stepMs5611Phased(uint32_t now_ms, float &alt_m) {
    if (ms5611_phase == MS_IDLE) {
      if (!writeReg(activeSensors.baro_addr, 0x48, 0x00)) { markBaroError(); return false; }
      ms5611_conv_start_ms = now_ms; ms5611_phase = MS_WAIT_D1; return false;
    }
    if (ms5611_phase == MS_WAIT_D1) {
      if ((now_ms - ms5611_conv_start_ms) < MS5611_CONV_DELAY_MS) return false;
      uint8_t ab[3]; if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) { ms5611_phase = MS_IDLE; return false; }
      ms5611_d1_raw = ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2];
      if (!writeReg(activeSensors.baro_addr, 0x58, 0x00)) { ms5611_phase = MS_IDLE; markBaroError(); return false; }
      ms5611_conv_start_ms = now_ms; ms5611_phase = MS_WAIT_D2; return false;
    }
    if ((now_ms - ms5611_conv_start_ms) < MS5611_CONV_DELAY_MS) return false;
    uint8_t ab[3]; if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) { ms5611_phase = MS_IDLE; return false; }
    ms5611_d2_raw = ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2];
    ms5611_phase = MS_IDLE;
    return computeMs5611AltitudeFromRaw(ms5611_d1_raw, ms5611_d2_raw, alt_m);
  }

  bool readRawBaroAltitudeBlocking(float &alt_m) {
    if (!activeSensors.baro_found || !baroCal.valid) return false;
    if (activeSensors.baro_type == BARO_MS5611) {
      uint8_t ab[3];
      if (!writeReg(activeSensors.baro_addr, 0x48, 0x00)) return false;
      delay(MS5611_CONV_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) return false;
      uint32_t d1 = ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2];
      if (!writeReg(activeSensors.baro_addr, 0x58, 0x00)) return false;
      delay(MS5611_CONV_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) return false;
      return computeMs5611AltitudeFromRaw(d1, ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2], alt_m);
    }
    if (activeSensors.baro_type == BARO_BMP180) {
      constexpr uint8_t oss = 1; uint8_t tb[2], pb[3];
      if (!writeReg(activeSensors.baro_addr, 0xF4, 0x2E)) return false;
      delay(BMP180_TEMP_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0xF6, tb, 2)) return false;
      int32_t ut = (int32_t)((uint16_t)tb[0] << 8 | tb[1]);
      if (!writeReg(activeSensors.baro_addr, 0xF4, (uint8_t)(0x34 + (oss << 6)))) return false;
      delay(BMP180_PRESS_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0xF6, pb, 3)) return false;
      float p_pa = computeBmp180PressurePa(ut, (int32_t)(((((uint32_t)pb[0] << 16) | ((uint32_t)pb[1] << 8) | pb[2])) >> (8 - oss)), oss);
      if (!isPressureValidPa(p_pa)) return false;
      alt_m = pressureToAltitudeMeters(p_pa); return true;
    }
    float p_pa = 0.0f;
    bool ok = (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) ? readBmp388PressurePa(p_pa) :
              (activeSensors.baro_type == BARO_DPS310) ? readDps310PressurePa(p_pa) : readBme280PressurePa(p_pa);
    if (!ok) return false;
    alt_m = pressureToAltitudeMeters(p_pa); return true;
  }

  void initBaroHardware() {
    if (!activeSensors.baro_found) return;
    baroCal.valid = false; ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE;
    if (activeSensors.baro_type == BARO_BMP280 || activeSensors.baro_type == BARO_BME280) {
      uint8_t c[24];
      if (readReg(activeSensors.baro_addr, 0x88, c, 24)) {
        baroCal.dig_T1 = (uint16_t)(c[1] << 8 | c[0]); baroCal.dig_T2 = (int16_t)(c[3] << 8 | c[2]); baroCal.dig_T3 = (int16_t)(c[5] << 8 | c[4]);
        baroCal.dig_P1 = (uint16_t)(c[7] << 8 | c[6]); baroCal.dig_P2 = (int16_t)(c[9] << 8 | c[8]); baroCal.dig_P3 = (int16_t)(c[11] << 8 | c[10]);
        baroCal.dig_P4 = (int16_t)(c[13] << 8 | c[12]); baroCal.dig_P5 = (int16_t)(c[15] << 8 | c[14]); baroCal.dig_P6 = (int16_t)(c[17] << 8 | c[16]);
        baroCal.dig_P7 = (int16_t)(c[19] << 8 | c[18]); baroCal.dig_P8 = (int16_t)(c[21] << 8 | c[20]); baroCal.dig_P9 = (int16_t)(c[23] << 8 | c[22]);
        baroCal.valid = (baroCal.dig_T1 != 0 && baroCal.dig_P1 != 0);
      }
      writeReg(activeSensors.baro_addr, 0xF4, 0x2F);
    } else if (activeSensors.baro_type == BARO_BMP180) {
      uint8_t c[22];
      if (readReg(activeSensors.baro_addr, 0xAA, c, 22)) {
        baroCal.ac1 = (int16_t)(c[0] << 8 | c[1]); baroCal.ac2 = (int16_t)(c[2] << 8 | c[3]); baroCal.ac3 = (int16_t)(c[4] << 8 | c[5]);
        baroCal.ac4 = (uint16_t)(c[6] << 8 | c[7]); baroCal.ac5 = (uint16_t)(c[8] << 8 | c[9]); baroCal.ac6 = (uint16_t)(c[10] << 8 | c[11]);
        baroCal.b1 = (int16_t)(c[12] << 8 | c[13]); baroCal.b2 = (int16_t)(c[14] << 8 | c[15]);
        baroCal.mc = (int16_t)(c[18] << 8 | c[19]); baroCal.md = (int16_t)(c[20] << 8 | c[21]);
        baroCal.valid = (baroCal.ac4 != 0 && baroCal.ac5 != 0 && baroCal.ac6 != 0);
      }
    } else if (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) {
      uint8_t n[21];
      if (readReg(activeSensors.baro_addr, 0x31, n, 21)) {
        uint16_t ut1 = (uint16_t)(n[1] << 8 | n[0]), ut2 = (uint16_t)(n[3] << 8 | n[2]);
        int8_t st3 = (int8_t)n[4]; int16_t sp1 = (int16_t)(n[6] << 8 | n[5]), sp2 = (int16_t)(n[8] << 8 | n[7]);
        int8_t sp3 = (int8_t)n[9], sp4 = (int8_t)n[10]; uint16_t up5 = (uint16_t)(n[12] << 8 | n[11]), up6 = (uint16_t)(n[14] << 8 | n[13]);
        int8_t sp7 = (int8_t)n[15], sp8 = (int8_t)n[16]; int16_t sp9 = (int16_t)(n[18] << 8 | n[17]);
        int8_t sp10 = (int8_t)n[19], sp11 = (int8_t)n[20];
        baroCal.b3_t1 = (float)ut1 * 256.0f; baroCal.b3_t2 = (float)ut2 / 1073741824.0f; baroCal.b3_t3 = (float)st3 / 281474976710656.0f;
        baroCal.b3_p1 = ((float)sp1 - 16384.0f) / 1048576.0f; baroCal.b3_p2 = ((float)sp2 - 16384.0f) / 536870912.0f;
        baroCal.b3_p3 = (float)sp3 / 4294967296.0f; baroCal.b3_p4 = (float)sp4 / 137438953472.0f;
        baroCal.b3_p5 = (float)up5 * 8.0f; baroCal.b3_p6 = (float)up6 / 64.0f;
        baroCal.b3_p7 = (float)sp7 / 256.0f; baroCal.b3_p8 = (float)sp8 / 32768.0f;
        baroCal.b3_p9 = (float)sp9 / 281474976710656.0f; baroCal.b3_p10 = (float)sp10 / 281474976710656.0f;
        baroCal.b3_p11 = (float)sp11 / (281474976710656.0f * 131072.0f);
        baroCal.valid = (ut1 != 0 && ut2 != 0);
      }
      writeReg(activeSensors.baro_addr, 0x1C, 0x03); writeReg(activeSensors.baro_addr, 0x1F, 0x04); writeReg(activeSensors.baro_addr, 0x1B, 0x33);
    } else if (activeSensors.baro_type == BARO_DPS310) {
      uint8_t c[18];
      if (readReg(activeSensors.baro_addr, 0x10, c, 18)) {
        int32_t c00 = (int32_t)(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (c[5] >> 4)); if (c00 & 0x00080000) c00 |= 0xFFF00000;
        int32_t c10 = (int32_t)(((uint32_t)(c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | c[7]); if (c10 & 0x00080000) c10 |= 0xFFF00000;
        baroCal.dps_c00 = c00; baroCal.dps_c10 = c10;
        baroCal.dps_c01 = (int16_t)((uint16_t)c[8] << 8 | c[9]); baroCal.dps_c11 = (int16_t)((uint16_t)c[10] << 8 | c[11]);
        baroCal.dps_c20 = (int16_t)((uint16_t)c[12] << 8 | c[13]); baroCal.dps_c21 = (int16_t)((uint16_t)c[14] << 8 | c[15]); baroCal.dps_c30 = (int16_t)((uint16_t)c[16] << 8 | c[17]);
        baroCal.valid = (c00 != 0 || c10 != 0);
      }
      writeReg(activeSensors.baro_addr, 0x06, 0x54); writeReg(activeSensors.baro_addr, 0x07, 0x84); writeReg(activeSensors.baro_addr, 0x09, 0x0C);
      baroCal.dps_kp = baroCal.dps_kt = 253952.0f;
      writeReg(activeSensors.baro_addr, 0x08, 0x07);
    } else if (activeSensors.baro_type == BARO_MS5611) {
      uint8_t prom[2]; bool ok = true;
      for (uint8_t i = 1; i <= 6; i++) {
        if (readReg(activeSensors.baro_addr, (uint8_t)(0xA0 + (i * 2)), prom, 2)) { baroCal.ms_c[i] = ((uint16_t)prom[0] << 8) | prom[1]; if (baroCal.ms_c[i] == 0) ok = false; }
        else ok = false;
      }
      baroCal.valid = ok;
    }
    if (!baroCal.valid && currentConfig.baro_en) markBaroError();
  }

  static void bindAndInitBaro(uint8_t addr, BaroType t) { activeSensors.baro_addr = addr; activeSensors.baro_type = t; activeSensors.baro_found = true; initBaroHardware(); }

  void detectBaro() {
    uint8_t id = 0, req = currentConfig.bar_id;
    activeSensors.baro_found = false; activeSensors.baro_type = BARO_UNKNOWN; baroCal.valid = false;
    const uint8_t addrs[2] = {0x76, 0x77};
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t a = addrs[i];
      if ((req == BARO_DPS310 || req == BARO_UNKNOWN) && readReg(a, 0x0D, &id, 1) && id == 0x10) { bindAndInitBaro(a, BARO_DPS310); return; }
      if (readReg(a, 0xD0, &id, 1)) {
        if ((req == BARO_BMP180 || req == BARO_UNKNOWN) && id == 0x55) { bindAndInitBaro(a, BARO_BMP180); return; }
        if ((req == BARO_BMP388 || req == BARO_UNKNOWN) && id == 0x50) { bindAndInitBaro(a, BARO_BMP388); return; }
        if ((req == BARO_BMP280 || req == BARO_UNKNOWN) && id == 0x58) { bindAndInitBaro(a, BARO_BMP280); return; }
        if (id == 0x60) {
          uint8_t cm = 0; bool has_cm = readReg(a, 0xF4, &cm, 1);
          BaroType det = (has_cm && cm != 0x00) ? BARO_BME280 : BARO_BMP390;
          if (req == BARO_UNKNOWN || req == (uint8_t)det) { bindAndInitBaro(a, det); return; }
        }
      }
      if (req == BARO_MS5611 || req == BARO_UNKNOWN) {
        uint8_t prom[2]; if (readReg(a, 0xA2, prom, 2) && (prom[0] != 0 || prom[1] != 0)) { bindAndInitBaro(a, BARO_MS5611); return; }
      }
    }
    if (currentConfig.baro_en) markBaroError();
  }

  bool stepBaroNonBlocking(uint32_t now_ms, float &alt_m) {
    if (!currentConfig.baro_en || !activeSensors.baro_found || !baroCal.valid) return false;
    float raw_alt = 0.0f;
    if (activeSensors.baro_type == BARO_MS5611) { if (!stepMs5611Phased(now_ms, raw_alt)) return false; }
    else if (activeSensors.baro_type == BARO_BMP180) { if (!stepBmp180Phased(now_ms, raw_alt)) return false; }
    else {
      float p_pa = 0.0f;
      bool ok = (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) ? readBmp388PressurePa(p_pa) :
                (activeSensors.baro_type == BARO_DPS310) ? readDps310PressurePa(p_pa) : readBme280PressurePa(p_pa);
      if (!ok) { markBaroError(); return false; }
      raw_alt = pressureToAltitudeMeters(p_pa);
    }
    if (!baro_cal_ready) { baro_ground_alt_m = raw_alt; lpf_baro_alt.output = 0.0f; baro_cal_ready = true; }
    alt_m = lpf_baro_alt.update(raw_alt - baro_ground_alt_m);
    return true;
  }

  void calibrateBaro() {
    if (!activeSensors.baro_found || !baroCal.valid) { uart1.println("ERR: baro"); return; }
    float sum = 0.0f; uint8_t ok_c = 0;
    for (uint8_t i = 0; i < 12; i++) { float ra = 0.0f; if (readRawBaroAltitudeBlocking(ra)) { sum += ra; ok_c++; } }
    if (ok_c > 0) {
      baro_ground_alt_m = sum / (float)ok_c;
      lpf_baro_alt.output = lpf_baro_vz.output = baro_alt_m = baro_vz_mps = 0.0f;
      ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE; baro_cal_ready = true;
      uart1.println("OK: cal_bar");
    } else uart1.println("ERR: cal_bar");
  }

  void initMagHardware() {
    if (!activeSensors.mag_found) return;
    magCal.asa_x = magCal.asa_y = magCal.asa_z = 1.0f;
    if (activeSensors.mag_type == MAG_AK8963) {
      writeReg(activeSensors.mag_addr, 0x0A, 0x00); delay(5); writeReg(activeSensors.mag_addr, 0x0A, 0x1F); delay(5);
      uint8_t asa[3] = {128, 128, 128};
      if (readReg(activeSensors.mag_addr, 0x10, asa, 3)) {
        magCal.asa_x = (((float)asa[0] - 128.0f) / 256.0f) + 1.0f;
        magCal.asa_y = (((float)asa[1] - 128.0f) / 256.0f) + 1.0f;
        magCal.asa_z = (((float)asa[2] - 128.0f) / 256.0f) + 1.0f;
      }
      writeReg(activeSensors.mag_addr, 0x0A, 0x00); delay(5); writeReg(activeSensors.mag_addr, 0x0A, 0x16); delay(5);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 0.15f;
    } else if (activeSensors.mag_type == MAG_AK09911) {
      writeReg(activeSensors.mag_addr, 0x31, 0x00); delay(3); writeReg(activeSensors.mag_addr, 0x31, 0x04); delay(3);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 0.6f;
    } else if (activeSensors.mag_type == MAG_HSCDTD008A) {
      writeReg(activeSensors.mag_addr, 0x1E, 0x10); delay(2); writeReg(activeSensors.mag_addr, 0x1B, 0x80); delay(3);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 0.15f;
    } else if (activeSensors.mag_type == MAG_QMC5883L) {
      writeReg(activeSensors.mag_addr, 0x0B, 0x01); delay(2); writeReg(activeSensors.mag_addr, 0x09, 0x0D); delay(2);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 100.0f / 12000.0f;
    } else if (activeSensors.mag_type == MAG_HMC5883L) {
      writeReg(activeSensors.mag_addr, 0x00, 0x78); writeReg(activeSensors.mag_addr, 0x01, 0x20); writeReg(activeSensors.mag_addr, 0x02, 0x00); delay(6);
      magCal.scale_x = magCal.scale_y = 100.0f / 1090.0f; magCal.scale_z = 100.0f / 980.0f;
    }
  }

  static void bindAndInitMag(uint8_t addr, MagType t) { activeSensors.mag_addr = addr; activeSensors.mag_type = t; activeSensors.mag_found = true; initMagHardware(); }

  bool readMagUnified(float &mx_u, float &my_u, float &mz_u) {
    if (!activeSensors.mag_found) return false;
    uint8_t buf[6]; float x = 0.0f, y = 0.0f, z = 0.0f;
    if (activeSensors.mag_type == MAG_AK8963) {
      uint8_t st1 = 0, st2 = 0;
      if (!readReg(activeSensors.mag_addr, 0x02, &st1, 1) || !(st1 & 0x01) || !readReg(activeSensors.mag_addr, 0x03, buf, 6)) return false;
      readReg(activeSensors.mag_addr, 0x09, &st2, 1); if (st2 & 0x08) return false;
      x = (float)((int16_t)(buf[1] << 8 | buf[0])) * magCal.asa_x * magCal.scale_x;
      y = (float)((int16_t)(buf[3] << 8 | buf[2])) * magCal.asa_y * magCal.scale_y;
      z = (float)((int16_t)(buf[5] << 8 | buf[4])) * magCal.asa_z * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_AK09911) {
      uint8_t st1 = 0, meas[8];
      if (!readReg(activeSensors.mag_addr, 0x10, &st1, 1) || !(st1 & 0x01) || !readReg(activeSensors.mag_addr, 0x11, meas, 8) || (meas[7] & 0x08)) return false;
      x = (float)((int16_t)(meas[1] << 8 | meas[0])) * magCal.scale_x; y = (float)((int16_t)(meas[3] << 8 | meas[2])) * magCal.scale_y; z = (float)((int16_t)(meas[5] << 8 | meas[4])) * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_HSCDTD008A) {
      uint8_t stat = 0; if (!readReg(activeSensors.mag_addr, 0x18, &stat, 1) || !readReg(activeSensors.mag_addr, 0x10, buf, 6)) return false;
      x = (float)((int16_t)(buf[1] << 8 | buf[0])) * magCal.scale_x; y = (float)((int16_t)(buf[3] << 8 | buf[2])) * magCal.scale_y; z = (float)((int16_t)(buf[5] << 8 | buf[4])) * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_QMC5883L) {
      uint8_t st = 0; if (!readReg(activeSensors.mag_addr, 0x06, &st, 1) || !(st & 0x01) || !readReg(activeSensors.mag_addr, 0x00, buf, 6)) return false;
      x = (float)((int16_t)(buf[1] << 8 | buf[0])) * magCal.scale_x; y = (float)((int16_t)(buf[3] << 8 | buf[2])) * magCal.scale_y; z = (float)((int16_t)(buf[5] << 8 | buf[4])) * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_HMC5883L) {
      if (!readReg(activeSensors.mag_addr, 0x03, buf, 6)) return false;
      x = (float)((int16_t)(buf[0] << 8 | buf[1])) * magCal.scale_x; y = (float)((int16_t)(buf[4] << 8 | buf[5])) * magCal.scale_y; z = (float)((int16_t)(buf[2] << 8 | buf[3])) * magCal.scale_z;
    } else return false;
    rotateXYByYawAlign(x, y, currentConfig.com_align);
    mx_u = x; my_u = y; mz_u = z; return true;
  }

  bool readCompassHeadingDeg(float &heading_deg) {
    if (!currentConfig.com_en || !activeSensors.mag_found) return false;
    float mx_u, my_u, mz_u;
    if (!readMagUnified(mx_u, my_u, mz_u)) return false;
    float x = mx_u - currentConfig.mag_bias_x, y = my_u - currentConfig.mag_bias_y, z = mz_u - currentConfig.mag_bias_z;
    float xh = x, yh = y;
    if (activeSensors.imu_found && !err_imu_fail) {
      float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
      float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
      float pitch = asinf(constrainF(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f));
      xh = x * cosf(pitch) + y * sinf(roll) * sinf(pitch) + z * cosf(roll) * sinf(pitch);
      yh = y * cosf(roll) - z * sinf(roll);
    }
    float h = atan2f(yh, xh) * FC_RAD_TO_DEG;
    if (!isfinite(h)) { setFcErrorRam(FC_ERR_NAN_INF_MATH); return false; }
    heading_deg = wrapDeg0To360(h); return true;
  }

  void detectMag() {
    uint8_t id = 0, req = currentConfig.com_id;
    activeSensors.mag_found = false; activeSensors.mag_type = MAG_UNKNOWN;
    if (activeSensors.imu_found && (activeSensors.imu_type == IMU_MPU9250 || activeSensors.imu_type == IMU_MPU6500)) enableImuI2cBypass();
    if ((req == MAG_AK8963 || req == MAG_UNKNOWN) && readReg(0x0C, 0x00, &id, 1) && id == 0x48) { bindAndInitMag(0x0C, MAG_AK8963); return; }
    if (req == MAG_AK09911 || req == MAG_UNKNOWN) {
      const uint8_t a2[2] = {0x0C, 0x0E};
      for (uint8_t i = 0; i < 2; i++) { uint8_t wia[2] = {0, 0}; if (readReg(a2[i], 0x00, wia, 2) && wia[0] == 0x48 && wia[1] == 0x05) { bindAndInitMag(a2[i], MAG_AK09911); return; } }
    }
    if (req == MAG_HSCDTD008A || req == MAG_UNKNOWN) {
      const uint8_t a2[2] = {0x0C, 0x0F};
      for (uint8_t i = 0; i < 2; i++) if (readReg(a2[i], 0x0F, &id, 1) && id == 0x49) { bindAndInitMag(a2[i], MAG_HSCDTD008A); return; }
    }
    if ((req == MAG_QMC5883L || req == MAG_UNKNOWN) && readReg(0x0D, 0x0D, &id, 1) && id != 0x00) { bindAndInitMag(0x0D, MAG_QMC5883L); return; }
    if (req == MAG_HMC5883L || req == MAG_UNKNOWN) {
      uint8_t hid[3] = {0}; if (readReg(0x1E, 0x0A, hid, 3) && hid[0] == 'H') { bindAndInitMag(0x1E, MAG_HMC5883L); return; }
    }
    if (currentConfig.com_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_MAG_FAIL); }
  }

  void detectLidar() {
    uint8_t req = currentConfig.lid_id;
    activeSensors.lidar_found = false; activeSensors.lidar_type = LIDAR_NONE;
    checkBusAndRecover();
    if (!i2c1WriteRead(0x29, nullptr, 0, nullptr, 0)) {
      if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); }
      return;
    }
    uint8_t id0 = 0; bool is0x = (readReg(0x29, 0xC0, &id0, 1) && (id0 == 0xEE || id0 == 0xAA));
    uint8_t mid[2] = {0, 0}; bool has_m = readReg16Addr(0x29, 0x010F, mid, 2);
    if (req == LIDAR_VL53L0X && is0x) { activeSensors.lidar_type = LIDAR_VL53L0X; activeSensors.lidar_max_m = 2.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_VL53L1X && has_m && mid[0] == 0xEA) { activeSensors.lidar_type = LIDAR_VL53L1X; activeSensors.lidar_max_m = 4.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_VL53L3CX && has_m && (mid[0] == 0xEB || mid[0] == 0xED) && mid[1] <= 0x40) { activeSensors.lidar_type = LIDAR_VL53L3CX; activeSensors.lidar_max_m = 3.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_VL53L4CX && has_m && (mid[0] == 0xEB || mid[0] == 0xED) && mid[1] > 0x40) { activeSensors.lidar_type = LIDAR_VL53L4CX; activeSensors.lidar_max_m = 6.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_NONE) {
      if (is0x) { activeSensors.lidar_type = LIDAR_VL53L0X; activeSensors.lidar_max_m = 2.0f; activeSensors.lidar_found = true; return; }
      if (has_m && mid[0] == 0xEA) { activeSensors.lidar_type = LIDAR_VL53L1X; activeSensors.lidar_max_m = 4.0f; activeSensors.lidar_found = true; return; }
      if (has_m && (mid[0] == 0xEB || mid[0] == 0xED)) {
        activeSensors.lidar_type = (mid[1] > 0x40) ? LIDAR_VL53L4CX : LIDAR_VL53L3CX;
        activeSensors.lidar_max_m = (activeSensors.lidar_type == LIDAR_VL53L4CX) ? 6.0f : 3.0f; activeSensors.lidar_found = true; return;
      }
    }
    if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); }
  }

  void initLidarHardware() {
    if (!activeSensors.lidar_found || !currentConfig.lidar_en) return;
    checkBusAndRecover();
    if (activeSensors.lidar_type == LIDAR_VL53L0X) { writeReg(0x29, 0x00, 0x01); delay(10); }
    else {
      writeReg16Addr8Val(0x29, 0x0000, 0x00); delay(10);
      writeReg16Addr8Val(0x29, 0x007F, 0x7F); writeReg16Addr8Val(0x29, 0x0080, 0x77);
      writeReg16Addr8Val(0x29, 0x0002, 0x00); writeReg16Addr8Val(0x29, 0x0086, 0x40);
    }
  }

  bool readRawLidarSlantMeters(float &slant_m) {
    if (!activeSensors.lidar_found) return false;
    uint16_t dist_mm = 0;
    if (activeSensors.lidar_type == LIDAR_VL53L0X) {
      uint8_t d[3] = {0};
      if (!readReg(0x29, 0x13, d, 3)) { if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); } return false; }
      uint8_t st = (d[0] >> 3) & 0x0F; if (st != 0 && st != 11) return false;
      dist_mm = ((uint16_t)d[1] << 8) | d[2];
    } else {
      uint8_t blk[15] = {0};
      if (!readReg16Addr(0x29, 0x0089, blk, 15)) { if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); } return false; }
      uint8_t st = blk[0] & 0x1F; if (st != 0 && st != 9) return false;
      dist_mm = ((uint16_t)blk[13] << 8) | blk[14];
    }
    if (dist_mm == 0 || dist_mm > (uint16_t)(getEffectiveLidarMaxM() * 1000.0f)) return false;
    slant_m = (float)dist_mm / 1000.0f; return true;
  }

  float computeCorrectedLidarMeters(float slant_m) {
    float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
    float vf = (2.0f * (q1 * q3 - q0 * q2) * SIN_45) + (((q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3)) * COS_45);
    if (vf < 0.15f) vf = 0.15f;
    float off_m = constrainF(currentConfig.lidar_offset_m, CFG_LIDAR_OFFSET_MIN, CFG_LIDAR_OFFSET_MAX);
    float c = (slant_m * vf) + off_m;
    return (c < 0.0f) ? 0.0f : c;
  }

  bool readLidar(float &dist_m) {
    if (!currentConfig.lidar_en || !activeSensors.lidar_found) return false;
    float sm = 0.0f; if (!readRawLidarSlantMeters(sm)) return false;
    dbg_lidar_m = adaptive_lidar.update(computeCorrectedLidarMeters(sm), currentConfig.lidar_filter_alpha);
    dist_m = dbg_lidar_m; last_lidar_packet_ms = millis(); return true;
  }

  void calibrateLidar() {
    if (!activeSensors.lidar_found) { uart1.println("ERR: lidar"); return; }
    float sum = 0.0f; uint8_t ok = 0;
    for (uint8_t i = 0; i < 20; i++) { float sm = 0.0f; if (readRawLidarSlantMeters(sm)) { sum += sm; ok++; } delay(15); }
    if (ok > 0) {
      currentConfig.lidar_offset_m = constrainF(-(sum / (float)ok), CFG_LIDAR_OFFSET_MIN, CFG_LIDAR_OFFSET_MAX);
      adaptive_lidar.reset(); saveSettings(); uart1.println("OK: cal_lid");
    } else uart1.println("ERR: cal_lid");
  }

  void calibrateBatVoltage(float actual_v) {
    if (!isfinite(actual_v) || actual_v < 1.0f || actual_v > 60.0f) { uart1.println("ERR: cal_bat"); return; }
    uint32_t sum = 0; for (uint8_t i = 0; i < 32; i++) { sum += adc1ReadChannel(4); delay(2); }
    float pv = (((float)sum / 32.0f) / 4095.0f) * 3.3f;
    if (pv < 0.01f) { uart1.println("ERR: adc"); return; }
    currentConfig.bat_scale_v = constrainF(actual_v / pv, CFG_BAT_SCALE_V_MIN, CFG_BAT_SCALE_V_MAX);
    v_batt = lpf_battery.output = actual_v; batt_valid = true; power_w = v_batt * current_a;
    saveSettings(); uart1.println("OK: cal_bat");
  }

  void calibrateCurrent(bool has_val, float actual_a) {
    uint32_t sum = 0; for (uint8_t i = 0; i < 32; i++) { sum += adc1ReadChannel(5); delay(2); }
    float sa = ((((float)sum / 32.0f) / 4095.0f) * 3.3f) / SHUNT_RESISTANCE_OHM;
    if (!has_val || fabsf(actual_a) < 0.001f) {
      currentConfig.bat_offset_a = constrainF(sa, CFG_BAT_OFFSET_A_MIN, CFG_BAT_OFFSET_A_MAX);
      current_a = power_w = lpf_current.output = 0.0f;
      saveSettings(); uart1.println("OK: cal_curr"); return;
    }
    if (!isfinite(actual_a) || actual_a < 0.0f || actual_a > 250.0f) { uart1.println("ERR: cal_curr"); return; }
    float da = sa - currentConfig.bat_offset_a;
    if (fabsf(da) < 0.01f) { uart1.println("ERR: cal_curr"); return; }
    currentConfig.bat_scale_a = constrainF(actual_a / da, CFG_BAT_SCALE_A_MIN, CFG_BAT_SCALE_A_MAX);
    current_a = lpf_current.output = actual_a; power_w = v_batt * current_a;
    saveSettings(); uart1.println("OK: cal_curr");
  }

  bool calibrateGyro(bool silent) {
    if (!activeSensors.imu_found) { if (!silent) uart1.println("ERR: imu"); return false; }
    float sgx = 0.0f, sgy = 0.0f, sgz = 0.0f, st = 0.0f; uint16_t ok = 0;
    for (uint16_t i = 0; i < 200; i++) {
      uint8_t b[14];
      if (readMPU6050(b)) {
        float gx = (float)((int16_t)(b[8] << 8 | b[9])) / activeSensors.gyro_scale;
        float gy = (float)((int16_t)(b[10] << 8 | b[11])) / activeSensors.gyro_scale;
        float gz = (float)((int16_t)(b[12] << 8 | b[13])) / activeSensors.gyro_scale;
        rotateXYByYawAlign(gx, gy, currentConfig.imu_align);
        sgx += gx; sgy += gy; sgz += gz; st += extractTempFromImuBuf(b); ok++;
      }
      delay(4);
    }
    if (ok == 0) { if (!silent) uart1.println("ERR: cal_gyro"); return false; }
    currentConfig.gyro_bias_x = sgx / (float)ok; currentConfig.gyro_bias_y = sgy / (float)ok;
    currentConfig.gyro_bias_z = sgz / (float)ok; currentConfig.base_temperature = st / (float)ok;
    if (!silent) { saveSettings(); uart1.println("OK: cal_gyro"); }
    return true;
  }

  bool calibrateAcc(bool silent) {
    if (!activeSensors.imu_found) { if (!silent) uart1.println("ERR: imu"); return false; }
    float sax = 0.0f, say = 0.0f, saz = 0.0f; uint16_t ok = 0;
    for (uint16_t i = 0; i < 200; i++) {
      uint8_t b[14];
      if (readMPU6050(b)) {
        float ax = (float)((int16_t)(b[0] << 8 | b[1])) / activeSensors.acc_scale;
        float ay = (float)((int16_t)(b[2] << 8 | b[3])) / activeSensors.acc_scale;
        float az = (float)((int16_t)(b[4] << 8 | b[5])) / activeSensors.acc_scale;
        rotateXYByYawAlign(ax, ay, currentConfig.imu_align);
        sax += ax; say += ay; saz += az; ok++;
      }
      delay(4);
    }
    if (ok == 0) { if (!silent) uart1.println("ERR: cal_acc"); return false; }
    currentConfig.acc_bias_x = sax / (float)ok; currentConfig.acc_bias_y = say / (float)ok; currentConfig.acc_bias_z = (saz / (float)ok) - 1.0f;
    if (!silent) { saveSettings(); uart1.println("OK: cal_acc"); }
    return true;
  }

  void calibrateIMU() {
    if (!activeSensors.imu_found) { uart1.println("ERR: imu"); return; }
    if (!calibrateGyro(true) || !calibrateAcc(true)) { uart1.println("ERR: cal_imu"); return; }
    saveSettings(); uart1.println("OK: cal_imu");
  }

  void autoDetectAll() {
    err_aux_i2c_fail = false; detectIMU();
    if (currentConfig.baro_en) detectBaro(); else { activeSensors.baro_found = false; activeSensors.baro_type = BARO_UNKNOWN; }
    if (currentConfig.com_en) detectMag(); else { activeSensors.mag_found = false; activeSensors.mag_type = MAG_UNKNOWN; }
    if (currentConfig.lidar_en) { detectLidar(); initLidarHardware(); } else { activeSensors.lidar_found = false; activeSensors.lidar_type = LIDAR_NONE; }
  }
} // namespace SensorDriver

void startMagCalibrationNonBlocking() {
  if (!SensorDriver::activeSensors.mag_found) { uart1.println("ERR: mag"); return; }
  mag_cal_active = true; mag_cal_start_ms = millis(); mag_cal_last_sample_ms = 0;
  mag_cal_min_x = mag_cal_min_y = mag_cal_min_z = 1e9f;
  mag_cal_max_x = mag_cal_max_y = mag_cal_max_z = -1e9f;
  uart1.println("OK: cal_mag");
}

void stepMagCalibrationNonBlocking(uint32_t now_ms) {
  if (!mag_cal_active) return;
  if ((now_ms - mag_cal_last_sample_ms) >= MAG_CAL_SAMPLE_MS) {
    mag_cal_last_sample_ms = now_ms;
    float mx, my, mz;
    if (SensorDriver::readMagUnified(mx, my, mz)) {
      if (mx < mag_cal_min_x) mag_cal_min_x = mx;
      if (mx > mag_cal_max_x) mag_cal_max_x = mx;
      if (my < mag_cal_min_y) mag_cal_min_y = my;
      if (my > mag_cal_max_y) mag_cal_max_y = my;
      if (mz < mag_cal_min_z) mag_cal_min_z = mz;
      if (mz > mag_cal_max_z) mag_cal_max_z = mz;
    }
  }
  if ((now_ms - mag_cal_start_ms) >= MAG_CAL_DURATION_MS) {
    mag_cal_active = false;
    if (mag_cal_max_x > mag_cal_min_x && mag_cal_max_y > mag_cal_min_y && mag_cal_max_z > mag_cal_min_z) {
      currentConfig.mag_bias_x = constrainF(0.5f * (mag_cal_min_x + mag_cal_max_x), CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
      currentConfig.mag_bias_y = constrainF(0.5f * (mag_cal_min_y + mag_cal_max_y), CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
      currentConfig.mag_bias_z = constrainF(0.5f * (mag_cal_min_z + mag_cal_max_z), CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
      saveSettings(); uart1.println("DONE: cal_mag");
    } else uart1.println("ERR: cal_mag");
  }
}

void stepMonitorStreamNonBlocking(uint32_t now_ms) {
  if (!mon_stream_active || (now_ms - mon_last_send_ms) < MON_STREAM_PERIOD_MS) return;
  mon_last_send_ms = now_ms;
  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float r = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * FC_RAD_TO_DEG;
  float p = asinf(constrainF(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * FC_RAD_TO_DEG;
  uart1.print("MON: err="); uart1.print((int)getActiveStatusErrorCode());
  uart1.print(" arm="); uart1.print(is_armed ? 1 : 0);
  uart1.print(" fs="); uart1.print(is_failsafe ? 1 : 0);
  uart1.print(" cpu="); uart1.print(cpu_load_pct, 1); uart1.print("/"); uart1.print(cpu_max_load_pct, 1);
  uart1.print(" vib="); uart1.print(vibration_index, 1);
  uart1.print(" rp="); uart1.print(r, 1); uart1.print(","); uart1.print(p, 1);
  uart1.print(" alt="); uart1.print(baro_alt_m, 2); uart1.print(" lid="); uart1.print(dbg_lidar_m, 2);
  uart1.print(" v="); uart1.print(v_batt, 2); uart1.print(" i="); uart1.print(current_a, 2);
  uart1.print(" lq="); uart1.println((int)current_link_quality);
}

void readBatteryAndCurrent() {
  float iv = (((float)adc1ReadChannel(4) / 4095.0f) * 3.3f) * SensorDriver::getBatScaleV();
  if (!isfinite(iv) || iv < 0.0f || iv > 60.0f) {
    batt_valid = err_batt_low = false; setFcErrorRam(FC_ERR_BAT_SENSOR_FAIL); return;
  }
  if (iv > 0.5f) { if (!batt_valid) lpf_battery.output = iv; batt_valid = true; }
  v_batt = lpf_battery.update(iv);
  current_a = lpf_current.update(SensorDriver::computeCurrentAmpsFromAdc(adc1ReadChannel(5)));
  power_w = v_batt * current_a;
  if (is_armed) consumed_wh += (power_w * DT) / 3600.0f;
  if (currentConfig.fs_bat_en && batt_valid && v_batt < currentConfig.bat_fs_v) { err_batt_low = true; setFcErrorRam(FC_ERR_BAT_LOW); }
  else err_batt_low = false;
}

void updateAuxSensorsNonBlocking(uint32_t now_ms) {
  static uint32_t last_aux_ms = 0, last_bvz_ms = 0;
  float alt_m = 0.0f;
  if (SensorDriver::stepBaroNonBlocking(now_ms, alt_m)) {
    baro_alt_m = alt_m;
    if (last_bvz_ms > 0 && (now_ms - last_bvz_ms) >= 10) {
      baro_vz_mps = lpf_baro_vz.update(constrainF((baro_alt_m - lpf_baro_alt.output) / ((float)(now_ms - last_bvz_ms) * 0.001f), -4.0f, 4.0f));
    }
    last_bvz_ms = now_ms;
  }
  if ((now_ms - last_aux_ms) < AUX_SENSOR_PERIOD_MS) return;
  last_aux_ms = now_ms;
  float lm = 0.0f;
  if (SensorDriver::readLidar(lm)) current_vz = constrainF((lm - adaptive_lidar.output) / (AUX_SENSOR_PERIOD_MS * 0.001f), -4.0f, 4.0f);
  float hd = 0.0f;
  if (SensorDriver::readCompassHeadingDeg(hd)) { compass_heading_deg = hd; compass_valid = true; }
  else compass_valid = false;
  readBatteryAndCurrent();
}

// ==============================================================================
// 10. АВТОКАЛИБРОВКА, INFO И CLI (USART1: PA9 TX / PA10 RX)
// ==============================================================================
namespace CliAutoCal {
  constexpr uint8_t BTN_PIN = 8;
  constexpr uint32_t BTN_DEBOUNCE_MS = 25, BTN_HOLD_MS = 1200;
  constexpr float TARGET_ALT_M = 1.10f, TAKEOFF_VZ_MPS = 0.40f, LANDING_VZ_MPS = -0.35f;
  constexpr float GROUND_EFFECT_M = 0.35f, TOUCHDOWN_M = 0.07f, TILT_LIMIT_DEG = 45.0f, SHOCK_G_DELTA = 1.35f;
  constexpr uint32_t HOVER_CAL_MS = 5000, SPIN_RAMP_MS = 1500, PHASE_TIMEOUT_MS = 12000;

  enum State : uint8_t { IDLE = 0, PRECHECK, CAL_GROUND, SPIN_RAMP, TAKEOFF, HOVER_CAL, LANDING, SAVE_DONE, ABORT };
  enum AbortReason : uint8_t { REASON_NONE = 0, REASON_USER_CANCEL, REASON_ARMED_CONFLICT, REASON_SENSOR_FAIL, REASON_BAT_LOW, REASON_TILT_DETECTED, REASON_TIMEOUT };

  static bool btn_raw = false, btn_deb = false, btn_hold = false, flight_active = false, hazard = false, pid_step = false;
  static uint32_t btn_deb_ms = 0, btn_press_ms = 0, state_start_ms = 0, lidar_prev_ms = 0;
  static State state = IDLE;
  static AbortReason abort_r = REASON_NONE;
  static float hover_sum = 0.0f, prev_r_kp = 0.0f, prev_p_kp = 0.0f, prev_r_kd = 0.0f, prev_p_kd = 0.0f, peak_vib = 0.0f, lidar_prev_m = 0.0f, lidar_vz = 0.0f;
  static uint16_t hover_cnt = 0;

  static void restoreTempPid() {
    if (!pid_step) return;
    currentConfig.roll_pid.kp = prev_r_kp; currentConfig.pitch_pid.kp = prev_p_kp;
    currentConfig.roll_pid.kd = prev_r_kd; currentConfig.pitch_pid.kd = prev_p_kd;
    updatePidTunings(); pid_step = false;
  }

  static void enterState(State s, uint32_t now_ms) { state = s; state_start_ms = now_ms; peak_vib = 0.0f; }

  void stop(uint8_t reason_code, uint8_t fc_err_code) {
    restoreTempPid();
    abort_r = (AbortReason)reason_code;
    flight_active = is_armed = false; arm_ramp_factor = 0.0f;
    vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
    resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
    if (fc_err_code != FC_ERR_NONE) setFcErrorRam(fc_err_code);
    state = (abort_r == REASON_USER_CANCEL) ? IDLE : ABORT;
  }

  void start() {
    uint32_t now_ms = millis();
    if (is_armed) { stop(REASON_ARMED_CONFLICT, FC_ERR_NONE); return; }
    stopBenchTest();
    hazard = flight_active = false; abort_r = REASON_NONE; hover_sum = 0.0f; hover_cnt = 0;
    lidar_prev_m = lidar_vz = 0.0f; lidar_prev_ms = now_ms;
    restoreTempPid();
    vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
    enterState(PRECHECK, now_ms);
  }

  void init() {
    gpioConfigPin(Reg::GPIOB, BTN_PIN, GPIO_IN_PULL, true);
    bool p = !gpioRead(Reg::GPIOB, BTN_PIN); uint32_t now = millis();
    btn_raw = btn_deb = p; btn_deb_ms = now; btn_press_ms = p ? now : 0; btn_hold = false;
  }

  void pollButton(uint32_t now_ms) {
    bool r = !gpioRead(Reg::GPIOB, BTN_PIN);
    if (r != btn_raw) { btn_raw = r; btn_deb_ms = now_ms; }
    if ((now_ms - btn_deb_ms) >= BTN_DEBOUNCE_MS && r != btn_deb) { btn_deb = r; btn_press_ms = r ? now_ms : 0; btn_hold = false; }
    if (btn_deb && !btn_hold && (now_ms - btn_press_ms) >= BTN_HOLD_MS) {
      btn_hold = true;
      if (state != IDLE && state != ABORT && state != SAVE_DONE) { stop(REASON_USER_CANCEL, FC_ERR_NONE); uart1.println("OK: auto_stop"); }
      else { start(); uart1.println("OK: auto_cal"); }
    }
  }

  bool isFlightActive() { return flight_active; }
  void onDisarmRequested() { if (flight_active) stop(REASON_USER_CANCEL, FC_ERR_NONE); }
  void onArmRequested() { if (state != IDLE) stop(REASON_ARMED_CONFLICT, FC_ERR_NONE); }

  void onAttitudeAndMotion(float ax_g, float ay_g, float az_g, float roll_deg, float pitch_deg, float gx_dps, float gy_dps) {
    float shock = fabsf(sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g) - 1.0f);
    float tilt = fmaxf(fabsf(roll_deg), fabsf(pitch_deg));
    float gxy = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps);
    if ((is_armed || flight_active) && (tilt >= TILT_LIMIT_DEG || (tilt >= 32.0f && gxy >= 260.0f) || shock >= SHOCK_G_DELTA)) {
      hazard = true; setFcErrorRam(FC_ERR_TILT_DISARM); is_armed = false; arm_ramp_factor = 0.0f;
      resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
      if (state != IDLE && state != ABORT && state != SAVE_DONE) stop(REASON_TILT_DETECTED, FC_ERR_TILT_DISARM);
    }
  }

  static bool readFreshLidar(uint32_t now_ms, float &d) {
    if (!currentConfig.lidar_en || !SensorDriver::activeSensors.lidar_found) return false;
    if ((now_ms - last_lidar_packet_ms) > 250 || !isfinite(dbg_lidar_m) || dbg_lidar_m < 0.0f) return false;
    if (dbg_lidar_m > SensorDriver::getEffectiveLidarMaxM()) return false;
    d = dbg_lidar_m; return true;
  }

  static void updateLidarVz(uint32_t now_ms, float lm) {
    if (lidar_prev_ms == 0) { lidar_prev_m = lm; lidar_prev_ms = now_ms; lidar_vz = 0.0f; return; }
    uint32_t dt = now_ms - lidar_prev_ms;
    if (dt >= 10) {
      lidar_vz = 0.35f * constrainF((lm - lidar_prev_m) / ((float)dt * 0.001f), -3.5f, 3.5f) + 0.65f * lidar_vz;
      lidar_prev_m = lm; lidar_prev_ms = now_ms;
    }
  }

  bool stepFlight(uint32_t now_ms, float gx_dps, float gy_dps, float gz_dps,
                  float gx_rad, float gy_rad, float gz_rad,
                  float roll_deg, float pitch_deg,
                  float &m1, float &m2, float &m3, float &m4) {
    (void)gz_dps;
    if (state == IDLE || state == ABORT || state == SAVE_DONE) return false;
    if (hazard) { stop(REASON_TILT_DETECTED, FC_ERR_TILT_DISARM); m1 = m2 = m3 = m4 = 0.0f; return true; }
    if (currentConfig.fs_bat_en && batt_valid && v_batt < currentConfig.bat_fs_v) { stop(REASON_BAT_LOW, FC_ERR_BAT_LOW); m1 = m2 = m3 = m4 = 0.0f; return true; }

    float lm = 0.0f; bool lok = readFreshLidar(now_ms, lm);
    if (lok) updateLidarVz(now_ms, lm);
    if (vibration_index > peak_vib) peak_vib = vibration_index;
    uint32_t el = now_ms - state_start_ms;

    if (state == PRECHECK) {
      m1 = m2 = m3 = m4 = 0.0f;
      if (!SensorDriver::activeSensors.imu_found || err_imu_fail) { stop(REASON_SENSOR_FAIL, FC_ERR_IMU_FAIL); return true; }
      if (!lok && el >= 2000) { stop(REASON_SENSOR_FAIL, FC_ERR_LIDAR_FAIL); return true; }
      if (lok) enterState(CAL_GROUND, now_ms);
      return true;
    }
    if (state == CAL_GROUND) {
      m1 = m2 = m3 = m4 = 0.0f;
      if (!lok) { stop(REASON_SENSOR_FAIL, FC_ERR_LIDAR_FAIL); return true; }
      if (!SensorDriver::calibrateGyro(true) || !SensorDriver::calibrateAcc(true)) { stop(REASON_SENSOR_FAIL, FC_ERR_IMU_FAIL); return true; }
      currentConfig.lidar_offset_m = constrainF(-lm, CFG_LIDAR_OFFSET_MIN, CFG_LIDAR_OFFSET_MAX);
      adaptive_lidar.reset();
      orientation = Quaternion();
      vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
      hover_sum = 0.0f; hover_cnt = 0; flight_active = true;
      enterState(SPIN_RAMP, now_ms);
      return true;
    }
    if (!lok) {
      if (state == TAKEOFF || state == HOVER_CAL) enterState(LANDING, now_ms);
      else if (el >= 1500) { stop(REASON_SENSOR_FAIL, FC_ERR_LIDAR_FAIL); return true; }
    }
    if (state == SPIN_RAMP) {
      float rt = base_hover_throttle * 0.45f * constrainF((float)el / (float)SPIN_RAMP_MS, 0.0f, 1.0f);
      float rc = rollRatePID.update(-gx_rad, gx_rad, DT), pc = pitchRatePID.update(-gy_rad, gy_rad, DT), yc = yawRatePID.update(-gz_rad, gz_rad, DT);
      m1 = constrainF(rt + pc + rc - yc, 0.0f, 450.0f); m2 = constrainF(rt + pc - rc + yc, 0.0f, 450.0f);
      m3 = constrainF(rt - pc - rc - yc, 0.0f, 450.0f); m4 = constrainF(rt - pc + rc + yc, 0.0f, 450.0f);
      if (el >= SPIN_RAMP_MS) enterState(TAKEOFF, now_ms);
      return true;
    }

    float dvz = 0.0f;
    if (state == TAKEOFF) {
      dvz = constrainF((TARGET_ALT_M - lm) * 1.2f, LANDING_VZ_MPS, TAKEOFF_VZ_MPS);
      if (lok && lm >= (TARGET_ALT_M - 0.08f) && fabsf(lidar_vz) < 0.25f) enterState(HOVER_CAL, now_ms);
      else if (el >= PHASE_TIMEOUT_MS) enterState(LANDING, now_ms);
    } else if (state == HOVER_CAL) {
      dvz = constrainF((TARGET_ALT_M - lm) * 1.1f, -0.45f, 0.45f);
      if (el < (HOVER_CAL_MS / 2)) {
        if (!pid_step) {
          prev_r_kp = currentConfig.roll_pid.kp; prev_p_kp = currentConfig.pitch_pid.kp;
          prev_r_kd = currentConfig.roll_pid.kd; prev_p_kd = currentConfig.pitch_pid.kd;
          currentConfig.roll_pid.kp = constrainF(currentConfig.roll_pid.kp * 1.12f, 0.0f, CFG_RATE_KP_MAX);
          currentConfig.pitch_pid.kp = constrainF(currentConfig.pitch_pid.kp * 1.12f, 0.0f, CFG_RATE_KP_MAX);
          currentConfig.roll_pid.kd = constrainF(currentConfig.roll_pid.kd * 1.10f, 0.0f, CFG_RATE_KD_MAX);
          currentConfig.pitch_pid.kd = constrainF(currentConfig.pitch_pid.kd * 1.10f, 0.0f, CFG_RATE_KD_MAX);
          updatePidTunings(); pid_step = true; peak_vib = vibration_index;
        }
      } else if (pid_step) {
        if (peak_vib > 35.0f) restoreTempPid(); else pid_step = false;
      }
      if (el >= HOVER_CAL_MS) {
        if (hover_cnt > 0) { float ah = hover_sum / (float)hover_cnt; if (ah > 50.0f && ah < 850.0f) base_hover_throttle = ah; }
        restoreTempPid(); enterState(LANDING, now_ms);
      }
    } else if (state == LANDING) {
      dvz = LANDING_VZ_MPS * ((lok && lm < GROUND_EFFECT_M) ? 0.55f : 1.0f);
      if (lok && lm <= TOUCHDOWN_M && fabsf(lidar_vz) < 0.30f) {
        stop(REASON_NONE, FC_ERR_NONE); saveSettings(); state = SAVE_DONE;
        m1 = m2 = m3 = m4 = 0.0f; return true;
      }
      if (el >= PHASE_TIMEOUT_MS) { stop(REASON_TIMEOUT, FC_ERR_NONE); m1 = m2 = m3 = m4 = 0.0f; return true; }
    }

    float vm = lok ? lidar_vz : baro_vz_mps;
    float tc = constrainF(base_hover_throttle + vzPID.update(dvz - vm, vm, DT), 40.0f, 850.0f);
    if (state == HOVER_CAL && lok && fabsf(TARGET_ALT_M - lm) < 0.15f && fabsf(vm) < 0.20f) { hover_sum += tc; hover_cnt++; }

    float tgx = constrainF(constrainF(-roll_deg, -currentConfig.fm_roll_max, currentConfig.fm_roll_max) * 2.5f * FC_DEG_TO_RAD, -3.0f, 3.0f);
    float tgy = constrainF(constrainF(-pitch_deg, -currentConfig.fm_pitch_max, currentConfig.fm_pitch_max) * 2.5f * FC_DEG_TO_RAD, -3.0f, 3.0f);
    float rc = rollRatePID.update(tgx - gx_rad, gx_rad, DT), pc = pitchRatePID.update(tgy - gy_rad, gy_rad, DT), yc = yawRatePID.update(-gz_rad, gz_rad, DT);

    m1 = constrainF(tc + pc + rc - yc, 0.0f, OUTPUT_MAX); m2 = constrainF(tc + pc - rc + yc, 0.0f, OUTPUT_MAX);
    m3 = constrainF(tc - pc - rc - yc, 0.0f, OUTPUT_MAX); m4 = constrainF(tc - pc + rc + yc, 0.0f, OUTPUT_MAX);
    return true;
  }
} // namespace CliAutoCal

static const char* skipSpaces(const char* s) {
  while (s && (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')) s++;
  return s;
}

static bool parseNumberStrict(const char* s, float &out) {
  s = skipSpaces(s);
  if (!s || *s == '\0') return false;
  char* ep = nullptr;
  float v = (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) ? (float)strtoul(s, &ep, 16) : strtof(s, &ep);
  if (ep == s || !isfinite(v)) return false;
  ep = (char*)skipSpaces(ep);
  if (*ep != '\0') return false;
  out = v; return true;
}

static void printSystemInfo() {
  sanitizeConfig();
  uart1.print("INFO: err="); uart1.print((int)getActiveStatusErrorCode());
  uart1.print(" arm="); uart1.print(is_armed ? 1 : 0);
  uart1.print(" fs="); uart1.print(is_failsafe ? 1 : 0);
  uart1.print(" fm="); uart1.print((int)currentConfig.fm_mode);
  uart1.print(" esc="); uart1.print((int)currentConfig.esc_prot);
  uart1.print(" lim="); uart1.print(getEffectiveEscDeltaLimit(), 1);
  uart1.print(" hov="); uart1.println(base_hover_throttle, 1);

  uart1.print("CFG_FM: rmax="); uart1.print(currentConfig.fm_roll_max, 1);
  uart1.print(" pmax="); uart1.print(currentConfig.fm_pitch_max, 1);
  uart1.print(" vzmax="); uart1.print(currentConfig.fm_vz_speed_max, 2);
  uart1.print(" fsvz="); uart1.println(currentConfig.fs_vz_speed, 2);

  uart1.print("CFG_RC: r="); uart1.print(currentConfig.rc_rate, 2);
  uart1.print(" se="); uart1.print(currentConfig.rc_super_expo, 2);
  uart1.print(" exp="); uart1.print(currentConfig.rc_stick_expo, 2);
  uart1.print(" yr="); uart1.print(currentConfig.rc_yaw_rate, 2);
  uart1.print(" db="); uart1.print(currentConfig.rc_deadband, 1);
  uart1.print(" sm="); uart1.println(currentConfig.rc_filter_smoothing, 2);

  uart1.print("CFG_FLT: lpf="); uart1.print((int)currentConfig.flt_lpf_en);
  uart1.print(" ga="); uart1.print(currentConfig.gyro_filter_alpha, 3);
  uart1.print(" aa="); uart1.print(currentConfig.acc_filter_alpha, 3);
  uart1.print(" not="); uart1.print((int)currentConfig.flt_not_en);
  uart1.print(" nf="); uart1.print(currentConfig.filter_notch_freq, 1);
  uart1.print(" bw="); uart1.print(currentConfig.filter_notch_bw, 1);
  uart1.print(" amb="); uart1.print((int)currentConfig.flt_amb_en);
  uart1.print(" hi="); uart1.print(currentConfig.flt_amb, 4);
  uart1.print(" lo="); uart1.println(currentConfig.flt_amb_min, 4);

  uart1.print("CFG_PID: r="); uart1.print(currentConfig.roll_pid.kp, 3); uart1.print("/"); uart1.print(currentConfig.roll_pid.ki, 3); uart1.print("/"); uart1.print(currentConfig.roll_pid.kd, 3);
  uart1.print(" p="); uart1.print(currentConfig.pitch_pid.kp, 3); uart1.print("/"); uart1.print(currentConfig.pitch_pid.ki, 3); uart1.print("/"); uart1.print(currentConfig.pitch_pid.kd, 3);
  uart1.print(" y="); uart1.print(currentConfig.yaw_pid.kp, 3); uart1.print("/"); uart1.print(currentConfig.yaw_pid.ki, 3); uart1.print("/"); uart1.print(currentConfig.yaw_pid.kd, 3);
  uart1.print(" vz="); uart1.print(currentConfig.vz_pid.kp, 2); uart1.print("/"); uart1.print(currentConfig.vz_pid.ki, 2); uart1.print("/"); uart1.println(currentConfig.vz_pid.kd, 2);

  uart1.print("CFG_BAT: v="); uart1.print(v_batt, 2);
  uart1.print(" vmax="); uart1.print(getEffectiveBatVMax(), 1);
  uart1.print(" fsv="); uart1.print(currentConfig.bat_fs_v, 1);
  uart1.print(" comp="); uart1.print((int)currentConfig.bat_comp_en);
  uart1.print(" sv="); uart1.print(currentConfig.bat_scale_v, 3);
  uart1.print(" oa="); uart1.print(currentConfig.bat_offset_a, 3);
  uart1.print(" sa="); uart1.print(currentConfig.bat_scale_a, 3);
  uart1.print(" wh="); uart1.println(consumed_wh, 3);

  uart1.print("CFG_HW: imu="); uart1.print((int)SensorDriver::activeSensors.imu_type);
  uart1.print(" bar="); uart1.print((int)SensorDriver::activeSensors.baro_type);
  uart1.print(" mag="); uart1.print((int)SensorDriver::activeSensors.mag_type);
  uart1.print(" lid="); uart1.print((int)SensorDriver::activeSensors.lidar_type);
  uart1.print(" loff="); uart1.print(currentConfig.lidar_offset_m, 3);
  uart1.print(" rtx="); uart1.print((int)currentConfig.rtx_en);
  uart1.print(" ch="); uart1.print((int)active_rtx_chan);
  uart1.print(" vtx="); uart1.print((int)currentConfig.vtx_en);
  uart1.print(" mhz="); uart1.print((unsigned int)currentConfig.vtx_mhz);
  uart1.print(" mw="); uart1.println((unsigned int)currentConfig.vtx_mw);
}

static void startBenchTestMode(BenchTestMode mode, const char* tag) {
  if (is_armed || CliAutoCal::isFlightActive()) { uart1.println("ERR: armed"); return; }
  if (!SensorDriver::activeSensors.imu_found || err_imu_fail) { uart1.println("ERR: imu"); return; }
  rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset(); resetMotorSlewState();
  bench_test_mode = mode; bench_test_start_ms = millis();
  uart1.print("OK: "); uart1.println(tag);
}

void stopBenchTest() {
  if (bench_test_mode == BENCH_TEST_NONE) return;
  bench_test_mode = BENCH_TEST_NONE; resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
}

static void executeCommand(const char* cmd) {
  const char* s = skipSpaces(cmd);
  if (!s || !*s) return;

  if (strcmp(s, "mon") != 0) {
    mon_stream_active = false;
  }

  if (strcmp(s, "help") == 0) {
    uart1.println("OK: info save default cal_imu cal_gyro cal_acc cal_mag cal_bar cal_lid cal_bat cal_curr auto_cal auto_stop mon test_imu test_mag test_bar test_lid test_bat test_rtx test_vtx test_mot test_pid");
  } else if (strcmp(s, "info") == 0) {
    printSystemInfo();
  } else if (strcmp(s, "save") == 0) {
    saveSettings(); uart1.println("OK: save");
  } else if (strcmp(s, "default") == 0) {
    resetToFactoryDefaults(); applyRtxConfiguration(false); vtxInit(); SensorDriver::autoDetectAll();
    uart1.println("OK: default");
  } else if (strcmp(s, "cal_imu") == 0) {
    SensorDriver::calibrateIMU();
  } else if (strcmp(s, "cal_gyro") == 0) {
    SensorDriver::calibrateGyro(false);
  } else if (strcmp(s, "cal_acc") == 0) {
    SensorDriver::calibrateAcc(false);
  } else if (strcmp(s, "cal_mag") == 0) {
    startMagCalibrationNonBlocking();
  } else if (strcmp(s, "cal_bar") == 0) {
    SensorDriver::calibrateBaro();
  } else if (strcmp(s, "cal_lid") == 0) {
    SensorDriver::calibrateLidar();
  } else if (strncmp(s, "cal_bat", 7) == 0 && (s[7] == '\0' || s[7] == ' ' || s[7] == '\t')) {
    float v = 0.0f;
    if (!parseNumberStrict(s + 7, v)) uart1.println("ERR: cal_bat"); else SensorDriver::calibrateBatVoltage(v);
  } else if (strncmp(s, "cal_curr", 8) == 0 && (s[8] == '\0' || s[8] == ' ' || s[8] == '\t')) {
    const char* ca = skipSpaces(s + 8);
    if (!*ca) SensorDriver::calibrateCurrent(false, 0.0f);
    else { float a = 0.0f; if (!parseNumberStrict(ca, a)) uart1.println("ERR: cal_curr"); else SensorDriver::calibrateCurrent(true, a); }
  } else if (strcmp(s, "auto_cal") == 0) {
    CliAutoCal::start(); uart1.println("OK: auto_cal");
  } else if (strcmp(s, "auto_stop") == 0) {
    CliAutoCal::stop(CliAutoCal::REASON_USER_CANCEL, FC_ERR_NONE); uart1.println("OK: auto_stop");
  } else if (strcmp(s, "mon") == 0) {
    mon_stream_active = true; mon_last_send_ms = 0; uart1.println("OK: mon");
  } else if (strcmp(s, "test_imu") == 0) {
    uint8_t b[14];
    if (!SensorDriver::activeSensors.imu_found || !SensorDriver::readMPU6050(b)) uart1.println("ERR: imu");
    else {
      float ax = (float)((int16_t)(b[0] << 8 | b[1])) / SensorDriver::activeSensors.acc_scale;
      float ay = (float)((int16_t)(b[2] << 8 | b[3])) / SensorDriver::activeSensors.acc_scale;
      float az = (float)((int16_t)(b[4] << 8 | b[5])) / SensorDriver::activeSensors.acc_scale;
      float gx = (float)((int16_t)(b[8] << 8 | b[9])) / SensorDriver::activeSensors.gyro_scale;
      float gy = (float)((int16_t)(b[10] << 8 | b[11])) / SensorDriver::activeSensors.gyro_scale;
      float gz = (float)((int16_t)(b[12] << 8 | b[13])) / SensorDriver::activeSensors.gyro_scale;
      uart1.print("IMU: type="); uart1.print((int)SensorDriver::activeSensors.imu_type);
      uart1.print(" acc="); uart1.print(ax, 2); uart1.print(","); uart1.print(ay, 2); uart1.print(","); uart1.print(az, 2);
      uart1.print(" gyr="); uart1.print(gx, 1); uart1.print(","); uart1.print(gy, 1); uart1.print(","); uart1.println(gz, 1);
    }
  } else if (strcmp(s, "test_mag") == 0) {
    float mx, my, mz, hd = 0.0f;
    if (!SensorDriver::activeSensors.mag_found || !SensorDriver::readMagUnified(mx, my, mz)) uart1.println("ERR: mag");
    else {
      if (!SensorDriver::readCompassHeadingDeg(hd)) hd = 0.0f;
      uart1.print("MAG: type="); uart1.print((int)SensorDriver::activeSensors.mag_type);
      uart1.print(" uT="); uart1.print(mx, 1); uart1.print(","); uart1.print(my, 1); uart1.print(","); uart1.print(mz, 1);
      uart1.print(" hdg="); uart1.println(hd, 1);
    }
  } else if (strcmp(s, "test_bar") == 0) {
    float alt;
    if (!SensorDriver::activeSensors.baro_found || !SensorDriver::readRawBaroAltitudeBlocking(alt)) uart1.println("ERR: baro");
    else { uart1.print("BARO: type="); uart1.print((int)SensorDriver::activeSensors.baro_type); uart1.print(" alt="); uart1.println(alt, 2); }
  } else if (strcmp(s, "test_lid") == 0) {
    float sm;
    if (!SensorDriver::activeSensors.lidar_found || !SensorDriver::readRawLidarSlantMeters(sm)) uart1.println("ERR: lidar");
    else { uart1.print("LIDAR: type="); uart1.print((int)SensorDriver::activeSensors.lidar_type); uart1.print(" raw="); uart1.print(sm, 3); uart1.print(" corr="); uart1.println(SensorDriver::computeCorrectedLidarMeters(sm), 3); }
  } else if (strcmp(s, "test_bat") == 0) {
    readBatteryAndCurrent();
    uart1.print("BAT: v="); uart1.print(v_batt, 2); uart1.print(" i="); uart1.print(current_a, 2);
    uart1.print(" w="); uart1.print(power_w, 1); uart1.print(" valid="); uart1.println(batt_valid ? 1 : 0);
  } else if (strcmp(s, "test_rtx") == 0) {
    uart1.print("RTX: en="); uart1.print((int)currentConfig.rtx_en); uart1.print(" chip="); uart1.print((int)rtx_active_chip_id);
    uart1.print(" chan="); uart1.print((int)active_rtx_chan); uart1.print(" lq="); uart1.println((int)current_link_quality);
  } else if (strcmp(s, "test_vtx") == 0) {
    uart1.print("VTX: en="); uart1.print((int)currentConfig.vtx_en); uart1.print(" proto="); uart1.print((int)vtx_detected_proto);
    uart1.print(" mhz="); uart1.print((unsigned int)currentConfig.vtx_mhz); uart1.print(" mw="); uart1.println((unsigned int)currentConfig.vtx_mw);
  } else if (strcmp(s, "test_mot") == 0) {
    startBenchTestMode(BENCH_TEST_MOTORS, "test_mot");
  } else if (strcmp(s, "test_pid") == 0) {
    startBenchTestMode(BENCH_TEST_PID, "test_pid");
  } else {
    uart1.println("ERR: cmd");
  }
}

void parseCli() {
  static char line[96]; static size_t len = 0;
  while (uart1.available() > 0) {
    int ch = uart1.read(); if (ch < 0) break;
    char c = (char)ch;
    if (c == '\r' || c == '\n') {
      if (len > 0) { line[len] = '\0'; executeCommand(line); len = 0; }
    } else if (len < sizeof(line) - 1) line[len++] = c;
    else len = 0;
  }
}

// ==============================================================================
// 11. УПРАВЛЕНИЕ ПОЛЁТОМ, ШИМ (TIM2) И ОСНОВНОЙ ЦИКЛ (250 Гц)
// ==============================================================================
float calculateSuperExpoRate(float in, float r, float s, float e) {
  float n = constrainF(in / 500.0f, -1.0f, 1.0f), a = fabsf(n), sgn = (n >= 0.0f) ? 1.0f : -1.0f;
  float curve = a * ((1.0f - e) + e * (a * a * a));
  float out = (r * 10.0f) * curve * sgn;
  if (s > 0.0f) { float denom = 1.0f - (a * s); if (denom < 0.1f) denom = 0.1f; out /= denom; }
  return isfinite(out) ? constrainF(out, -1.0f, 1.0f) : 0.0f;
}

float applyDeadband(float v, float db) {
  float d = v - STICK_CENTER;
  if (fabsf(d) <= db) return STICK_CENTER;
  return STICK_CENTER + ((d > 0.0f) ? (d - db) : (d + db));
}

void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float an = sqrtf(ax * ax + ay * ay + az * az);
  float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
  if (an > 1e-6f && isfinite(an)) {
    ax /= an; ay /= an; az /= an;
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2, _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float f1 = _2q1 * q3 - _2q0 * q2 - ax, f2 = _2q0 * q1 + _2q2 * q3 - ay, f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - az;
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 1e-9f && isfinite(sn)) { s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn; }
    else s0 = s1 = s2 = s3 = 0.0f;
  }
  float beta = 0.1f;
  if (currentConfig.flt_amb_en) {
    float hi = constrainF(currentConfig.flt_amb, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
    float lo = constrainF(currentConfig.flt_amb_min, 0.0005f, hi);
    beta = constrainF(hi - (vibration_index * 0.002f), lo, hi);
  }
  orientation.q0 = q0 + (0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0) * dt;
  orientation.q1 = q1 + (0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1) * dt;
  orientation.q2 = q2 + (0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2) * dt;
  orientation.q3 = q3 + (0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3) * dt;
  orientation.normalize();
  if (!isfinite(orientation.q0) || !isfinite(orientation.q1) || !isfinite(orientation.q2) || !isfinite(orientation.q3)) {
    orientation = Quaternion(); setFcErrorRam(FC_ERR_NAN_INF_MATH);
  }
}

void setupHardwareTimers() {
  gpioConfigPin(Reg::GPIOA, 0, GPIO_AF_PP_50M);
  gpioConfigPin(Reg::GPIOA, 1, GPIO_AF_PP_50M);
  gpioConfigPin(Reg::GPIOA, 2, GPIO_AF_PP_50M);
  gpioConfigPin(Reg::GPIOA, 3, GPIO_AF_PP_50M);

  uint16_t arr = (currentConfig.esc_prot == 1) ? 2499U : 19999U;
  Reg::TIM2->CR1 = 0U; Reg::TIM2->PSC = 71U; Reg::TIM2->ARR = arr;
  Reg::TIM2->CCMR1 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);
  Reg::TIM2->CCMR2 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);
  Reg::TIM2->CCER = (1U << 0) | (1U << 4) | (1U << 8) | (1U << 12);
  Reg::TIM2->CCR1 = Reg::TIM2->CCR2 = Reg::TIM2->CCR3 = Reg::TIM2->CCR4 = 1000U;
  Reg::TIM2->EGR = 1U; Reg::TIM2->CR1 = (1U << 7) | 1U;
}

void resetMotorSlewState() { prev_m1_out = prev_m2_out = prev_m3_out = prev_m4_out = 0.0f; }

static inline float applyMotorSlew(float target, float &prev) {
  float lim = getEffectiveEscDeltaLimit(), t = constrainF(target, OUTPUT_MIN, OUTPUT_MAX);
  if (!isfinite(prev)) prev = 0.0f;
  prev = constrainF(t, prev - lim, prev + lim);
  return prev;
}

void setMotorsPower(float m1, float m2, float m3, float m4, float max_cap) {
  float cap = constrainF(max_cap, OUTPUT_MIN, OUTPUT_MAX);
  uint16_t p1 = (uint16_t)constrainL(lroundf(1000.0f + constrainF(applyMotorSlew(m1, prev_m1_out), 0.0f, cap)), 1000L, 2000L);
  uint16_t p2 = (uint16_t)constrainL(lroundf(1000.0f + constrainF(applyMotorSlew(m2, prev_m2_out), 0.0f, cap)), 1000L, 2000L);
  uint16_t p3 = (uint16_t)constrainL(lroundf(1000.0f + constrainF(applyMotorSlew(m3, prev_m3_out), 0.0f, cap)), 1000L, 2000L);
  uint16_t p4 = (uint16_t)constrainL(lroundf(1000.0f + constrainF(applyMotorSlew(m4, prev_m4_out), 0.0f, cap)), 1000L, 2000L);
  Reg::TIM2->CCR1 = p1; Reg::TIM2->CCR2 = p2; Reg::TIM2->CCR3 = p3; Reg::TIM2->CCR4 = p4;
}

void stepBenchTest(uint32_t now_ms, float gx_dps, float gy_dps, float gz_dps, float gx_rad, float gy_rad, float gz_rad) {
  if (bench_test_mode == BENCH_TEST_NONE) return;
  if (is_armed || CliAutoCal::isFlightActive() || (now_ms - bench_test_start_ms) >= BENCH_TEST_DURATION_MS) { stopBenchTest(); return; }
  float m1 = 0.0f, m2 = 0.0f, m3 = 0.0f, m4 = 0.0f;
  if (bench_test_mode == BENCH_TEST_MOTORS) {
    uint32_t el = now_ms - bench_test_start_ms;
    float r = BENCH_TEST_THROTTLE * constrainF((float)el / 500.0f, 0.0f, 1.0f);
    if (el >= 9500) r *= constrainF((float)(10000 - el) / 500.0f, 0.0f, 1.0f);
    uint8_t ph = (uint8_t)((el / 2500) % 4);
    if (ph == 0) m1 = r; else if (ph == 1) m2 = r; else if (ph == 2) m3 = r; else m4 = r;
  } else {
    float rc = rollRatePID.update(-gx_rad, gx_rad, DT), pc = pitchRatePID.update(-gy_rad, gy_rad, DT), yc = yawRatePID.update(-gz_rad, gz_rad, DT);
    m1 = constrainF(80.0f + pc + rc - yc, 0.0f, 250.0f); m2 = constrainF(80.0f + pc - rc + yc, 0.0f, 250.0f);
    m3 = constrainF(80.0f - pc - rc - yc, 0.0f, 250.0f); m4 = constrainF(80.0f - pc + rc + yc, 0.0f, 250.0f);
  }
  (void)gx_dps; (void)gy_dps; (void)gz_dps;
  setMotorsPower(m1, m2, m3, m4, 250.0f);
}

void flightControlTask() {
  uint32_t now_ms = millis();
  handleMotorArmStateAndEepromError();

  if (SensorDriver::readMPU6050(mpu_dma_buffer)) {
    err_imu_fail = false;
    float ax_r = (float)((int16_t)(mpu_dma_buffer[0] << 8 | mpu_dma_buffer[1])) / SensorDriver::activeSensors.acc_scale;
    float ay_r = (float)((int16_t)(mpu_dma_buffer[2] << 8 | mpu_dma_buffer[3])) / SensorDriver::activeSensors.acc_scale;
    float az_r = (float)((int16_t)(mpu_dma_buffer[4] << 8 | mpu_dma_buffer[5])) / SensorDriver::activeSensors.acc_scale;
    float tc = SensorDriver::extractTempFromImuBuf(mpu_dma_buffer);
    current_chip_temp = lpf_temp.update(tc);
    float gx_r = (float)((int16_t)(mpu_dma_buffer[8] << 8 | mpu_dma_buffer[9])) / SensorDriver::activeSensors.gyro_scale;
    float gy_r = (float)((int16_t)(mpu_dma_buffer[10] << 8 | mpu_dma_buffer[11])) / SensorDriver::activeSensors.gyro_scale;
    float gz_r = (float)((int16_t)(mpu_dma_buffer[12] << 8 | mpu_dma_buffer[13])) / SensorDriver::activeSensors.gyro_scale;
    SensorDriver::rotateXYByYawAlign(ax_r, ay_r, currentConfig.imu_align);
    SensorDriver::rotateXYByYawAlign(gx_r, gy_r, currentConfig.imu_align);

    float dt_t = current_chip_temp - currentConfig.base_temperature;
    float ax = ax_r - currentConfig.acc_bias_x, ay = ay_r - currentConfig.acc_bias_y, az = az_r - currentConfig.acc_bias_z;
    float gx = gx_r - currentConfig.gyro_bias_x - (currentConfig.temp_coeff_x * dt_t);
    float gy = gy_r - currentConfig.gyro_bias_y - (currentConfig.temp_coeff_y * dt_t);
    float gz = gz_r - currentConfig.gyro_bias_z - (currentConfig.temp_coeff_z * dt_t);

    if (currentConfig.flt_lpf_en) {
      gx = lpf_gx.update(gx); gy = lpf_gy.update(gy); gz = lpf_gz.update(gz);
      ax = lpf_ax.update(ax); ay = lpf_ay.update(ay); az = lpf_az.update(az);
    }
    if (currentConfig.flt_not_en) {
      notch_gx.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw);
      notch_gy.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw);
      notch_gz.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw);
      gx = notch_gx.update(gx); gy = notch_gy.update(gy); gz = notch_gz.update(gz);
    }

    float hp_x = gx - lpf_xy_hp_x.update(gx), hp_y = gy - lpf_xy_hp_y.update(gy);
    vibration_index = constrainF(0.92f * vibration_index + 0.08f * sqrtf(hp_x * hp_x + hp_y * hp_y), 0.0f, 500.0f);

    float gxr = gx * FC_DEG_TO_RAD, gyr = gy * FC_DEG_TO_RAD, gzr = gz * FC_DEG_TO_RAD;
    madgwickUpdate(gxr, gyr, gzr, ax, ay, az, DT);

    float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
    float roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * FC_RAD_TO_DEG;
    float pitch_deg = asinf(constrainF(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * FC_RAD_TO_DEG;
    CliAutoCal::onAttitudeAndMotion(ax, ay, az, roll_deg, pitch_deg, gx, gy);

    float ac_m1 = 0.0f, ac_m2 = 0.0f, ac_m3 = 0.0f, ac_m4 = 0.0f;
    if (CliAutoCal::stepFlight(now_ms, gx, gy, gz, gxr, gyr, gzr, roll_deg, pitch_deg, ac_m1, ac_m2, ac_m3, ac_m4)) {
      setMotorsPower(ac_m1, ac_m2, ac_m3, ac_m4); return;
    }

    if (bench_test_mode != BENCH_TEST_NONE) { stepBenchTest(now_ms, gx, gy, gz, gxr, gyr, gzr); return; }

    if (currentConfig.fs_rtx_en && (now_ms - last_rc_packet_ms) > FAILSAFE_TIMEOUT_MS) {
      if (!is_failsafe) { is_failsafe = true; setFcErrorRam(FC_ERR_RC_LOSS); }
    } else if (is_failsafe && (now_ms - last_rc_packet_ms) <= FAILSAFE_TIMEOUT_MS) is_failsafe = false;

    if (!is_armed) {
      arm_ramp_factor = 0.0f;
      vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
      resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f); return;
    }

    arm_ramp_factor = constrainF(arm_ramp_factor + 0.008f, 0.0f, 1.0f);
    float bf = getBatteryCompensationFactor();
    float rc = calculateSuperExpoRate(applyDeadband(smooth_roll, currentConfig.rc_deadband), currentConfig.rc_rate, currentConfig.rc_super_expo, currentConfig.rc_stick_expo);
    float pc = calculateSuperExpoRate(applyDeadband(smooth_pitch, currentConfig.rc_deadband), currentConfig.rc_rate, currentConfig.rc_super_expo, currentConfig.rc_stick_expo);
    float yc = calculateSuperExpoRate(applyDeadband(smooth_yaw, currentConfig.rc_deadband), currentConfig.rc_yaw_rate, currentConfig.rc_yaw_super_expo, currentConfig.rc_yaw_expo);

    float thr = is_failsafe ? constrainF(base_hover_throttle - (currentConfig.fs_vz_speed * 60.0f), 0.0f, 650.0f) : smooth_throttle;
    if (is_failsafe && dbg_lidar_m > 0.0f && dbg_lidar_m < 0.08f) {
      is_armed = false; arm_ramp_factor = 0.0f; resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f); return;
    }

    float tgx = 0.0f, tgy = 0.0f, tgz = yc * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD;
    if (currentConfig.fm_mode == 0) {
      tgx = rc * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD; tgy = pc * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD;
    } else if (currentConfig.fm_mode == 1) {
      tgx = constrainF((rc * currentConfig.fm_roll_max) - roll_deg, -currentConfig.fm_roll_max, currentConfig.fm_roll_max) * 2.5f * FC_DEG_TO_RAD;
      tgy = constrainF((pc * currentConfig.fm_pitch_max) - pitch_deg, -currentConfig.fm_pitch_max, currentConfig.fm_pitch_max) * 2.5f * FC_DEG_TO_RAD;
    } else {
      float tr = rc * currentConfig.fm_roll_max, tp = pc * currentConfig.fm_pitch_max;
      float st = constrainF((fabsf(thr - base_hover_throttle) / 250.0f) / currentConfig.rc_horizon_transition, 0.0f, 1.0f);
      float arx = constrainF(tr - roll_deg, -currentConfig.fm_roll_max, currentConfig.fm_roll_max) * 2.5f * FC_DEG_TO_RAD;
      float ary = constrainF(tp - pitch_deg, -currentConfig.fm_pitch_max, currentConfig.fm_pitch_max) * 2.5f * FC_DEG_TO_RAD;
      tgx = (1.0f - st) * arx + st * (rc * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD);
      tgy = (1.0f - st) * ary + st * (pc * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD);
    }

    float to = thr * bf;
    bool sat = (to <= 5.0f || to >= 995.0f);
    if (currentConfig.fm_mode == 3 && !is_failsafe) {
      to = constrainF(base_hover_throttle + vzPID.update(((smooth_throttle - STICK_CENTER) / 500.0f) * currentConfig.fm_vz_speed_max - current_vz, current_vz, DT, sat), 0.0f, 900.0f) * bf;
    }

    float r_cmd = rollRatePID.update(tgx - gxr, gxr, DT, sat), p_cmd = pitchRatePID.update(tgy - gyr, gyr, DT, sat), y_cmd = yawRatePID.update(tgz - gzr, gzr, DT, sat);
    float m1 = (to + p_cmd + r_cmd - y_cmd) * arm_ramp_factor, m2 = (to + p_cmd - r_cmd + y_cmd) * arm_ramp_factor;
    float m3 = (to - p_cmd - r_cmd - y_cmd) * arm_ramp_factor, m4 = (to - p_cmd + r_cmd + y_cmd) * arm_ramp_factor;
    float mx = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
    if (mx > OUTPUT_MAX) { float sc = OUTPUT_MAX / mx; m1 *= sc; m2 *= sc; m3 *= sc; m4 *= sc; }
    setMotorsPower(m1, m2, m3, m4);
  } else {
    err_imu_fail = true; setFcErrorRam(FC_ERR_IMU_FAIL);
    if (is_armed || CliAutoCal::isFlightActive()) { is_armed = false; resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f); }
  }
}

void setup() {
  Reg::RCC->APB2ENR |= (1U << 0) | (1U << 2) | (1U << 3) | (1U << 9) | (1U << 14);
  Reg::RCC->APB1ENR |= (1U << 0) | (1U << 14) | (1U << 18) | (1U << 21);

  uart1.begin(SERIAL_BAUD_RATE);
  delay(200);
  initI2c1(); initAdc1();
  loadSettings();
  setupHardwareTimers();
  rtx_init_hardware();
  main_rtx_chan = active_rtx_chan = (uint8_t)constrainI(currentConfig.rtx_pri_chan, 0, 99);
  is_on_backup = false;
  applyRtxConfiguration(false);
  vtxInit();
  SensorDriver::autoDetectAll();
  applyFilterConfig();
  CliAutoCal::init();
  last_rc_packet_ms = millis();
  uart1.println("OK: v4.40");
}

void loop() {
  static uint32_t last_loop_us = micros();
  uint32_t now_us = micros();
  if ((now_us - last_loop_us) >= LOOP_INTERVAL_US) {
    uint32_t late_us = (now_us - last_loop_us) - LOOP_INTERVAL_US;
    last_loop_us = now_us;
    if (late_us > LOOP_OVERRUN_MARGIN_US && (is_armed || CliAutoCal::isFlightActive())) setFcErrorRam(FC_ERR_LOOP_OVERRUN);

    uint32_t t0 = micros();
    flightControlTask();
    uint32_t now_ms = millis();
    CliAutoCal::pollButton(now_ms);
    updateAuxSensorsNonBlocking(now_ms);
    vtxStepNonBlocking(now_ms);
    stepMagCalibrationNonBlocking(now_ms);
    serviceError202Event(now_ms);
    pollTelemetryTxState();
    stepChannelScanNonBlocking();
    readRx();
    if (send_telemetry_pending && !rtx_tx_in_flight && !rtx_scan_active) startTelemetryNonBlocking();
    parseCli();
    stepMonitorStreamNonBlocking(now_ms);

    float ld = ((float)(micros() - t0) / (float)LOOP_INTERVAL_US) * 100.0f;
    if (isfinite(ld)) {
      cpu_load_pct = constrainF(ld, 0.0f, 999.0f);
      if (cpu_load_pct > cpu_max_load_pct) cpu_max_load_pct = cpu_load_pct;
    }
  }
}

int main(void) {
  SystemInit();
  setup();
  while (1) {
    loop();
  }
}
