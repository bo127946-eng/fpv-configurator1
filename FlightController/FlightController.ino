/*
 * ==============================================================================
 * Project: FPV Flight Controller Firmware (FC v3.40 / ESC Protocols: PWM & Oneshot125)
 * Target: STM32F103C8 (Blue Pill) + Multi-IMU + Multi-Baro + Multi-Mag + Multi-ToF + RTX(SPI2) + VTX(USART3 SA v2.1 / Tramp)
 * ==============================================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ==============================================================================
// 1. КОНСТАНТЫ И ТАЙМИНГИ
// ==============================================================================
constexpr uint32_t POWER_ON_SETTLE_MS = 500;
constexpr uint32_t LOOP_INTERVAL_US = 4000;
constexpr uint32_t LOOP_OVERRUN_MARGIN_US = 200;
constexpr float DT = 0.004f;
constexpr uint32_t TIM2_TICK_HZ = 8000000UL;
constexpr uint32_t TIM2_ONESHOT_HZ = 24000000UL;

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

// ==============================================================================
// 2. ПИНЫ И АППАРАТНЫЕ ПАРАМЕТРЫ
// ==============================================================================
constexpr uint8_t MOTOR1_PIN = PA0;
constexpr uint8_t MOTOR2_PIN = PA1;
constexpr uint8_t MOTOR3_PIN = PA2;
constexpr uint8_t MOTOR4_PIN = PA3;
constexpr uint8_t BATTERY_PIN = PA4;
constexpr uint8_t CURRENT_PIN = PA5;
constexpr uint8_t VTX_TX_PIN = PB10;
constexpr uint8_t VTX_RX_PIN = PB11;
constexpr float SHUNT_RESISTANCE_OHM = 0.01f;

constexpr uint8_t RTX_SCK_PIN = PB13;
constexpr uint8_t RTX_MISO_PIN = PB14;
constexpr uint8_t RTX_MOSI_PIN = PB15;
constexpr uint8_t RTX_NSS_PIN = PB12;
constexpr uint8_t RTX_RESET_PIN = PB0;
constexpr uint8_t RTX_BUSY_PIN = PB1;

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
constexpr float FC_DEG_TO_RAD = 0.01745329251994329577f;
constexpr float FC_RAD_TO_DEG = 57.29577951308232087680f;
constexpr float SIN_45 = 0.70710678f;
constexpr float COS_45 = 0.70710678f;

constexpr uint32_t FAILSAFE_TIMEOUT_MS = 1000;
constexpr uint8_t I2C_MAX_ERRORS = 10;
constexpr uint8_t RTX_MAX_ERRORS = 10;

// ==============================================================================
// 3. ПЕРЕЧИСЛЕНИЯ И СТРУКТУРЫ ДАННЫХ
// ==============================================================================
enum FcErrorCode : uint8_t {
  FC_ERR_NONE = 0,
  FC_ERR_LOOP_OVERRUN = 1,
  FC_ERR_IMU_FAIL = 2,
  FC_ERR_I2C_BUS = 3,
  FC_ERR_SPI_RTX = 4,
  FC_ERR_RC_LOSS = 5,
  FC_ERR_BAT_LOW = 6,
  FC_ERR_TILT_DISARM = 7,
  FC_ERR_RTX_TX_TIMEOUT = 8,
  FC_ERR_NAN_INF_MATH = 9,
  FC_ERR_BARO_FAIL = 10,
  FC_ERR_LIDAR_FAIL = 11,
  FC_ERR_MAG_FAIL = 12,
  FC_ERR_EEPROM_RESET = 13,
  FC_ERR_BAT_SENSOR_FAIL = 14,
  FC_ERR_SA_VTX_FAIL = 16,
  FC_ERR_CALIB_MISSING = 18
};

enum VtxProtSetting : uint8_t {
  VTX_PROT_AUTO = 0,
  VTX_PROT_TRAMP = 1,
  VTX_PROT_SMARTAUDIO = 2
};

enum VtxActiveProto : uint8_t {
  VTX_PROTO_NONE = 0,
  VTX_PROTO_TRAMP = 1,
  VTX_PROTO_SMARTAUDIO = 2
};

enum RtxChipId : uint8_t {
  RTX_CHIP_AUTO = 0,
  RTX_CHIP_SX1262 = 1,
  RTX_CHIP_SX1268 = 2,
  RTX_CHIP_LLCC68 = 3
};

enum VtxProbeStep : uint8_t {
  VTX_STEP_IDLE = 0,
  VTX_STEP_WAIT_SA,
  VTX_STEP_WAIT_TRAMP,
  VTX_STEP_WAIT_CMD_ACK
};

enum DelayedRtxRespType : uint8_t {
  RTX_RESP_NONE = 0,
  RTX_RESP_202_ERR_EVENT = 202,
  RTX_RESP_230_SCAN_BEST = 230,
  RTX_RESP_240_CHAN_SW = 240,
  RTX_RESP_250_VTX_ACK = 250
};

enum BenchTestMode : uint8_t {
  BENCH_TEST_NONE = 0,
  BENCH_TEST_MOTORS = 1,
  BENCH_TEST_PID = 2
};

enum CliValType : uint8_t;
struct CliParam;

struct DelayedRtxRespItem {
  DelayedRtxRespType type = RTX_RESP_NONE;
  uint16_t vtx_mhz = 0;
  uint8_t tier = 0;
  uint16_t vtx_mw = 0;
};

// Значения по умолчанию
constexpr uint8_t DEFAULT_ESC_PROT = 0;
constexpr uint8_t DEFAULT_IMU_ALIGN = 0;
constexpr uint8_t DEFAULT_COM_ALIGN = 0;
constexpr uint8_t DEFAULT_IMU_ID = 0;
constexpr uint8_t DEFAULT_BAR_ID = 0;
constexpr uint8_t DEFAULT_COM_ID = 0;
constexpr uint8_t DEFAULT_LID_ID = 0;
constexpr uint8_t DEFAULT_RTX_ID = RTX_CHIP_AUTO;

constexpr uint8_t DEFAULT_VTX_EN = 1;
constexpr uint8_t DEFAULT_VTX_PROT = VTX_PROT_AUTO;
constexpr uint16_t DEFAULT_VTX_MHZ = 5658;
constexpr uint16_t DEFAULT_VTX_MW = 25;

constexpr float DEFAULT_GYRO_BIAS_X = 0.0f;
constexpr float DEFAULT_GYRO_BIAS_Y = 0.0f;
constexpr float DEFAULT_GYRO_BIAS_Z = 0.0f;
constexpr float DEFAULT_BASE_TEMP = 25.0f;
constexpr float DEFAULT_TEMP_COEFF_X = 0.0005f;
constexpr float DEFAULT_TEMP_COEFF_Y = 0.0005f;
constexpr float DEFAULT_TEMP_COEFF_Z = 0.0005f;
constexpr float DEFAULT_ACC_BIAS_X = 0.0f;
constexpr float DEFAULT_ACC_BIAS_Y = 0.0f;
constexpr float DEFAULT_ACC_BIAS_Z = 0.0f;
constexpr float DEFAULT_MAG_BIAS_X = 0.0f;
constexpr float DEFAULT_MAG_BIAS_Y = 0.0f;
constexpr float DEFAULT_MAG_BIAS_Z = 0.0f;
constexpr float DEFAULT_LIDAR_OFFSET_M = 0.0f;
constexpr float DEFAULT_BAT_SCALE_V = 4.85f;
constexpr float DEFAULT_BAT_OFFSET_A = 0.0f;
constexpr float DEFAULT_BAT_SCALE_A = 1.0f;
constexpr uint8_t DEFAULT_BAT_COMP_EN = 1;
constexpr float DEFAULT_BAT_V_MAX = 16.8f;
constexpr float DEFAULT_FLT_ESC_LIMIT = 45.0f;
constexpr uint8_t DEFAULT_FC_ERROR = FC_ERR_NONE;

constexpr uint8_t DEFAULT_FM_MODE = 1;
constexpr float DEFAULT_FM_ROLL_MAX = 25.0f;
constexpr float DEFAULT_FM_PITCH_MAX = 25.0f;
constexpr float DEFAULT_FM_VZ_SPEED_MAX = 1.2f;

constexpr float DEFAULT_RC_RATE = 0.35f;
constexpr float DEFAULT_RC_SUPER_EXPO = 0.03f;
constexpr float DEFAULT_RC_STICK_EXPO = 0.45f;
constexpr float DEFAULT_RC_YAW_RATE = 0.30f;
constexpr float DEFAULT_RC_YAW_SUPER_EXPO = 0.03f;
constexpr float DEFAULT_RC_YAW_EXPO = 0.35f;
constexpr float DEFAULT_RC_DEADBAND = 15.0f;
constexpr float DEFAULT_RC_HORIZON_TRANS = 0.75f;
constexpr float DEFAULT_RC_SMOOTH = 0.35f;

constexpr uint8_t DEFAULT_FS_BAT_EN = 1;
constexpr uint8_t DEFAULT_FS_RTX_EN = 1;
constexpr float DEFAULT_FS_VZ_SPEED = 0.5f;

constexpr float DEFAULT_BAT_FS_V = 13.8f;
constexpr float DEFAULT_BAT_FILTER_ALPHA = 0.05f;

constexpr uint8_t DEFAULT_LIDAR_EN = 1;
constexpr float DEFAULT_LIDAR_FIL_ALPHA = 0.15f;
constexpr uint8_t DEFAULT_BARO_EN = 1;
constexpr uint8_t DEFAULT_COM_EN = 0;

constexpr uint8_t DEFAULT_FLT_AMB_EN = 1;
constexpr uint8_t DEFAULT_FLT_LPF_EN = 1;
constexpr uint8_t DEFAULT_FLT_NOT_EN = 1;
constexpr float DEFAULT_GYRO_FILTER_ALPHA = 0.10f;
constexpr float DEFAULT_ACC_FILTER_ALPHA = 0.10f;
constexpr float DEFAULT_NOTCH_FREQ = 120.0f;
constexpr float DEFAULT_NOTCH_BW = 20.0f;

constexpr uint8_t DEFAULT_RTX_EN = 1;
constexpr uint8_t DEFAULT_RTX_BAND = 9;
constexpr uint8_t DEFAULT_RTX_HW_CRC = 1;
constexpr uint8_t DEFAULT_RTX_ADDR_EN = 1;
constexpr uint16_t DEFAULT_RTX_ADDR = 0x0000;
constexpr uint16_t DEFAULT_RTX_SYNC = 0x1424;
constexpr uint8_t DEFAULT_RTX_IQ_INV = 0;
constexpr uint8_t DEFAULT_RTX_PRI_CHAN = 15;
constexpr uint8_t DEFAULT_RTX_PRI_SF = 7;
constexpr uint8_t DEFAULT_RTX_PRI_BW = 7;
constexpr uint8_t DEFAULT_RTX_PRI_CR = 1;
constexpr int8_t DEFAULT_RTX_PRI_POWER = 22;
constexpr uint8_t DEFAULT_RTX_BAK_CHAN = 25;
constexpr uint8_t DEFAULT_RTX_BAK_SF = 9;
constexpr uint8_t DEFAULT_RTX_BAK_BW = 7;
constexpr uint8_t DEFAULT_RTX_BAK_CR = 1;
constexpr int8_t DEFAULT_RTX_BAK_POWER = 22;

constexpr float PID_ROLL_KP_DEF = 0.45f;
constexpr float PID_ROLL_KI_DEF = 0.010f;
constexpr float PID_ROLL_KD_DEF = 0.018f;
constexpr float PID_PITCH_KP_DEF = 0.45f;
constexpr float PID_PITCH_KI_DEF = 0.010f;
constexpr float PID_PITCH_KD_DEF = 0.018f;
constexpr float PID_YAW_KP_DEF = 0.60f;
constexpr float PID_YAW_KI_DEF = 0.010f;
constexpr float PID_YAW_KD_DEF = 0.0f;
constexpr float PID_VZ_KP_DEF = 7.0f;
constexpr float PID_VZ_KI_DEF = 1.2f;
constexpr float PID_VZ_KD_DEF = 0.30f;

struct PIDConfig {
  float kp, ki, kd;
};

struct __attribute__((aligned(4))) FlightConfig {
  uint32_t magic_marker;
  float gyro_bias_x, gyro_bias_y, gyro_bias_z;
  float base_temperature;
  float temp_coeff_x, temp_coeff_y, temp_coeff_z;
  float acc_bias_x, acc_bias_y, acc_bias_z;
  float mag_bias_x, mag_bias_y, mag_bias_z;
  float lidar_offset_m;
  float bat_scale_v;
  float bat_offset_a;
  float bat_scale_a;
  float bat_v_max;
  float flt_esc_limit;
  float fm_roll_max;
  float fm_pitch_max;
  float fm_vz_speed_max;
  float gyro_filter_alpha;
  float acc_filter_alpha;
  float filter_notch_freq;
  float filter_notch_bw;
  float flt_amb;
  float flt_amb_min;
  float fs_vz_speed;
  float rc_rate, rc_super_expo, rc_stick_expo;
  float rc_yaw_rate, rc_yaw_super_expo, rc_yaw_expo;
  float rc_deadband;
  float rc_horizon_transition;
  float rc_filter_smoothing;
  float bat_fs_v;
  float bat_filter_alpha;
  float lidar_filter_alpha;
  PIDConfig roll_pid;
  PIDConfig pitch_pid;
  PIDConfig yaw_pid;
  PIDConfig vz_pid;
  uint16_t vtx_mhz;
  uint16_t vtx_mw;
  uint16_t rtx_addr;
  uint16_t rtx_sync_word;
  uint8_t bat_comp_en;
  uint8_t vtx_en;
  uint8_t vtx_prot;
  uint8_t esc_prot;
  uint8_t imu_align;
  uint8_t com_align;
  uint8_t fc_error;
  uint8_t fm_mode;
  uint8_t flt_amb_en;
  uint8_t flt_lpf_en;
  uint8_t flt_not_en;
  uint8_t fs_bat_en;
  uint8_t fs_rtx_en;
  uint8_t lidar_en;
  uint8_t baro_en;
  uint8_t com_en;
  uint8_t rtx_en;
  uint8_t rtx_band;
  uint8_t rtx_hw_crc_en;
  uint8_t rtx_addr_en;
  uint8_t rtx_iq_inv;
  uint8_t rtx_pri_chan;
  uint8_t rtx_pri_sf;
  uint8_t rtx_pri_bw;
  uint8_t rtx_pri_cr;
  int8_t rtx_pri_power;
  uint8_t rtx_bak_chan;
  uint8_t rtx_bak_sf;
  uint8_t rtx_bak_bw;
  uint8_t rtx_bak_cr;
  int8_t rtx_bak_power;
  uint8_t rtx_id;
  uint8_t imu_id;
  uint8_t bar_id;
  uint8_t com_id;
  uint8_t lid_id;
  uint8_t _pad[2];
};

static_assert((sizeof(FlightConfig) % 4) == 0, "FlightConfig size must be multiple of 4 bytes");
static_assert(sizeof(FlightConfig) <= 2048, "FlightConfig exceeds safe STM32F103C8 EEPROM page budget");

struct Quaternion {
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  void normalize() {
    if (isnan(q0) || isnan(q1) || isnan(q2) || isnan(q3) ||
        isinf(q0) || isinf(q1) || isinf(q2) || isinf(q3)) {
      q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
      return;
    }
    float norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (norm_sq > 0.0f && !isnan(norm_sq) && !isinf(norm_sq)) {
      float inv_norm = 1.0f / sqrtf(norm_sq);
      if (!isnan(inv_norm) && !isinf(inv_norm)) {
        q0 *= inv_norm; q1 *= inv_norm; q2 *= inv_norm; q3 *= inv_norm;
        return;
      }
    }
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  }
};

struct LowPassFilter {
  float alpha = 0.1f;
  float output = 0.0f;
  void reset(float val = 0.0f) { output = val; }
  float update(float input) {
    if (isnan(input) || isinf(input)) return output;
    output = alpha * input + (1.0f - alpha) * output;
    return isnan(output) ? 0.0f : output;
  }
};

struct BiquadNotchFilter {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
  float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
  float last_freq = -1.0f;
  float last_bw = -1.0f;
  void reset() { x1 = x2 = y1 = y2 = 0.0f; }
  void setParameters(float sampleRate, float freq, float bandwidth, bool force = false) {
    if (!force && fabsf(freq - last_freq) < 1.5f && fabsf(bandwidth - last_bw) < 0.5f) return;
    last_freq = freq;
    last_bw = bandwidth;
    if (freq <= 0.0f || bandwidth <= 0.0f || sampleRate <= 0.0f) {
      b0 = 1.0f; b1 = 0.0f; b2 = 0.0f; a1 = 0.0f; a2 = 0.0f;
      return;
    }
    float omega = 2.0f * (float)M_PI * freq / sampleRate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float sn_safe = (fabsf(sn) < 1e-6f) ? ((sn >= 0.0f) ? 1e-6f : -1e-6f) : sn;
    float alpha = sn * sinhf(logf(2.0f) / 2.0f * bandwidth * omega / sn_safe);
    float a0 = 1.0f + alpha;
    b0 = 1.0f / a0;
    b1 = (-2.0f * cs) / a0;
    b2 = (1.0f - alpha) / a0;
    a1 = (-2.0f * cs) / a0;
    a2 = (1.0f - alpha) / a0;
  }
  float update(float input) {
    if (b0 == 1.0f && b1 == 0.0f && a1 == 0.0f && a2 == 0.0f) return input;
    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = input;
    y2 = y1; y1 = output;
    return (isnan(output) || isinf(output)) ? input : output;
  }
};

struct AdaptiveLidarFilter {
  float output = 0.0f;
  bool initialized = false;
  float history[3] = {0.0f, 0.0f, 0.0f};
  void reset() {
    output = 0.0f;
    initialized = false;
    history[0] = 0.0f; history[1] = 0.0f; history[2] = 0.0f;
  }
  float update(float raw_val, float base_alpha) {
    if (isnan(raw_val) || isinf(raw_val)) return output;
    if (!initialized) {
      output = raw_val;
      history[0] = raw_val; history[1] = raw_val; history[2] = raw_val;
      initialized = true;
      return output;
    }
    history[0] = history[1];
    history[1] = history[2];
    history[2] = raw_val;
    float median = history[1];
    if ((history[0] >= history[1] && history[0] <= history[2]) || (history[0] <= history[1] && history[0] >= history[2]))
      median = history[0];
    else if ((history[2] >= history[0] && history[2] <= history[1]) || (history[2] <= history[0] && history[2] >= history[1]))
      median = history[2];
    output = base_alpha * median + (1.0f - base_alpha) * output;
    return isnan(output) ? raw_val : output;
  }
};

struct PID {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float integral = 0.0f;
  float last_measurement = 0.0f;
  float last_derivative = 0.0f;
  float int_limit = 200.0f;
  void init(const PIDConfig &cfg, float limit) {
    kp = cfg.kp; ki = cfg.ki; kd = cfg.kd;
    int_limit = limit;
    reset();
  }
  void reset() {
    integral = 0.0f;
    last_measurement = 0.0f;
    last_derivative = 0.0f;
  }
  void updateConfig(const PIDConfig &cfg) {
    kp = cfg.kp; ki = cfg.ki; kd = cfg.kd;
  }
  float update(float error, float measurement, float dt, bool is_saturated = false) {
    if (isnan(error) || isnan(measurement) || isnan(dt) || isinf(error) || isinf(measurement)) return 0.0f;
    if (isnan(integral) || isinf(integral)) integral = 0.0f;
    if (!is_saturated && int_limit > 0.0f && dt > 0.0f) {
      integral += error * dt;
      integral = constrain(integral, -int_limit, int_limit);
    }
    float raw_derivative = (dt > 0.0f) ? -(measurement - last_measurement) / dt : 0.0f;
    if (isnan(raw_derivative) || isinf(raw_derivative)) raw_derivative = 0.0f;
    last_measurement = measurement;
    float derivative = PID_D_ALPHA * raw_derivative + (1.0f - PID_D_ALPHA) * last_derivative;
    if (isnan(derivative) || isinf(derivative)) derivative = 0.0f;
    last_derivative = derivative;
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    return (isnan(output) || isinf(output)) ? 0.0f : output;
  }
};

// ==============================================================================
// 4. ОПЕРЕЖАЮЩИЕ ОБЪЯВЛЕНИЯ И ГЛОБАЛЬНОЕ СОСТОЯНИЕ
// ==============================================================================
void loadSettings();
void saveSettings();
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
inline void queueDelayedRtxResponse(DelayedRtxRespType type, uint16_t vtx_mhz) {
  queueDelayedRtxResponse(type, vtx_mhz, 0, 0);
}
bool hasQueuedRtxResponse();
bool popDelayedRtxResponse(DelayedRtxRespItem &out);
void setupHardwareTimers();
void setMotorsPower(float m1, float m2, float m3, float m4, float max_cap = OUTPUT_MAX);
void resetMotorSlewState();
void flightControlTask();
void parseCli();
void executeCommand(const char* cmd);
float calculateSuperExpoRate(float in, float r, float s, float e);
void setFcErrorRam(uint8_t err_code);
void handleMotorArmStateAndEepromError();
void vtxInit();
void vtxSetFrequencyMhz(uint16_t mhz, bool send_now, bool save_eeprom);
void vtxSetPowerMw(uint16_t mw, bool send_now, bool save_eeprom);
void vtxSetFrequencyAndPower(uint16_t mhz, uint16_t power_mw, bool save_eeprom);
void vtxStepNonBlocking(uint32_t now_ms);
void startMagCalibrationNonBlocking();
void stepMagCalibrationNonBlocking(uint32_t now_ms);
void stepMonitorStreamNonBlocking(uint32_t now_ms);
void stopBenchTest();
void stepBenchTestNonBlocking(uint32_t now_ms, float gx_rad, float gy_rad, float gz_rad, float &m1, float &m2, float &m3, float &m4);

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

FlightConfig currentConfig;
Quaternion orientation;

SPIClass rtxSpi(RTX_MOSI_PIN, RTX_MISO_PIN, RTX_SCK_PIN);
SPISettings rtxSpiSettings(8000000, MSBFIRST, SPI_MODE0);

uint8_t fc_error_ram = FC_ERR_NONE;
uint8_t fc_latched_error_ram = FC_ERR_NONE;
uint8_t last_sent_202_err = FC_ERR_NONE;
uint32_t last_202_send_ms = 0;

bool prev_armed_state = false;

DelayedRtxRespItem rtx_resp_queue[RTX_RESP_QUEUE_SIZE];
uint8_t rtx_resp_head = 0;
uint8_t rtx_resp_tail = 0;
uint8_t rtx_resp_count = 0;

float cpu_load_pct = 0.0f;
float cpu_max_load_pct = 0.0f;
float vibration_index = 0.0f;

float last_cal_gx_dps = 0.0f, last_cal_gy_dps = 0.0f, last_cal_gz_dps = 0.0f;
float last_cal_ax_g = 0.0f, last_cal_ay_g = 0.0f, last_cal_az_g = 1.0f;

bool mon_stream_active = false;
uint32_t mon_last_send_ms = 0;

BenchTestMode bench_test_mode = BENCH_TEST_NONE;
uint32_t bench_test_start_ms = 0;

float prev_m1_out = 0.0f, prev_m2_out = 0.0f, prev_m3_out = 0.0f, prev_m4_out = 0.0f;

LowPassFilter lpf_ax, lpf_ay, lpf_az;
LowPassFilter lpf_gx, lpf_gy, lpf_gz;
LowPassFilter lpf_xy_hp_x{0.15f}, lpf_xy_hp_y{0.15f};
LowPassFilter lpf_temp{0.05f};
LowPassFilter lpf_battery{0.05f};
LowPassFilter lpf_current{0.15f};
LowPassFilter lpf_baro_alt{0.12f};
LowPassFilter lpf_baro_vz{0.25f};
AdaptiveLidarFilter adaptive_lidar;
BiquadNotchFilter notch_gx, notch_gy, notch_gz;

float base_hover_throttle = INITIAL_BASE_HOVER;
float current_vz = 0.0f;
float baro_alt_m = 0.0f;
float baro_ground_alt_m = 0.0f;
bool baro_cal_ready = false;
float baro_vz_mps = 0.0f;

float v_batt = 14.8f;
bool batt_valid = false;
float current_a = 0.0f;
float power_w = 0.0f;
float consumed_wh = 0.0f;
uint8_t current_link_quality = 100;
float current_chip_temp = DEFAULT_BASE_TEMP;

bool vtx_uart_ready = false;
bool vtx_hw_unavailable = false;
bool vtx_initial_probe_done = false;
bool vtx_send_freq_req = false;
bool vtx_send_power_req = false;
bool vtx_link_ok = false;
uint8_t vtx_detected_proto = VTX_PROTO_NONE;
uint16_t vtx_actual_mw = 0;
int8_t vtx_actual_dbm = 0;
uint32_t vtx_last_tx_ms = 0;
uint32_t vtx_last_probe_ms = 0;
uint32_t vtx_baud = 0;
VtxProbeStep vtx_probe_step = VTX_STEP_IDLE;
uint32_t vtx_step_start_ms = 0;
uint8_t tramp_rx_buf[16];
uint8_t tramp_rx_idx = 0;
uint8_t sa_rx_buf[16];
uint8_t sa_rx_idx = 0;

uint8_t rtx_active_chip_id = RTX_CHIP_SX1262;
bool rtx_recovering = false;

bool mag_cal_active = false;
uint32_t mag_cal_start_ms = 0;
uint32_t mag_cal_last_sample_ms = 0;
float mag_cal_min_x = 1e9f, mag_cal_max_x = -1e9f;
float mag_cal_min_y = 1e9f, mag_cal_max_y = -1e9f;
float mag_cal_min_z = 1e9f, mag_cal_max_z = -1e9f;

float compass_heading_deg = 0.0f;
bool compass_valid = false;

uint8_t i2c_error_counter = 0;
uint8_t rtx_error_counter = 0;
uint8_t rtx_noise_floor[100];
uint8_t rtx_scan_cursor = 0;
uint8_t rtx_best_clean_chan = DEFAULT_RTX_PRI_CHAN;
bool rtx_scan_active = false;
bool rtx_switched_flag = false;
uint8_t rtx_new_switched_chan = DEFAULT_RTX_PRI_CHAN;

uint8_t main_rtx_chan = DEFAULT_RTX_PRI_CHAN;
uint8_t active_rtx_chan = DEFAULT_RTX_PRI_CHAN;
bool is_on_backup = false;
bool pending_channel_switch = false;
uint8_t target_pending_chan = DEFAULT_RTX_PRI_CHAN;

bool rtx_tx_in_flight = false;
uint32_t rtx_tx_start_ms = 0;
uint32_t rtx_tx_timeout_ms = RTX_TX_TIMEOUT_PRI_MS;
bool rtx_apply_switch_after_tx = false;
uint8_t rtx_post_tx_chan = DEFAULT_RTX_PRI_CHAN;
bool rtx_post_tx_backup = false;

PID vzPID, rollRatePID, pitchRatePID, yawRatePID;
HardwareTimer tim2(TIM2);

bool is_armed = false;
bool is_failsafe = false;
float arm_ramp_factor = 0.0f;
float smooth_throttle = 0.0f, smooth_roll = STICK_CENTER, smooth_pitch = STICK_CENTER, smooth_yaw = STICK_CENTER;
uint8_t mpu_dma_buffer[14];
uint32_t last_rc_packet_ms = 0;
bool send_telemetry_pending = false;

bool err_rc_loss = false;
bool err_batt_low = false;
bool err_imu_fail = false;
bool err_aux_i2c_fail = false;

float dbg_lidar_m = 0.0f;
uint32_t last_lidar_packet_ms = 0;

// ==============================================================================
// 5. ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ОШИБОК, КОНФИГУРАЦИИ И EEPROM
// ==============================================================================
inline float roundToTenthVolt(float v) {
  return (float)lroundf(v * 10.0f) * 0.1f;
}

inline float getEffectiveEscDeltaLimit() {
  if (isnan(currentConfig.flt_esc_limit) || isinf(currentConfig.flt_esc_limit)) return DEFAULT_FLT_ESC_LIMIT;
  return constrain(currentConfig.flt_esc_limit, 1.0f, 100.0f);
}

inline float getEffectiveBatVMax() {
  if (isnan(currentConfig.bat_v_max) || isinf(currentConfig.bat_v_max)) return DEFAULT_BAT_V_MAX;
  return roundToTenthVolt(constrain(currentConfig.bat_v_max, 7.0f, 25.0f));
}

inline float getBatteryCompensationFactor() {
  if (!currentConfig.bat_comp_en || !batt_valid) return 1.0f;
  if (v_batt <= 2.0f || isnan(v_batt) || isinf(v_batt)) return 1.0f;
  float v_max = getEffectiveBatVMax();
  float sag_factor = v_max / v_batt;
  return constrain(sag_factor, 0.95f, 1.35f);
}

inline bool isCriticalErrorCode(uint8_t err_code) {
  switch (err_code) {
    case FC_ERR_IMU_FAIL:
    case FC_ERR_RC_LOSS:
    case FC_ERR_BAT_LOW:
    case FC_ERR_TILT_DISARM:
    case FC_ERR_NAN_INF_MATH:
      return true;
    default:
      return false;
  }
}

inline bool hasBlockingArmError(uint8_t err_code) {
  return (err_code == FC_ERR_IMU_FAIL || err_code == FC_ERR_CALIB_MISSING || err_code == FC_ERR_NAN_INF_MATH);
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
  if (fc_error_ram == FC_ERR_NONE) {
    last_sent_202_err = FC_ERR_NONE;
    return;
  }
  if (isCriticalErrorCode(fc_error_ram) && (now_ms - last_202_send_ms) >= ERR_202_REPEAT_MS) {
    uint8_t flags = 0x01 | (is_armed ? 0x02 : 0x00);
    queueDelayedRtxResponse(RTX_RESP_202_ERR_EVENT, fc_error_ram, flags, 0);
    last_sent_202_err = fc_error_ram;
    last_202_send_ms = now_ms;
  }
}

void handleMotorArmStateAndEepromError() {
  if (!prev_armed_state && is_armed) {
    stopBenchTest();
    CliAutoCal::onArmRequested();
    if (hasBlockingArmError(fc_error_ram) || hasBlockingArmError(fc_latched_error_ram)) {
      is_armed = false;
      resetMotorSlewState();
      setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);
      prev_armed_state = false;
      return;
    }
    fc_error_ram = FC_ERR_NONE;
    fc_latched_error_ram = FC_ERR_NONE;
    last_sent_202_err = FC_ERR_NONE;
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
  lpf_gx.alpha = currentConfig.gyro_filter_alpha;
  lpf_gy.alpha = currentConfig.gyro_filter_alpha;
  lpf_gz.alpha = currentConfig.gyro_filter_alpha;
  lpf_ax.alpha = currentConfig.acc_filter_alpha;
  lpf_ay.alpha = currentConfig.acc_filter_alpha;
  lpf_az.alpha = currentConfig.acc_filter_alpha;
  if (!currentConfig.flt_lpf_en) {
    lpf_gx.reset(); lpf_gy.reset(); lpf_gz.reset();
    lpf_ax.reset(); lpf_ay.reset(); lpf_az.reset();
  }
  lpf_battery.alpha = currentConfig.bat_filter_alpha;
  notch_gx.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  notch_gy.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  notch_gz.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  if (!currentConfig.flt_not_en) {
    notch_gx.reset(); notch_gy.reset(); notch_gz.reset();
  }
}

void updatePidTunings() {
  rollRatePID.updateConfig(currentConfig.roll_pid);
  pitchRatePID.updateConfig(currentConfig.pitch_pid);
  yawRatePID.updateConfig(currentConfig.yaw_pid);
  vzPID.updateConfig(currentConfig.vz_pid);
}

void resetToFactoryDefaults() {
  memset(&currentConfig, 0, sizeof(currentConfig));
  currentConfig.magic_marker = EEPROM_MAGIC_MARKER;
  currentConfig.esc_prot = DEFAULT_ESC_PROT;
  currentConfig.imu_align = DEFAULT_IMU_ALIGN;
  currentConfig.com_align = DEFAULT_COM_ALIGN;
  currentConfig.imu_id = DEFAULT_IMU_ID;
  currentConfig.bar_id = DEFAULT_BAR_ID;
  currentConfig.com_id = DEFAULT_COM_ID;
  currentConfig.lid_id = DEFAULT_LID_ID;
  currentConfig.rtx_id = DEFAULT_RTX_ID;

  currentConfig.vtx_en = DEFAULT_VTX_EN;
  currentConfig.vtx_prot = DEFAULT_VTX_PROT;
  currentConfig.vtx_mhz = DEFAULT_VTX_MHZ;
  currentConfig.vtx_mw = DEFAULT_VTX_MW;

  currentConfig.gyro_bias_x = DEFAULT_GYRO_BIAS_X;
  currentConfig.gyro_bias_y = DEFAULT_GYRO_BIAS_Y;
  currentConfig.gyro_bias_z = DEFAULT_GYRO_BIAS_Z;
  currentConfig.base_temperature = DEFAULT_BASE_TEMP;
  currentConfig.temp_coeff_x = DEFAULT_TEMP_COEFF_X;
  currentConfig.temp_coeff_y = DEFAULT_TEMP_COEFF_Y;
  currentConfig.temp_coeff_z = DEFAULT_TEMP_COEFF_Z;
  currentConfig.acc_bias_x = DEFAULT_ACC_BIAS_X;
  currentConfig.acc_bias_y = DEFAULT_ACC_BIAS_Y;
  currentConfig.acc_bias_z = DEFAULT_ACC_BIAS_Z;
  currentConfig.mag_bias_x = DEFAULT_MAG_BIAS_X;
  currentConfig.mag_bias_y = DEFAULT_MAG_BIAS_Y;
  currentConfig.mag_bias_z = DEFAULT_MAG_BIAS_Z;
  currentConfig.lidar_offset_m = DEFAULT_LIDAR_OFFSET_M;
  currentConfig.bat_scale_v = DEFAULT_BAT_SCALE_V;
  currentConfig.bat_offset_a = DEFAULT_BAT_OFFSET_A;
  currentConfig.bat_scale_a = DEFAULT_BAT_SCALE_A;
  currentConfig.bat_comp_en = DEFAULT_BAT_COMP_EN;
  currentConfig.bat_v_max = DEFAULT_BAT_V_MAX;
  currentConfig.flt_esc_limit = DEFAULT_FLT_ESC_LIMIT;
  currentConfig.fc_error = DEFAULT_FC_ERROR;

  currentConfig.fm_mode = DEFAULT_FM_MODE;
  currentConfig.fm_roll_max = DEFAULT_FM_ROLL_MAX;
  currentConfig.fm_pitch_max = DEFAULT_FM_PITCH_MAX;
  currentConfig.fm_vz_speed_max = DEFAULT_FM_VZ_SPEED_MAX;

  currentConfig.rc_rate = DEFAULT_RC_RATE;
  currentConfig.rc_super_expo = DEFAULT_RC_SUPER_EXPO;
  currentConfig.rc_stick_expo = DEFAULT_RC_STICK_EXPO;
  currentConfig.rc_yaw_rate = DEFAULT_RC_YAW_RATE;
  currentConfig.rc_yaw_super_expo = DEFAULT_RC_YAW_SUPER_EXPO;
  currentConfig.rc_yaw_expo = DEFAULT_RC_YAW_EXPO;
  currentConfig.rc_deadband = DEFAULT_RC_DEADBAND;
  currentConfig.rc_horizon_transition = DEFAULT_RC_HORIZON_TRANS;
  currentConfig.rc_filter_smoothing = DEFAULT_RC_SMOOTH;

  currentConfig.fs_bat_en = DEFAULT_FS_BAT_EN;
  currentConfig.fs_rtx_en = DEFAULT_FS_RTX_EN;
  currentConfig.fs_vz_speed = DEFAULT_FS_VZ_SPEED;

  currentConfig.bat_fs_v = DEFAULT_BAT_FS_V;
  currentConfig.bat_filter_alpha = DEFAULT_BAT_FILTER_ALPHA;

  currentConfig.lidar_en = DEFAULT_LIDAR_EN;
  currentConfig.lidar_filter_alpha = DEFAULT_LIDAR_FIL_ALPHA;
  currentConfig.baro_en = DEFAULT_BARO_EN;
  currentConfig.com_en = DEFAULT_COM_EN;

  currentConfig.flt_amb_en = DEFAULT_FLT_AMB_EN;
  currentConfig.flt_lpf_en = DEFAULT_FLT_LPF_EN;
  currentConfig.flt_not_en = DEFAULT_FLT_NOT_EN;
  currentConfig.gyro_filter_alpha = DEFAULT_GYRO_FILTER_ALPHA;
  currentConfig.acc_filter_alpha = DEFAULT_ACC_FILTER_ALPHA;
  currentConfig.filter_notch_freq = DEFAULT_NOTCH_FREQ;
  currentConfig.filter_notch_bw = DEFAULT_NOTCH_BW;
  currentConfig.flt_amb = DEFAULT_FLT_AMB;
  currentConfig.flt_amb_min = DEFAULT_FLT_AMB_MIN;

  currentConfig.rtx_en = DEFAULT_RTX_EN;
  currentConfig.rtx_band = DEFAULT_RTX_BAND;
  currentConfig.rtx_hw_crc_en = DEFAULT_RTX_HW_CRC;
  currentConfig.rtx_addr_en = DEFAULT_RTX_ADDR_EN;
  currentConfig.rtx_addr = DEFAULT_RTX_ADDR;
  currentConfig.rtx_sync_word = DEFAULT_RTX_SYNC;
  currentConfig.rtx_iq_inv = DEFAULT_RTX_IQ_INV;
  currentConfig.rtx_pri_chan = DEFAULT_RTX_PRI_CHAN;
  currentConfig.rtx_pri_sf = DEFAULT_RTX_PRI_SF;
  currentConfig.rtx_pri_bw = DEFAULT_RTX_PRI_BW;
  currentConfig.rtx_pri_cr = DEFAULT_RTX_PRI_CR;
  currentConfig.rtx_pri_power = DEFAULT_RTX_PRI_POWER;
  currentConfig.rtx_bak_chan = DEFAULT_RTX_BAK_CHAN;
  currentConfig.rtx_bak_sf = DEFAULT_RTX_BAK_SF;
  currentConfig.rtx_bak_bw = DEFAULT_RTX_BAK_BW;
  currentConfig.rtx_bak_cr = DEFAULT_RTX_BAK_CR;
  currentConfig.rtx_bak_power = DEFAULT_RTX_BAK_POWER;

  currentConfig.roll_pid = {PID_ROLL_KP_DEF, PID_ROLL_KI_DEF, PID_ROLL_KD_DEF};
  currentConfig.pitch_pid = {PID_PITCH_KP_DEF, PID_PITCH_KI_DEF, PID_PITCH_KD_DEF};
  currentConfig.yaw_pid = {PID_YAW_KP_DEF, PID_YAW_KI_DEF, PID_YAW_KD_DEF};
  currentConfig.vz_pid = {PID_VZ_KP_DEF, PID_VZ_KI_DEF, PID_VZ_KD_DEF};

  vtx_link_ok = false;
  vtx_detected_proto = VTX_PROTO_NONE;
  main_rtx_chan = currentConfig.rtx_pri_chan;
  active_rtx_chan = main_rtx_chan;
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
  EEPROM.get(0, currentConfig);
  if (currentConfig.magic_marker != EEPROM_MAGIC_MARKER) {
    resetToFactoryDefaults();
    setFcErrorRam(FC_ERR_EEPROM_RESET);
    return;
  }
  if (currentConfig.imu_id > 6) currentConfig.imu_id = DEFAULT_IMU_ID;
  if (currentConfig.bar_id > 7) currentConfig.bar_id = DEFAULT_BAR_ID;
  if (currentConfig.com_id > 6) currentConfig.com_id = DEFAULT_COM_ID;
  if (currentConfig.lid_id > 4) currentConfig.lid_id = DEFAULT_LID_ID;
  if (currentConfig.bat_comp_en > 1) currentConfig.bat_comp_en = DEFAULT_BAT_COMP_EN;
  if (isnan(currentConfig.bat_v_max) || isinf(currentConfig.bat_v_max)) {
    currentConfig.bat_v_max = DEFAULT_BAT_V_MAX;
  }
  currentConfig.bat_v_max = getEffectiveBatVMax();

  if (isnan(currentConfig.flt_esc_limit) || isinf(currentConfig.flt_esc_limit)) {
    currentConfig.flt_esc_limit = DEFAULT_FLT_ESC_LIMIT;
  }
  currentConfig.flt_esc_limit = getEffectiveEscDeltaLimit();

  if (currentConfig.vtx_en > 1) currentConfig.vtx_en = DEFAULT_VTX_EN;
  if (currentConfig.vtx_prot > VTX_PROT_SMARTAUDIO) currentConfig.vtx_prot = DEFAULT_VTX_PROT;
  if (currentConfig.vtx_mhz < 5000 || currentConfig.vtx_mhz > 6000) currentConfig.vtx_mhz = DEFAULT_VTX_MHZ;
  if (currentConfig.vtx_mw == 0 || currentConfig.vtx_mw > 10000) currentConfig.vtx_mw = DEFAULT_VTX_MW;
  if (currentConfig.rtx_id > RTX_CHIP_LLCC68) currentConfig.rtx_id = DEFAULT_RTX_ID;
  if (currentConfig.flt_amb_en > 1) currentConfig.flt_amb_en = DEFAULT_FLT_AMB_EN;
  if (currentConfig.flt_lpf_en > 1) currentConfig.flt_lpf_en = DEFAULT_FLT_LPF_EN;
  if (currentConfig.flt_not_en > 1) currentConfig.flt_not_en = DEFAULT_FLT_NOT_EN;
  if (isnan(currentConfig.flt_amb) || currentConfig.flt_amb <= 0.0f) {
    currentConfig.flt_amb = DEFAULT_FLT_AMB;
  }
  if (isnan(currentConfig.flt_amb_min) || currentConfig.flt_amb_min < 0.0f) {
    currentConfig.flt_amb_min = DEFAULT_FLT_AMB_MIN;
  }

  rollRatePID.init(currentConfig.roll_pid, 200.0f);
  pitchRatePID.init(currentConfig.pitch_pid, 200.0f);
  yawRatePID.init(currentConfig.yaw_pid, 200.0f);
  vzPID.init(currentConfig.vz_pid, 300.0f);
  if (currentConfig.fc_error != FC_ERR_NONE) {
    fc_latched_error_ram = currentConfig.fc_error;
  }
}

void saveSettings() {
  if (is_armed || CliAutoCal::isFlightActive()) return;
  currentConfig.bat_v_max = getEffectiveBatVMax();
  currentConfig.flt_esc_limit = getEffectiveEscDeltaLimit();
  currentConfig.magic_marker = EEPROM_MAGIC_MARKER;
  EEPROM.put(0, currentConfig);
}

// ==============================================================================
// 6. РАДИОКАНАЛ RTX (SPI2 / SX126x / LLCC68)
// ==============================================================================
inline bool hasQueuedRtxResponse() {
  return (rtx_resp_count > 0) && (rtx_resp_queue[rtx_resp_head].type != RTX_RESP_NONE);
}

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
  if (rtx_resp_count == 0) {
    out.type = RTX_RESP_NONE;
    return false;
  }
  out = rtx_resp_queue[rtx_resp_head];
  rtx_resp_queue[rtx_resp_head].type = RTX_RESP_NONE;
  rtx_resp_head = (rtx_resp_head + 1) % RTX_RESP_QUEUE_SIZE;
  rtx_resp_count--;
  return true;
}

uint32_t getTelemetryTxTimeoutMs(bool backup_mode) {
  uint8_t sf = backup_mode ? constrain(currentConfig.rtx_bak_sf, 5, 12) : constrain(currentConfig.rtx_pri_sf, 5, 12);
  uint32_t base_timeout = backup_mode ? RTX_TX_TIMEOUT_BAK_MS : RTX_TX_TIMEOUT_PRI_MS;
  if (sf >= 11) base_timeout += 140;
  else if (sf >= 10) base_timeout += 70;
  return base_timeout;
}

inline void rtx_spi_begin_tx() {
  rtxSpi.beginTransaction(rtxSpiSettings);
  digitalWrite(RTX_NSS_PIN, LOW);
}

inline void rtx_spi_end_tx() {
  digitalWrite(RTX_NSS_PIN, HIGH);
  rtxSpi.endTransaction();
}

void checkRtxBusAndRecover() {
  if (rtx_recovering || rtx_error_counter < RTX_MAX_ERRORS) return;
  rtx_recovering = true;
  setFcErrorRam(FC_ERR_SPI_RTX);
  digitalWrite(RTX_RESET_PIN, LOW);
  uint32_t start = micros();
  while ((micros() - start) < 2000);
  digitalWrite(RTX_RESET_PIN, HIGH);
  rtx_tx_in_flight = false;
  rtx_apply_switch_after_tx = false;
  rtx_scan_active = false;
  rtx_error_counter = 0;
  applyRtxConfiguration(is_on_backup);
  rtx_recovering = false;
}

inline bool rtx_try_ready_short(uint32_t timeout_us = RTX_BUSY_SHORT_TIMEOUT_US) {
  uint32_t start = micros();
  while (digitalRead(RTX_BUSY_PIN) == HIGH) {
    if ((micros() - start) >= timeout_us) {
      rtx_error_counter++;
      setFcErrorRam(FC_ERR_SPI_RTX);
      checkRtxBusAndRecover();
      return false;
    }
  }
  return true;
}

void rtx_write_command(uint8_t cmd, const uint8_t *data, uint8_t len) {
  if (!rtx_try_ready_short()) return;
  rtx_spi_begin_tx();
  rtxSpi.transfer(cmd);
  for (uint8_t i = 0; i < len; i++) rtxSpi.transfer(data[i]);
  rtx_spi_end_tx();
  rtx_error_counter = 0;
}

void rtx_read_command(uint8_t cmd, uint8_t *data, uint8_t len) {
  if (!rtx_try_ready_short()) return;
  rtx_spi_begin_tx();
  rtxSpi.transfer(cmd);
  rtxSpi.transfer(0x00);
  for (uint8_t i = 0; i < len; i++) data[i] = rtxSpi.transfer(0x00);
  rtx_spi_end_tx();
  rtx_error_counter = 0;
}

static uint8_t detectOrSelectRtxChip() {
  uint8_t req = currentConfig.rtx_id;
  if (req == RTX_CHIP_SX1262 || req == RTX_CHIP_SX1268 || req == RTX_CHIP_LLCC68) {
    return req;
  }
  uint8_t st[2] = {0, 0};
  rtx_read_command(0xC0, st, 2);
  return (currentConfig.rtx_band <= 5) ? RTX_CHIP_SX1268 : RTX_CHIP_SX1262;
}

void rtx_init_hardware() {
  pinMode(RTX_NSS_PIN, OUTPUT);
  pinMode(RTX_RESET_PIN, OUTPUT);
  pinMode(RTX_BUSY_PIN, INPUT);
  digitalWrite(RTX_NSS_PIN, HIGH);
  rtxSpi.begin();
  digitalWrite(RTX_RESET_PIN, LOW);
  delay(5);
  digitalWrite(RTX_RESET_PIN, HIGH);
  delay(10);
  rtx_tx_in_flight = false;
  rtx_apply_switch_after_tx = false;
  rtx_scan_active = false;
  rtx_active_chip_id = detectOrSelectRtxChip();
}

void applyRtxConfiguration(bool use_backup_settings) {
  if (!currentConfig.rtx_en || !rtx_try_ready_short()) return;
  rtx_tx_in_flight = false;
  rtx_apply_switch_after_tx = false;
  rtx_scan_active = false;
  rtx_active_chip_id = detectOrSelectRtxChip();

  uint8_t mode_stby = 0x00;
  rtx_write_command(0x80, &mode_stby, 1);

  uint8_t reg_mode = 0x01;
  rtx_write_command(0x96, &reg_mode, 1);

  uint8_t pkt_type = 0x01;
  rtx_write_command(0x8A, &pkt_type, 1);

  if (currentConfig.rtx_addr_en) {
    uint8_t addr_buf[2] = {(uint8_t)(currentConfig.rtx_addr >> 8), (uint8_t)(currentConfig.rtx_addr & 0xFF)};
    rtx_write_command(0x6C, addr_buf, 2);
  }
  uint8_t sync_buf[2] = {(uint8_t)(currentConfig.rtx_sync_word >> 8), (uint8_t)(currentConfig.rtx_sync_word & 0xFF)};
  rtx_write_command(0x74, sync_buf, 2);

  uint8_t band = constrain(currentConfig.rtx_band, 1, 9);
  uint32_t base_freq_hz = (uint32_t)band * 100000000UL;
  uint8_t channel = use_backup_settings ? constrain(currentConfig.rtx_bak_chan, 0, 99) : constrain(active_rtx_chan, 0, 99);
  uint32_t freq_reg = (uint32_t)(((uint64_t)(base_freq_hz + ((uint32_t)channel * 1000000UL)) << 25) / 32000000UL);
  uint8_t freq_buf[4] = {(uint8_t)(freq_reg >> 24), (uint8_t)(freq_reg >> 16), (uint8_t)(freq_reg >> 8), (uint8_t)freq_reg};
  rtx_write_command(0x86, freq_buf, 4);

  uint8_t sf = use_backup_settings ? constrain(currentConfig.rtx_bak_sf, 5, 12) : constrain(currentConfig.rtx_pri_sf, 5, 12);
  uint8_t bw = use_backup_settings ? constrain(currentConfig.rtx_bak_bw, 0, 9) : constrain(currentConfig.rtx_pri_bw, 0, 9);
  if (rtx_active_chip_id == RTX_CHIP_LLCC68) sf = constrain(sf, 5, 11);
  uint8_t cr = use_backup_settings ? constrain(currentConfig.rtx_bak_cr, 1, 4) : constrain(currentConfig.rtx_pri_cr, 1, 4);
  uint8_t mod_params[4] = {sf, bw, cr, 0x00};
  rtx_write_command(0x8B, mod_params, 4);

  uint8_t pkt_params[6] = {
    0x00, 0x08, 0x00, RTX_RX_MAX_PAYLOAD_BYTES,
    (uint8_t)(currentConfig.rtx_hw_crc_en ? 0x01 : 0x00),
    (uint8_t)(currentConfig.rtx_iq_inv ? 0x01 : 0x00)
  };
  rtx_write_command(0x8C, pkt_params, 6);

  int8_t pwr_dbm = use_backup_settings ? (int8_t)constrain(currentConfig.rtx_bak_power, -9, 22) : (int8_t)constrain(currentConfig.rtx_pri_power, -9, 22);
  uint8_t pa_cfg[4] = {0x04, 0x07, 0x00, 0x01};
  rtx_write_command(0x95, pa_cfg, 4);
  uint8_t tx_pwr[2] = {(uint8_t)pwr_dbm, 0x02};
  rtx_write_command(0x8E, tx_pwr, 2);

  rtx_set_rx_mode();
}

void rtx_set_rx_mode() {
  if (!currentConfig.rtx_en) return;
  uint8_t rx_params[3] = {0xFF, 0xFF, 0xFF};
  rtx_write_command(0x82, rx_params, 3);
}

void startChannelScanNonBlocking() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_recovering) return;
  rtx_scan_active = true;
  rtx_scan_cursor = 0;
}

void stepChannelScanNonBlocking() {
  if (!rtx_scan_active || !currentConfig.rtx_en || rtx_tx_in_flight || rtx_recovering) return;
  if (digitalRead(RTX_BUSY_PIN) == HIGH) return;

  uint8_t band = constrain(currentConfig.rtx_band, 1, 9);
  uint32_t base_freq_hz = (uint32_t)band * 100000000UL;
  uint32_t base_reg = (uint32_t)(((uint64_t)base_freq_hz << 25) / 32000000UL);
  uint32_t step_reg = (uint32_t)(((uint64_t)1000000UL << 25) / 32000000UL);

  uint8_t end_chan = rtx_scan_cursor + RTX_SCAN_CHANNELS_PER_TICK;
  if (end_chan > 100) end_chan = 100;
  uint32_t freq_reg = base_reg + ((uint32_t)rtx_scan_cursor * step_reg);

  for (uint8_t c = rtx_scan_cursor; c < end_chan; c++) {
    uint32_t w_start = micros();
    while (digitalRead(RTX_BUSY_PIN) == HIGH) {
      if ((micros() - w_start) > RTX_SCAN_BUSY_TIMEOUT_US) {
        setFcErrorRam(FC_ERR_SPI_RTX); rtx_scan_active = false; applyRtxConfiguration(is_on_backup); return;
      }
    }
    uint8_t freq_buf[4] = {(uint8_t)(freq_reg >> 24), (uint8_t)(freq_reg >> 16), (uint8_t)(freq_reg >> 8), (uint8_t)freq_reg};
    rtx_spi_begin_tx(); rtxSpi.transfer(0x86);
    for (uint8_t i = 0; i < 4; i++) rtxSpi.transfer(freq_buf[i]);
    rtx_spi_end_tx();

    uint32_t r_start = micros();
    while (digitalRead(RTX_BUSY_PIN) == HIGH) {
      if ((micros() - r_start) > RTX_SCAN_BUSY_TIMEOUT_US) {
        setFcErrorRam(FC_ERR_SPI_RTX); rtx_scan_active = false; applyRtxConfiguration(is_on_backup); return;
      }
    }
    rtx_spi_begin_tx(); rtxSpi.transfer(0x15); rtxSpi.transfer(0x00);
    rtx_noise_floor[c] = rtxSpi.transfer(0x00);
    rtx_spi_end_tx();
    freq_reg += step_reg;
  }

  rtx_scan_cursor = end_chan;
  if (rtx_scan_cursor >= 100) {
    uint8_t best_chan = 0, min_noise = 255;
    for (uint8_t i = 0; i < 100; i++) {
      if (rtx_noise_floor[i] < min_noise) { min_noise = rtx_noise_floor[i]; best_chan = i; }
    }
    rtx_best_clean_chan = best_chan;
    rtx_scan_active = false;
    queueDelayedRtxResponse(RTX_RESP_230_SCAN_BEST, rtx_best_clean_chan, 0, 0);

    uint32_t work_freq_reg = base_reg + ((uint32_t)constrain(active_rtx_chan, 0, 99) * step_reg);
    uint8_t work_freq_buf[4] = {(uint8_t)(work_freq_reg >> 24), (uint8_t)(work_freq_reg >> 16), (uint8_t)(work_freq_reg >> 8), (uint8_t)work_freq_reg};
    rtx_write_command(0x86, work_freq_buf, 4);
    rtx_set_rx_mode();
  }
}

void readRx() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_scan_active || rtx_recovering) return;
  if (digitalRead(RTX_BUSY_PIN) == HIGH) return;

  uint8_t irq_status[2] = {0, 0};
  rtx_read_command(0x12, irq_status, 2);
  uint16_t irq = ((uint16_t)irq_status[0] << 8) | irq_status[1];
  if (irq & 0x02) {
    uint8_t clear_irq[2] = {0x00, 0x02};
    rtx_write_command(0x02, clear_irq, 2);
    if (!rtx_try_ready_short()) { rtx_set_rx_mode(); return; }

    uint8_t rx_buf_status[2] = {0, 0};
    rtx_read_command(0x13, rx_buf_status, 2);
    uint8_t payload_len = rx_buf_status[0];
    uint8_t start_ptr = rx_buf_status[1];
    if (payload_len == 0 || payload_len > RTX_RX_MAX_PAYLOAD_BYTES) { rtx_set_rx_mode(); return; }

    rtx_spi_begin_tx(); rtxSpi.transfer(0x1E); rtxSpi.transfer(start_ptr);
    uint8_t raw_buf[RTX_RX_MAX_PAYLOAD_BYTES] = {0};
    for (uint8_t i = 0; i < payload_len; i++) raw_buf[i] = rtxSpi.transfer(0x00);
    rtx_spi_end_tx();

    uint8_t idx = 0;
    if (currentConfig.rtx_addr_en) {
      if (payload_len < (idx + 2)) { rtx_set_rx_mode(); return; }
      uint16_t pkt_adr = ((uint16_t)raw_buf[idx] << 8) | raw_buf[idx + 1];
      idx += 2;
      if (pkt_adr != currentConfig.rtx_addr) { rtx_set_rx_mode(); return; }
    }
    if (payload_len < (uint8_t)(idx + 2)) { rtx_set_rx_mode(); return; }

    uint8_t cmd_type = raw_buf[idx++];
    uint8_t lq_val = raw_buf[idx++];
    uint8_t rem_bytes = payload_len - idx;

    switch (cmd_type) {
      case 110: {
        if (rem_bytes < 4) { rtx_set_rx_mode(); return; }
        current_link_quality = lq_val;
        float t_val = (float)raw_buf[idx++] * 10.0f;
        float r_val = (float)raw_buf[idx++] * 10.0f;
        float p_val = (float)raw_buf[idx++] * 10.0f;
        float y_val = (float)raw_buf[idx++] * 10.0f;
        float alpha_rc = constrain(currentConfig.rc_filter_smoothing, 0.01f, 1.0f);
        smooth_throttle = alpha_rc * t_val + (1.0f - alpha_rc) * smooth_throttle;
        smooth_roll = alpha_rc * r_val + (1.0f - alpha_rc) * smooth_roll;
        smooth_pitch = alpha_rc * p_val + (1.0f - alpha_rc) * smooth_pitch;
        smooth_yaw = alpha_rc * y_val + (1.0f - alpha_rc) * smooth_yaw;
        last_rc_packet_ms = millis();
        send_telemetry_pending = true;
        break;
      }
      case 120: {
        if (rem_bytes < 1) { rtx_set_rx_mode(); return; }
        current_link_quality = lq_val;
        bool req_arm = (raw_buf[idx++] > 0);
        if (!req_arm) { CliAutoCal::onDisarmRequested(); is_armed = false; }
        else if (!is_failsafe) { is_armed = true; }
        last_rc_packet_ms = millis();
        break;
      }
      case 130: {
        current_link_quality = lq_val;
        startChannelScanNonBlocking();
        last_rc_packet_ms = millis();
        break;
      }
      case 140: {
        if (rem_bytes < 1) { rtx_set_rx_mode(); return; }
        current_link_quality = lq_val;
        uint8_t target_chan = constrain(raw_buf[idx++], 0, 99);
        target_pending_chan = target_chan; pending_channel_switch = true;
        rtx_switched_flag = true; rtx_new_switched_chan = target_chan;
        queueDelayedRtxResponse(RTX_RESP_240_CHAN_SW, target_chan, 0, 0);
        last_rc_packet_ms = millis();
        break;
      }
      case 150: {
        if (rem_bytes < 4) { rtx_set_rx_mode(); return; }
        current_link_quality = lq_val;
        uint16_t req_mhz = ((uint16_t)raw_buf[idx] << 8) | raw_buf[idx + 1];
        uint16_t pwr_mw = ((uint16_t)raw_buf[idx + 2] << 8) | raw_buf[idx + 3];
        vtxSetFrequencyAndPower(req_mhz, pwr_mw, true);
        queueDelayedRtxResponse(RTX_RESP_250_VTX_ACK, currentConfig.vtx_mhz, 0, currentConfig.vtx_mw);
        last_rc_packet_ms = millis();
        break;
      }
      default: break;
    }
  }
  rtx_set_rx_mode();
}

static uint8_t getActiveStatusErrorCode() {
  return (fc_error_ram != FC_ERR_NONE) ? fc_error_ram : fc_latched_error_ram;
}

void startTelemetryNonBlocking() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_scan_active || rtx_recovering) return;
  if (digitalRead(RTX_BUSY_PIN) == HIGH) return;

  uint8_t stby = 0x00; rtx_write_command(0x80, &stby, 1);
  if (!rtx_try_ready_short()) return;

  uint8_t tx_payload[32]; uint8_t p_idx = 0;
  if (currentConfig.rtx_addr_en) {
    tx_payload[p_idx++] = (uint8_t)(currentConfig.rtx_addr >> 8);
    tx_payload[p_idx++] = (uint8_t)(currentConfig.rtx_addr & 0xFF);
  }

  DelayedRtxRespItem q_item;
  if (hasQueuedRtxResponse() && popDelayedRtxResponse(q_item)) {
    tx_payload[p_idx++] = (uint8_t)q_item.type;
    tx_payload[p_idx++] = current_link_quality;
    if (q_item.type == RTX_RESP_202_ERR_EVENT) {
      tx_payload[p_idx++] = (uint8_t)q_item.vtx_mhz;
      tx_payload[p_idx++] = q_item.tier;
    } else if (q_item.type == RTX_RESP_230_SCAN_BEST || q_item.type == RTX_RESP_240_CHAN_SW) {
      tx_payload[p_idx++] = (uint8_t)q_item.vtx_mhz;
    } else if (q_item.type == RTX_RESP_250_VTX_ACK) {
      tx_payload[p_idx++] = (uint8_t)(q_item.vtx_mhz >> 8); tx_payload[p_idx++] = (uint8_t)(q_item.vtx_mhz & 0xFF);
      tx_payload[p_idx++] = (uint8_t)(q_item.vtx_mw >> 8);  tx_payload[p_idx++] = (uint8_t)(q_item.vtx_mw & 0xFF);
    }
  } else {
    tx_payload[p_idx++] = 210;
    tx_payload[p_idx++] = current_link_quality;
    uint16_t v_centi = (uint16_t)constrain(lroundf(v_batt * 100.0f), 0L, 65535L);
    uint16_t i_centi = (uint16_t)constrain(lroundf(current_a * 100.0f), 0L, 65535L);
    uint16_t w_deci  = (uint16_t)constrain(lroundf(power_w * 10.0f), 0L, 65535L);
    uint16_t wh_milli = (uint16_t)constrain(lroundf(consumed_wh * 1000.0f), 0L, 65535L);
    int16_t alt_cm   = (int16_t)constrain(lroundf(baro_alt_m * 100.0f), -32768L, 32767L);
    uint16_t hdg_deci = (uint16_t)constrain(lroundf(compass_heading_deg * 10.0f), 0L, 3600L);
    uint16_t lid_mm   = (uint16_t)constrain(lroundf(dbg_lidar_m * 1000.0f), 0L, 65535L);
    int16_t temp_c10  = (int16_t)constrain(lroundf(current_chip_temp * 10.0f), -32768L, 32767L);
    tx_payload[p_idx++] = (uint8_t)(v_centi >> 8);  tx_payload[p_idx++] = (uint8_t)(v_centi & 0xFF);
    tx_payload[p_idx++] = (uint8_t)(i_centi >> 8);  tx_payload[p_idx++] = (uint8_t)(i_centi & 0xFF);
    tx_payload[p_idx++] = (uint8_t)(w_deci >> 8);   tx_payload[p_idx++] = (uint8_t)(w_deci & 0xFF);
    tx_payload[p_idx++] = (uint8_t)(wh_milli >> 8); tx_payload[p_idx++] = (uint8_t)(wh_milli & 0xFF);
    tx_payload[p_idx++] = (uint8_t)((uint16_t)alt_cm >> 8); tx_payload[p_idx++] = (uint8_t)((uint16_t)alt_cm & 0xFF);
    tx_payload[p_idx++] = (uint8_t)(hdg_deci >> 8); tx_payload[p_idx++] = (uint8_t)(hdg_deci & 0xFF);
    tx_payload[p_idx++] = (uint8_t)(lid_mm >> 8);   tx_payload[p_idx++] = (uint8_t)(lid_mm & 0xFF);
    tx_payload[p_idx++] = (uint8_t)((uint16_t)temp_c10 >> 8); tx_payload[p_idx++] = (uint8_t)((uint16_t)temp_c10 & 0xFF);
    tx_payload[p_idx++] = (uint8_t)(is_armed ? 1 : 0);
    tx_payload[p_idx++] = getActiveStatusErrorCode();
  }

  uint8_t buf_cfg[2] = {0x00, 0x00};
  rtx_write_command(0x8F, buf_cfg, 2);

  rtx_spi_begin_tx(); rtxSpi.transfer(0x0E); rtxSpi.transfer(0x00);
  for (uint8_t i = 0; i < p_idx; i++) rtxSpi.transfer(tx_payload[i]);
  rtx_spi_end_tx();

  uint8_t pkt_params[6] = {
    0x00, 0x08, 0x00, p_idx,
    (uint8_t)(currentConfig.rtx_hw_crc_en ? 0x01 : 0x00),
    (uint8_t)(currentConfig.rtx_iq_inv ? 0x01 : 0x00)
  };
  rtx_write_command(0x8C, pkt_params, 6);

  uint8_t tx_params[3] = {0x00, 0x00, 0x00};
  rtx_write_command(0x83, tx_params, 3);

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
  bool tx_done = false, tx_timed_out = false;

  if (digitalRead(RTX_BUSY_PIN) == LOW) {
    uint8_t irq_status[2] = {0, 0};
    rtx_read_command(0x12, irq_status, 2);
    uint16_t irq = ((uint16_t)irq_status[0] << 8) | irq_status[1];
    if (irq & 0x01) {
      uint8_t clear_irq[2] = {0x00, 0x01};
      rtx_write_command(0x02, clear_irq, 2);
      tx_done = true;
    }
  }
  if (!tx_done && ((now_ms - rtx_tx_start_ms) >= rtx_tx_timeout_ms)) {
    tx_timed_out = true; rtx_error_counter++; setFcErrorRam(FC_ERR_RTX_TX_TIMEOUT);
    if (!is_on_backup) {
      is_on_backup = true; active_rtx_chan = constrain(currentConfig.rtx_bak_chan, 0, 99);
      rtx_apply_switch_after_tx = true; rtx_post_tx_chan = active_rtx_chan; rtx_post_tx_backup = true;
    }
  }

  if (tx_done || tx_timed_out) {
    rtx_tx_in_flight = false;
    if (rtx_apply_switch_after_tx) {
      rtx_apply_switch_after_tx = false; is_on_backup = rtx_post_tx_backup; active_rtx_chan = rtx_post_tx_chan;
      applyRtxConfiguration(is_on_backup);
    } else rtx_set_rx_mode();
  }
}

// ==============================================================================
// 7. УПРАВЛЕНИЕ VTX (SMARTAUDIO v2.1 / TRAMP)
// ==============================================================================
static inline float vtxMwToDbm(uint16_t mw) {
  return (mw == 0) ? 0.0f : (10.0f * log10f((float)mw));
}

static inline uint16_t vtxDbmToMw(float dbm) {
  float mw = powf(10.0f, constrain(dbm, 0.0f, 40.0f) / 10.0f);
  return (uint16_t)lroundf(constrain(mw, 1.0f, 10000.0f));
}

static inline uint16_t vtxClampMw(uint16_t mw) {
  return (mw == 0) ? DEFAULT_VTX_MW : (uint16_t)constrain((int)mw, 1, 10000);
}

static inline uint8_t vtxEncodeSaDirectDbm(uint16_t mw) {
  int dbm = constrain((int)lroundf(vtxMwToDbm(vtxClampMw(mw))), 0, 40);
  return (uint8_t)(0x80 | (dbm & 0x7F));
}

static HardwareSerial* getVtxSerial(const char* &port_name_out) {
#if defined(HAVE_HWSERIAL3) || defined(USART3)
  port_name_out = "USART3(PB10/PB11)";
  return &Serial3;
#elif defined(HAVE_HWSERIAL2) || defined(USART2)
  port_name_out = "USART2(PA2/PA3)";
  return &Serial2;
#else
  port_name_out = "NONE";
  return nullptr;
#endif
}

static void vtxEnsureBaud(HardwareSerial* ser, uint32_t baud) {
  if (!ser) return;
  if (!vtx_uart_ready || vtx_baud != baud) {
    if (vtx_uart_ready) {
      ser->flush();
      ser->end();
    }
#if defined(USART3)
    if (ser == &Serial3) {
      Serial3.setTx(VTX_TX_PIN);
      Serial3.setRx(VTX_RX_PIN);
    }
#endif
    ser->begin(baud);
    vtx_baud = baud;
    vtx_uart_ready = true;
    tramp_rx_idx = 0;
    sa_rx_idx = 0;
    while (ser->available() > 0) (void)ser->read();
  }
}

static uint8_t smartAudioCrc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static uint8_t trampCrc8(const uint8_t *f) {
  uint16_t sum = 0;
  for (uint8_t i = 1; i <= 13; i++) sum += f[i];
  return (uint8_t)(sum & 0xFF);
}

static void vtxMarkFail() {
  vtx_link_ok = false;
  setFcErrorRam(FC_ERR_SA_VTX_FAIL);
}

static void vtxMarkOk(uint8_t proto, uint32_t now_ms) {
  if (proto == VTX_PROTO_SMARTAUDIO || proto == VTX_PROTO_TRAMP) {
    vtx_detected_proto = proto;
  }
  vtx_link_ok = true;
  vtx_last_probe_ms = now_ms;
  vtx_initial_probe_done = true;
}

static uint8_t vtxResolveProto() {
  if (currentConfig.vtx_prot == VTX_PROT_TRAMP) return VTX_PROTO_TRAMP;
  if (currentConfig.vtx_prot == VTX_PROT_SMARTAUDIO) return VTX_PROTO_SMARTAUDIO;
  return vtx_detected_proto;
}

static void vtxParseSmartAudioFrame(const uint8_t *buf, uint8_t total_len) {
  uint8_t payload_len = buf[3];
  if (payload_len >= 5 && (uint8_t)(4 + payload_len + 1) == total_len) {
    uint8_t raw_dbm = buf[8] & 0x7F;
    if (raw_dbm <= 40) {
      vtx_actual_dbm = (int8_t)raw_dbm;
      vtx_actual_mw = vtxDbmToMw((float)vtx_actual_dbm);
    }
  }
}

static void vtxParseTrampFrame(const uint8_t *f) {
  uint16_t val16 = (uint16_t)f[2] | ((uint16_t)f[3] << 8);
  if ((f[1] == 'r' || f[1] == 'v') && val16 > 0 && val16 <= 10000) {
    vtx_actual_mw = val16;
    vtx_actual_dbm = (int8_t)constrain(lroundf(vtxMwToDbm(val16)), 0L, 40L);
  }
}

static bool pollSmartAudioRx(HardwareSerial* ser) {
  bool seen = false;
  while (ser->available() > 0) {
    uint8_t b = (uint8_t)ser->read();
    if (sa_rx_idx == 0) {
      if (b == 0xAA) sa_rx_buf[sa_rx_idx++] = b;
    } else if (sa_rx_idx == 1) {
      if (b == 0x55) sa_rx_buf[sa_rx_idx++] = b;
      else sa_rx_idx = (b == 0xAA) ? 1 : 0;
    } else {
      sa_rx_buf[sa_rx_idx++] = b;
      if (sa_rx_idx >= 4) {
        uint8_t payload_len = sa_rx_buf[3];
        if (payload_len > 10) { sa_rx_idx = 0; continue; }
        uint8_t total_len = (uint8_t)(4 + payload_len + 1);
        if (sa_rx_idx == total_len) {
          if (smartAudioCrc8(sa_rx_buf, (uint8_t)(total_len - 1)) == sa_rx_buf[total_len - 1]) {
            vtxParseSmartAudioFrame(sa_rx_buf, total_len);
            seen = true;
          }
          sa_rx_idx = 0;
        }
      }
    }
  }
  return seen;
}

static bool pollTrampRx(HardwareSerial* ser) {
  bool seen = false;
  while (ser->available() > 0) {
    uint8_t b = (uint8_t)ser->read();
    if (tramp_rx_idx == 0) {
      if (b == '$') tramp_rx_buf[tramp_rx_idx++] = b;
    } else {
      tramp_rx_buf[tramp_rx_idx++] = b;
      if (tramp_rx_idx == 16) {
        if (tramp_rx_buf[15] == 0x00 && tramp_rx_buf[14] == trampCrc8(tramp_rx_buf)) {
          vtxParseTrampFrame(tramp_rx_buf);
          seen = true;
        }
        tramp_rx_idx = 0;
      }
    }
  }
  return seen;
}

static bool sendSmartAudioFrame(uint8_t cmd_id, const uint8_t *payload, uint8_t payload_len) {
  const char* port_name = nullptr;
  HardwareSerial* ser = getVtxSerial(port_name);
  if (!ser || vtx_hw_unavailable) {
    setFcErrorRam(FC_ERR_SA_VTX_FAIL);
    return false;
  }
  vtxEnsureBaud(ser, SMARTAUDIO_BAUD_RATE);
  sa_rx_idx = 0;
  while (ser->available() > 0) (void)ser->read();
  uint8_t frame[12];
  uint8_t idx = 0;
  frame[idx++] = 0xAA; frame[idx++] = 0x55; frame[idx++] = cmd_id; frame[idx++] = payload_len;
  for (uint8_t i = 0; i < payload_len && idx < sizeof(frame) - 1; i++) frame[idx++] = payload[i];
  frame[idx] = smartAudioCrc8(frame, idx);
  idx++;
  if (ser->write(frame, idx) != idx) {
    vtxMarkFail();
    return false;
  }
  return true;
}

static bool sendTrampFrame(char cmd, uint16_t param) {
  const char* port_name = nullptr;
  HardwareSerial* ser = getVtxSerial(port_name);
  if (!ser || vtx_hw_unavailable) {
    setFcErrorRam(FC_ERR_SA_VTX_FAIL);
    return false;
  }
  vtxEnsureBaud(ser, TRAMP_BAUD_RATE);
  tramp_rx_idx = 0;
  while (ser->available() > 0) (void)ser->read();
  uint8_t frame[16];
  memset(frame, 0, 16);
  frame[0] = '$'; frame[1] = (uint8_t)cmd;
  frame[2] = (uint8_t)(param & 0xFF); frame[3] = (uint8_t)((param >> 8) & 0xFF);
  frame[14] = trampCrc8(frame);
  if (ser->write(frame, 16) != 16) {
    vtxMarkFail();
    return false;
  }
  return true;
}

static bool vtxSendFreqCmd(uint8_t proto) {
  if (proto == VTX_PROTO_TRAMP) return sendTrampFrame('F', currentConfig.vtx_mhz);
  uint8_t fbuf[2] = {(uint8_t)(currentConfig.vtx_mhz >> 8), (uint8_t)(currentConfig.vtx_mhz & 0xFF)};
  return sendSmartAudioFrame(0x09, fbuf, 2);
}

static bool vtxSendPowerCmd(uint8_t proto) {
  if (proto == VTX_PROTO_TRAMP) return sendTrampFrame('P', currentConfig.vtx_mw);
  uint8_t pbyte = vtxEncodeSaDirectDbm(currentConfig.vtx_mw);
  return sendSmartAudioFrame(0x05, &pbyte, 1);
}

static void vtxArmCmdAck(uint32_t now_ms) {
  vtx_last_tx_ms = now_ms;
  vtx_step_start_ms = now_ms;
  vtx_probe_step = VTX_STEP_WAIT_CMD_ACK;
}

void vtxSetFrequencyMhz(uint16_t mhz, bool send_now, bool save_eeprom) {
  currentConfig.vtx_mhz = (uint16_t)constrain((int)mhz, 5000, 6000);
  vtx_send_freq_req = true;
  if (send_now && currentConfig.vtx_en) {
    uint8_t proto = vtxResolveProto();
    if ((proto == VTX_PROTO_TRAMP || proto == VTX_PROTO_SMARTAUDIO) && vtxSendFreqCmd(proto)) {
      vtx_send_freq_req = false;
      vtxArmCmdAck(millis());
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
      vtx_send_power_req = false;
      vtxArmCmdAck(millis());
    }
  }
  if (save_eeprom) saveSettings();
}

void vtxSetFrequencyAndPower(uint16_t mhz, uint16_t power_mw, bool save_eeprom) {
  currentConfig.vtx_mhz = (uint16_t)constrain((int)mhz, 5000, 6000);
  currentConfig.vtx_mw = vtxClampMw(power_mw);
  vtx_send_freq_req = true;
  vtx_send_power_req = true;
  if (save_eeprom) saveSettings();
}

void vtxInit() {
  vtx_hw_unavailable = false;
  vtx_initial_probe_done = false;
  vtx_probe_step = VTX_STEP_IDLE;
  vtx_step_start_ms = 0;
  vtx_baud = 0;
  vtx_link_ok = false;
  vtx_detected_proto = VTX_PROTO_NONE;
  vtx_actual_dbm = 0;
  vtx_actual_mw = 0;
  tramp_rx_idx = 0;
  sa_rx_idx = 0;

  if (!currentConfig.vtx_en) {
    vtx_uart_ready = false;
    vtx_send_freq_req = false;
    vtx_send_power_req = false;
    return;
  }

  const char* port_name = nullptr;
  HardwareSerial* ser = getVtxSerial(port_name);
  if (!ser) {
    vtx_uart_ready = false;
    vtx_hw_unavailable = true;
    vtxMarkFail();
    return;
  }

  uint8_t proto = vtxResolveProto();
  vtxEnsureBaud(ser, (proto == VTX_PROTO_TRAMP) ? TRAMP_BAUD_RATE : SMARTAUDIO_BAUD_RATE);
  vtx_send_freq_req = true;
  vtx_send_power_req = true;
  vtx_last_probe_ms = millis();
}

static void vtxFinishProbeTimeout(uint32_t now_ms) {
  vtx_initial_probe_done = true;
  vtx_last_probe_ms = now_ms;
  vtxMarkFail();
  vtx_probe_step = VTX_STEP_IDLE;
}

void vtxStepNonBlocking(uint32_t now_ms) {
  if (!currentConfig.vtx_en) {
    vtx_link_ok = false;
    vtx_probe_step = VTX_STEP_IDLE;
    return;
  }
  if (vtx_hw_unavailable) {
    setFcErrorRam(FC_ERR_SA_VTX_FAIL);
    return;
  }
  if (!vtx_uart_ready) {
    vtxInit();
    if (!vtx_uart_ready) return;
  }
  const char* port_name = nullptr;
  HardwareSerial* ser = getVtxSerial(port_name);
  if (!ser) {
    vtx_hw_unavailable = true;
    vtxMarkFail();
    return;
  }

  uint8_t known_proto = vtxResolveProto();
  bool allow_auto_fallback = (currentConfig.vtx_prot == VTX_PROT_AUTO) && (known_proto == VTX_PROTO_NONE);

  if (vtx_probe_step == VTX_STEP_WAIT_SA) {
    if (pollSmartAudioRx(ser)) {
      vtxMarkOk(VTX_PROTO_SMARTAUDIO, now_ms);
      vtx_probe_step = VTX_STEP_IDLE;
      return;
    }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) {
      if (allow_auto_fallback && sendTrampFrame('v', 0)) {
        vtx_last_tx_ms = now_ms;
        vtx_step_start_ms = now_ms;
        vtx_probe_step = VTX_STEP_WAIT_TRAMP;
      } else {
        vtxFinishProbeTimeout(now_ms);
      }
    }
    return;
  }

  if (vtx_probe_step == VTX_STEP_WAIT_TRAMP) {
    if (pollTrampRx(ser)) {
      vtxMarkOk(VTX_PROTO_TRAMP, now_ms);
      vtx_probe_step = VTX_STEP_IDLE;
      return;
    }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) vtxFinishProbeTimeout(now_ms);
    return;
  }

  if (vtx_probe_step == VTX_STEP_WAIT_CMD_ACK) {
    bool got_ack = (known_proto == VTX_PROTO_TRAMP) ? pollTrampRx(ser) : pollSmartAudioRx(ser);
    if (got_ack) {
      vtxMarkOk(known_proto ? known_proto : VTX_PROTO_SMARTAUDIO, now_ms);
      vtx_probe_step = VTX_STEP_IDLE;
      return;
    }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) vtxFinishProbeTimeout(now_ms);
    return;
  }

  if (known_proto == VTX_PROTO_TRAMP && pollTrampRx(ser)) vtxMarkOk(VTX_PROTO_TRAMP, now_ms);
  else if (known_proto == VTX_PROTO_SMARTAUDIO && pollSmartAudioRx(ser)) vtxMarkOk(VTX_PROTO_SMARTAUDIO, now_ms);

  if ((now_ms - vtx_last_tx_ms) < VTX_CMD_INTERVAL_MS) return;

  if (vtx_send_freq_req || vtx_send_power_req) {
    uint8_t use_proto = (known_proto != VTX_PROTO_NONE) ? known_proto : VTX_PROTO_SMARTAUDIO;
    if (vtx_send_freq_req && vtxSendFreqCmd(use_proto)) {
      vtx_send_freq_req = false;
      vtxArmCmdAck(now_ms);
      return;
    }
    if (vtx_send_power_req && vtxSendPowerCmd(use_proto)) {
      vtx_send_power_req = false;
      vtxArmCmdAck(now_ms);
      return;
    }
  }

  if (!vtx_initial_probe_done || (now_ms - vtx_last_probe_ms) >= VTX_PROBE_INTERVAL_MS) {
    if (known_proto == VTX_PROTO_TRAMP) {
      if (sendTrampFrame('v', 0)) {
        vtx_last_tx_ms = now_ms; vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_TRAMP;
      }
    } else {
      if (sendSmartAudioFrame(0x03, nullptr, 0)) {
        vtx_last_tx_ms = now_ms; vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_SA;
      }
    }
  }
}

// ==============================================================================
// 8. ДРАЙВЕРЫ ДАТЧИКОВ (IMU / BARO / MAG / LIDAR / BAT)
// ==============================================================================
namespace SensorDriver {
  enum IMUType : uint8_t {
    IMU_UNKNOWN = 0, IMU_MPU6050 = 1, IMU_MPU6500 = 2, IMU_MPU9250 = 3,
    IMU_ICM20602 = 4, IMU_BMI160 = 5, IMU_QMI8658 = 6
  };
  enum BaroType : uint8_t {
    BARO_UNKNOWN = 0, BARO_BMP280 = 1, BARO_BME280 = 2, BARO_MS5611 = 3,
    BARO_BMP180 = 4, BARO_BMP388 = 5, BARO_BMP390 = 6, BARO_DPS310 = 7
  };
  enum MagType : uint8_t {
    MAG_UNKNOWN = 0, MAG_HMC5883L = 1, MAG_QMC5883L = 2, MAG_AK8963 = 3,
    MAG_IST8310 = 4, MAG_HSCDTD008A = 5, MAG_AK09911 = 6
  };
  enum LidarType : uint8_t {
    LIDAR_NONE = 0, LIDAR_VL53L0X = 1, LIDAR_VL53L1X = 2, LIDAR_VL53L3CX = 3, LIDAR_VL53L4CX = 4
  };
  enum Ms5611Phase : uint8_t { MS_IDLE = 0, MS_WAIT_D1, MS_WAIT_D2 };
  enum Bmp180Phase : uint8_t { B180_IDLE = 0, B180_WAIT_TEMP, B180_WAIT_PRESS };

  struct BaroCalib {
    uint16_t dig_T1 = 0; int16_t dig_T2 = 0; int16_t dig_T3 = 0;
    uint16_t dig_P1 = 0; int16_t dig_P2 = 0; int16_t dig_P3 = 0; int16_t dig_P4 = 0;
    int16_t dig_P5 = 0; int16_t dig_P6 = 0; int16_t dig_P7 = 0; int16_t dig_P8 = 0; int16_t dig_P9 = 0;
    uint16_t ms_c[7] = {0};
    int16_t ac1 = 0, ac2 = 0, ac3 = 0;
    uint16_t ac4 = 0, ac5 = 0, ac6 = 0;
    int16_t b1 = 0, b2 = 0, mc = 0, md = 0;
    float b3_t1 = 0.0f, b3_t2 = 0.0f, b3_t3 = 0.0f;
    float b3_p1 = 0.0f, b3_p2 = 0.0f, b3_p3 = 0.0f, b3_p4 = 0.0f, b3_p5 = 0.0f;
    float b3_p6 = 0.0f, b3_p7 = 0.0f, b3_p8 = 0.0f, b3_p9 = 0.0f, b3_p10 = 0.0f, b3_p11 = 0.0f;
    int32_t dps_c00 = 0, dps_c10 = 0;
    int16_t dps_c01 = 0, dps_c11 = 0, dps_c20 = 0, dps_c21 = 0, dps_c30 = 0;
    float dps_kp = 524288.0f, dps_kt = 524288.0f;
    bool valid = false;
  } baroCal;

  struct MagCalib {
    float asa_x = 1.0f, asa_y = 1.0f, asa_z = 1.0f;
    float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;
  } magCal;

  struct SensorStatus {
    IMUType imu_type = IMU_UNKNOWN; BaroType baro_type = BARO_UNKNOWN;
    MagType mag_type = MAG_UNKNOWN; LidarType lidar_type = LIDAR_NONE;
    uint8_t imu_addr = 0x68; uint8_t baro_addr = 0x76; uint8_t mag_addr = 0x0C; uint8_t lidar_addr = 0x29;
    float gyro_scale = 131.0f; float acc_scale = 16384.0f; float temp_scale = 340.0f; float temp_offset = 25.0f;
    float lidar_max_m = 4.0f;
    bool imu_found = false; bool baro_found = false; bool mag_found = false; bool lidar_found = false;
  } activeSensors;

  Ms5611Phase ms5611_phase = MS_IDLE; uint32_t ms5611_conv_start_ms = 0; uint32_t ms5611_d1_raw = 0; uint32_t ms5611_d2_raw = 0;
  Bmp180Phase bmp180_phase = B180_IDLE; uint32_t bmp180_conv_start_ms = 0; int32_t bmp180_ut_raw = 0;

  void detectIMU();
  void detectBaro();
  void detectMag();
  void detectLidar();
  void initLidarHardware();
  bool calibrateGyro(bool silent = false);
  bool calibrateAcc(bool silent = false);
  void calibrateIMU();
  void calibrateBaro();
  void calibrateLidar();
  void calibrateBatVoltage(float actual_voltage);
  void calibrateCurrent(bool has_value, float actual_current_a);

  float getEffectiveLidarMaxM() { return activeSensors.lidar_max_m * 0.90f; }
  inline float getBatScaleV() { return (currentConfig.bat_scale_v > 0.1f) ? currentConfig.bat_scale_v : DEFAULT_BAT_SCALE_V; }
  inline float getBatScaleA() { return (currentConfig.bat_scale_a > 0.01f) ? currentConfig.bat_scale_a : DEFAULT_BAT_SCALE_A; }

  inline float computeCurrentAmpsFromAdc(int raw_adc) {
    float pin_v = ((float)raw_adc / 4095.0f) * 3.3f;
    float scaled_a = ((pin_v / SHUNT_RESISTANCE_OHM) - currentConfig.bat_offset_a) * getBatScaleA();
    return (scaled_a > 0.0f) ? scaled_a : 0.0f;
  }

  inline float wrapDeg0To360(float deg) {
    if (isnan(deg) || isinf(deg)) { setFcErrorRam(FC_ERR_NAN_INF_MATH); return 0.0f; }
    float wrapped = fmodf(deg, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return (wrapped >= 360.0f || wrapped < 0.0f) ? 0.0f : wrapped;
  }

  inline void rotateXYByYawAlign(float &x, float &y, uint8_t align) {
    float ox = x, oy = y;
    switch (align & 0x03) {
      case 1: x = oy; y = -ox; break;
      case 2: x = -ox; y = -oy; break;
      case 3: x = -oy; y = ox; break;
      case 0: default: break;
    }
  }

  inline float extractTempFromImuBuf(const uint8_t *buf) {
    return ((float)((int16_t)(buf[6] << 8 | buf[7])) / activeSensors.temp_scale) + activeSensors.temp_offset;
  }

  inline void packNormalizedImuToBuffer(uint8_t *out14, float ax_g, float ay_g, float az_g, float temp_c, float gx_dps, float gy_dps, float gz_dps) {
    int16_t iax = (int16_t)constrain(ax_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t iay = (int16_t)constrain(ay_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t iaz = (int16_t)constrain(az_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t itm = (int16_t)constrain((temp_c - activeSensors.temp_offset) * activeSensors.temp_scale, -32767.0f, 32767.0f);
    int16_t igx = (int16_t)constrain(gx_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    int16_t igy = (int16_t)constrain(gy_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    int16_t igz = (int16_t)constrain(gz_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
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
      Wire.end(); Wire.setSCL(PB6); Wire.setSDA(PB7); Wire.begin(); Wire.setClock(400000);
      ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE; i2c_error_counter = 0;
    }
  }

  static bool recordI2cError() { i2c_error_counter++; setFcErrorRam(FC_ERR_I2C_BUS); return false; }

  bool readReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    checkBusAndRecover();
    Wire.beginTransmission(addr); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return recordI2cError();
    if (Wire.requestFrom(addr, len, (uint8_t)true) == len) {
      for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
      i2c_error_counter = 0; return true;
    }
    return recordI2cError();
  }

  bool readReg16Addr(uint8_t addr, uint16_t reg16, uint8_t *buf, uint8_t len) {
    checkBusAndRecover();
    Wire.beginTransmission(addr); Wire.write((uint8_t)(reg16 >> 8)); Wire.write((uint8_t)(reg16 & 0xFF));
    if (Wire.endTransmission(false) != 0) return recordI2cError();
    if (Wire.requestFrom(addr, len, (uint8_t)true) == len) {
      for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
      i2c_error_counter = 0; return true;
    }
    return recordI2cError();
  }

  bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    checkBusAndRecover();
    Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val);
    if (Wire.endTransmission() != 0) return recordI2cError();
    i2c_error_counter = 0; return true;
  }

  bool writeReg16Addr8Val(uint8_t addr, uint16_t reg16, uint8_t val) {
    checkBusAndRecover();
    Wire.beginTransmission(addr); Wire.write((uint8_t)(reg16 >> 8)); Wire.write((uint8_t)(reg16 & 0xFF)); Wire.write(val);
    if (Wire.endTransmission() != 0) return recordI2cError();
    i2c_error_counter = 0; return true;
  }

  void enableImuI2cBypass() {
    if (!activeSensors.imu_found) return;
    if (activeSensors.imu_type == IMU_MPU9250 || activeSensors.imu_type == IMU_MPU6500) {
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

  void detectIMU() {
    uint8_t id = 0, req_id = currentConfig.imu_id;
    activeSensors.imu_found = false; activeSensors.imu_type = IMU_UNKNOWN; activeSensors.temp_offset = 25.0f;
    if (req_id == IMU_BMI160) {
      for (uint8_t addr : (const uint8_t[]){0x68, 0x69}) if (readReg(addr, 0x00, &id, 1) && id == 0xD1) { initSpecificImu(addr, IMU_BMI160); return; }
    } else if (req_id == IMU_QMI8658) {
      for (uint8_t addr : (const uint8_t[]){0x6A, 0x6B, 0x68, 0x69}) if (readReg(addr, 0x00, &id, 1) && id == 0x05) { initSpecificImu(addr, IMU_QMI8658); return; }
    } else if (req_id == IMU_MPU6050 || req_id == IMU_MPU6500 || req_id == IMU_MPU9250 || req_id == IMU_ICM20602) {
      for (uint8_t addr : (const uint8_t[]){0x68, 0x69}) {
        if (!readReg(addr, 0x75, &id, 1)) continue;
        if (req_id == IMU_MPU6050 && id == 0x68) { initSpecificImu(addr, IMU_MPU6050); return; }
        if (req_id == IMU_MPU6500 && id == 0x70) { initSpecificImu(addr, IMU_MPU6500); return; }
        if (req_id == IMU_MPU9250 && (id == 0x71 || id == 0x73 || id == 0x70)) { initSpecificImu(addr, IMU_MPU9250); return; }
        if (req_id == IMU_ICM20602 && id == 0x12) { initSpecificImu(addr, IMU_ICM20602); return; }
      }
    } else {
      for (uint8_t addr : (const uint8_t[]){0x68, 0x69}) if (readReg(addr, 0x00, &id, 1) && id == 0xD1) { initSpecificImu(addr, IMU_BMI160); return; }
      for (uint8_t addr : (const uint8_t[]){0x6A, 0x6B, 0x68, 0x69}) if (readReg(addr, 0x00, &id, 1) && id == 0x05) { initSpecificImu(addr, IMU_QMI8658); return; }
      for (uint8_t addr : (const uint8_t[]){0x68, 0x69}) {
        if (!readReg(addr, 0x75, &id, 1)) continue;
        if (id == 0x68) { initSpecificImu(addr, IMU_MPU6050); return; }
        if (id == 0x70) {
          uint8_t ak_id = 0; writeReg(addr, 0x6A, 0x00); delay(2); writeReg(addr, 0x37, 0x02); delay(5);
          initSpecificImu(addr, (readReg(0x0C, 0x00, &ak_id, 1) && ak_id == 0x48) ? IMU_MPU9250 : IMU_MPU6500);
          return;
        }
        if (id == 0x71 || id == 0x73) { initSpecificImu(addr, IMU_MPU9250); return; }
        if (id == 0x12) { initSpecificImu(addr, IMU_ICM20602); return; }
      }
    }
    err_imu_fail = true; setFcErrorRam(FC_ERR_IMU_FAIL);
  }

  bool readMPU6050(uint8_t *buffer) {
    if (!activeSensors.imu_found) return false;
    if (activeSensors.imu_type == IMU_BMI160) {
      uint8_t raw[14];
      if (!readReg(activeSensors.imu_addr, 0x0C, raw, 14)) return false;
      packNormalizedImuToBuffer(buffer,
                                (float)((int16_t)((uint16_t)raw[7] << 8 | raw[6])) / activeSensors.acc_scale,
                                (float)((int16_t)((uint16_t)raw[9] << 8 | raw[8])) / activeSensors.acc_scale,
                                (float)((int16_t)((uint16_t)raw[11] << 8 | raw[10])) / activeSensors.acc_scale,
                                ((float)((int16_t)((uint16_t)raw[13] << 8 | raw[12])) / activeSensors.temp_scale) + activeSensors.temp_offset,
                                (float)((int16_t)((uint16_t)raw[1] << 8 | raw[0])) / activeSensors.gyro_scale,
                                (float)((int16_t)((uint16_t)raw[3] << 8 | raw[2])) / activeSensors.gyro_scale,
                                (float)((int16_t)((uint16_t)raw[5] << 8 | raw[4])) / activeSensors.gyro_scale);
      return true;
    }
    if (activeSensors.imu_type == IMU_QMI8658) {
      uint8_t t_buf[2], xyz_buf[12];
      if (!readReg(activeSensors.imu_addr, 0x33, t_buf, 2) || !readReg(activeSensors.imu_addr, 0x35, xyz_buf, 12)) return false;
      packNormalizedImuToBuffer(buffer,
                                (float)((int16_t)((uint16_t)xyz_buf[1] << 8 | xyz_buf[0])) / activeSensors.acc_scale,
                                (float)((int16_t)((uint16_t)xyz_buf[3] << 8 | xyz_buf[2])) / activeSensors.acc_scale,
                                (float)((int16_t)((uint16_t)xyz_buf[5] << 8 | xyz_buf[4])) / activeSensors.acc_scale,
                                ((float)((int16_t)((uint16_t)t_buf[1] << 8 | t_buf[0])) / activeSensors.temp_scale) + activeSensors.temp_offset,
                                (float)((int16_t)((uint16_t)xyz_buf[7] << 8 | xyz_buf[6])) / activeSensors.gyro_scale,
                                (float)((int16_t)((uint16_t)xyz_buf[9] << 8 | xyz_buf[8])) / activeSensors.gyro_scale,
                                (float)((int16_t)((uint16_t)xyz_buf[11] << 8 | xyz_buf[10])) / activeSensors.gyro_scale);
      return true;
    }
    return readReg(activeSensors.imu_addr, 0x3B, buffer, 14);
  }

  inline float pressureToAltitudeMeters(float pressure_pa) { return 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f)); }
  inline bool isPressureValidPa(float pressure_pa) { return (pressure_pa >= 30000.0f && pressure_pa <= 125000.0f && !isnan(pressure_pa) && !isinf(pressure_pa)); }
  static void markBaroError() { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_BARO_FAIL); }

  static float dps310ScaleFromReg(uint8_t prc_code) {
    switch (prc_code & 0x07) {
      case 0: return 524288.0f; case 1: return 1572864.0f; case 2: return 3670016.0f; case 3: return 7864320.0f;
      case 4: return 253952.0f; case 5: return 516096.0f; case 6: return 1040384.0f; case 7: return 2088960.0f;
      default: return 524288.0f;
    }
  }

  bool readBme280PressurePa(float &pressure_pa) {
    uint8_t raw[6];
    if (!readReg(activeSensors.baro_addr, 0xF7, raw, 6)) return false;
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);
    int32_t var1_t = ((((adc_T >> 3) - ((int32_t)baroCal.dig_T1 << 1))) * ((int32_t)baroCal.dig_T2)) >> 11;
    int32_t var2_t = (((((adc_T >> 4) - ((int32_t)baroCal.dig_T1)) * ((adc_T >> 4) - ((int32_t)baroCal.dig_T1))) >> 12) * ((int32_t)baroCal.dig_T3)) >> 14;
    int32_t t_fine = var1_t + var2_t;
    int64_t var1_p = ((int64_t)t_fine) - 128000;
    int64_t var2_p = var1_p * var1_p * (int64_t)baroCal.dig_P6 + ((var1_p * (int64_t)baroCal.dig_P5) << 17) + (((int64_t)baroCal.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)baroCal.dig_P3) >> 8) + ((var1_p * (int64_t)baroCal.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)baroCal.dig_P1) >> 33;
    if (var1_p == 0) return false;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = (((int64_t)baroCal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2_p = (((int64_t)baroCal.dig_P8) * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)baroCal.dig_P7) << 4);
    pressure_pa = (float)p / 256.0f;
    return isPressureValidPa(pressure_pa);
  }

  bool computeMs5611AltitudeFromRaw(uint32_t d1, uint32_t d2, float &raw_alt_m) {
    if (d1 == 0 || d2 == 0) return false;
    int32_t dT = (int32_t)d2 - ((int32_t)baroCal.ms_c[5] << 8);
    int64_t temp = 2000LL + (((int64_t)dT * baroCal.ms_c[6]) >> 23);
    int64_t off = ((int64_t)baroCal.ms_c[2] << 16) + (((int64_t)baroCal.ms_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)baroCal.ms_c[1] << 15) + (((int64_t)baroCal.ms_c[3] * dT) >> 8);
    if (temp < 2000) {
      int64_t t2 = ((int64_t)dT * dT) >> 31; int64_t dt_sub = temp - 2000LL;
      int64_t off2 = (5LL * dt_sub * dt_sub) >> 1; int64_t sens2 = (5LL * dt_sub * dt_sub) >> 2;
      if (temp < -1500) { int64_t dt15 = temp + 1500LL; off2 += 7LL * dt15 * dt15; sens2 += (11LL * dt15 * dt15) >> 1; }
      temp -= t2; off -= off2; sens -= sens2;
    }
    float pressure_pa = (float)(((((int64_t)d1 * sens) >> 21) - off) >> 15) * 10.0f;
    if (!isPressureValidPa(pressure_pa)) { markBaroError(); return false; }
    raw_alt_m = pressureToAltitudeMeters(pressure_pa); return true;
  }

  float computeBmp180PressurePa(int32_t ut, int32_t up, uint8_t oss = 1) {
    int32_t x1 = ((ut - (int32_t)baroCal.ac6) * (int32_t)baroCal.ac5) >> 15;
    int32_t denom = x1 + (int32_t)baroCal.md; if (denom == 0) denom = 1;
    int32_t x2 = ((int32_t)baroCal.mc << 11) / denom;
    int32_t b5 = x1 + x2; int32_t b6 = b5 - 4000;
    x1 = ((int32_t)baroCal.b2 * ((b6 * b6) >> 12)) >> 11; x2 = ((int32_t)baroCal.ac2 * b6) >> 11;
    int32_t b3 = (((((int32_t)baroCal.ac1) * 4 + (x1 + x2)) << oss) + 2) >> 2;
    x1 = ((int32_t)baroCal.ac3 * b6) >> 13; x2 = ((int32_t)baroCal.b1 * ((b6 * b6) >> 12)) >> 16;
    uint32_t b4 = ((uint32_t)baroCal.ac4 * (uint32_t)(((x1 + x2) + 2) >> 2 + 32768)) >> 15; if (b4 == 0) b4 = 1;
    uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * (50000UL >> oss);
    int32_t p = (b7 < 0x80000000UL) ? (int32_t)((b7 * 2UL) / b4) : (int32_t)((b7 / b4) * 2UL);
    x1 = (p >> 8) * (p >> 8); x1 = (x1 * 3038) >> 16; x2 = (-7357 * p) >> 16;
    return (float)(p + ((x1 + x2 + 3791) >> 4));
  }

  bool stepBmp180Phased(uint32_t now_ms, float &raw_alt_m) {
    constexpr uint8_t oss = 1;
    if (bmp180_phase == B180_IDLE) {
      if (!writeReg(activeSensors.baro_addr, 0xF4, 0x2E)) { markBaroError(); return false; }
      bmp180_conv_start_ms = now_ms; bmp180_phase = B180_WAIT_TEMP; return false;
    }
    if (bmp180_phase == B180_WAIT_TEMP) {
      if ((now_ms - bmp180_conv_start_ms) < BMP180_TEMP_DELAY_MS) return false;
      uint8_t t_buf[2]; if (!readReg(activeSensors.baro_addr, 0xF6, t_buf, 2)) { bmp180_phase = B180_IDLE; return false; }
      bmp180_ut_raw = (int32_t)((uint16_t)t_buf[0] << 8 | t_buf[1]);
      if (!writeReg(activeSensors.baro_addr, 0xF4, (uint8_t)(0x34 + (oss << 6)))) { bmp180_phase = B180_IDLE; return false; }
      bmp180_conv_start_ms = now_ms; bmp180_phase = B180_WAIT_PRESS; return false;
    }
    if ((now_ms - bmp180_conv_start_ms) < BMP180_PRESS_DELAY_MS) return false;
    uint8_t p_buf[3]; if (!readReg(activeSensors.baro_addr, 0xF6, p_buf, 3)) { bmp180_phase = B180_IDLE; return false; }
    int32_t up = (int32_t)(((((uint32_t)p_buf[0] << 16) | ((uint32_t)p_buf[1] << 8) | p_buf[2])) >> (8 - oss));
    bmp180_phase = B180_IDLE;
    float pressure_pa = computeBmp180PressurePa(bmp180_ut_raw, up, oss);
    if (!isPressureValidPa(pressure_pa)) { markBaroError(); return false; }
    raw_alt_m = pressureToAltitudeMeters(pressure_pa); return true;
  }

  bool readBmp388PressurePa(float &pressure_pa) {
    uint8_t d[6];
    if (!readReg(activeSensors.baro_addr, 0x04, d, 6)) return false;
    uint32_t uncomp_p = ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
    uint32_t uncomp_t = ((uint32_t)d[5] << 16) | ((uint32_t)d[4] << 8) | d[3];
    float pd1 = (float)uncomp_t - baroCal.b3_t1;
    float t_lin = (pd1 * baroCal.b3_t2) + (pd1 * pd1) * baroCal.b3_t3;
    float out1 = baroCal.b3_p5 + baroCal.b3_p6 * t_lin + baroCal.b3_p7 * (t_lin * t_lin) + baroCal.b3_p8 * (t_lin * t_lin * t_lin);
    float out2 = (float)uncomp_p * (baroCal.b3_p1 + baroCal.b3_p2 * t_lin + baroCal.b3_p3 * (t_lin * t_lin) + baroCal.b3_p4 * (t_lin * t_lin * t_lin));
    float p_sq = (float)uncomp_p * (float)uncomp_p;
    pressure_pa = out1 + out2 + p_sq * (baroCal.b3_p9 + baroCal.b3_p10 * t_lin) + (p_sq * (float)uncomp_p) * baroCal.b3_p11;
    return isPressureValidPa(pressure_pa);
  }

  bool readDps310PressurePa(float &pressure_pa) {
    uint8_t d[6];
    if (!readReg(activeSensors.baro_addr, 0x00, d, 6)) return false;
    int32_t praw = (int32_t)(((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2]); if (praw & 0x00800000) praw |= 0xFF000000;
    int32_t traw = (int32_t)(((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5]); if (traw & 0x00800000) traw |= 0xFF000000;
    float traw_sc = (float)traw / baroCal.dps_kt, praw_sc = (float)praw / baroCal.dps_kp;
    pressure_pa = (float)baroCal.dps_c00 + praw_sc * ((float)baroCal.dps_c10 + praw_sc * ((float)baroCal.dps_c20 + praw_sc * (float)baroCal.dps_c30)) +
                  traw_sc * ((float)baroCal.dps_c01 + praw_sc * ((float)baroCal.dps_c11 + praw_sc * (float)baroCal.dps_c21));
    return isPressureValidPa(pressure_pa);
  }

  bool stepMs5611Phased(uint32_t now_ms, float &raw_alt_m) {
    if (ms5611_phase == MS_IDLE) {
      if (!writeReg(activeSensors.baro_addr, 0x48, 0x00)) { markBaroError(); return false; }
      ms5611_conv_start_ms = now_ms; ms5611_phase = MS_WAIT_D1; return false;
    }
    if (ms5611_phase == MS_WAIT_D1) {
      if ((now_ms - ms5611_conv_start_ms) < MS5611_CONV_DELAY_MS) return false;
      uint8_t adc_buf[3]; if (!readReg(activeSensors.baro_addr, 0x00, adc_buf, 3)) { ms5611_phase = MS_IDLE; return false; }
      ms5611_d1_raw = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];
      if (!writeReg(activeSensors.baro_addr, 0x58, 0x00)) { ms5611_phase = MS_IDLE; markBaroError(); return false; }
      ms5611_conv_start_ms = now_ms; ms5611_phase = MS_WAIT_D2; return false;
    }
    if ((now_ms - ms5611_conv_start_ms) < MS5611_CONV_DELAY_MS) return false;
    uint8_t adc_buf[3]; if (!readReg(activeSensors.baro_addr, 0x00, adc_buf, 3)) { ms5611_phase = MS_IDLE; return false; }
    ms5611_d2_raw = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];
    ms5611_phase = MS_IDLE;
    return computeMs5611AltitudeFromRaw(ms5611_d1_raw, ms5611_d2_raw, raw_alt_m);
  }

  bool readRawBaroAltitudeBlocking(float &raw_alt_m) {
    if (!activeSensors.baro_found || !baroCal.valid) return false;
    if (activeSensors.baro_type == BARO_MS5611) {
      uint8_t adc_buf[3];
      if (!writeReg(activeSensors.baro_addr, 0x48, 0x00)) return false; delay(MS5611_CONV_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0x00, adc_buf, 3)) return false;
      uint32_t d1 = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];
      if (!writeReg(activeSensors.baro_addr, 0x58, 0x00)) return false; delay(MS5611_CONV_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0x00, adc_buf, 3)) return false;
      return computeMs5611AltitudeFromRaw(d1, ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2], raw_alt_m);
    }
    if (activeSensors.baro_type == BARO_BMP180) {
      constexpr uint8_t oss = 1; uint8_t t_buf[2], p_buf[3];
      if (!writeReg(activeSensors.baro_addr, 0xF4, 0x2E)) return false; delay(BMP180_TEMP_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0xF6, t_buf, 2)) return false;
      int32_t ut = (int32_t)((uint16_t)t_buf[0] << 8 | t_buf[1]);
      if (!writeReg(activeSensors.baro_addr, 0xF4, (uint8_t)(0x34 + (oss << 6)))) return false; delay(BMP180_PRESS_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0xF6, p_buf, 3)) return false;
      float p_pa = computeBmp180PressurePa(ut, (int32_t)(((((uint32_t)p_buf[0] << 16) | ((uint32_t)p_buf[1] << 8) | p_buf[2])) >> (8 - oss)), oss);
      if (!isPressureValidPa(p_pa)) return false;
      raw_alt_m = pressureToAltitudeMeters(p_pa); return true;
    }
    float p_pa = 0.0f;
    bool ok = (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) ? readBmp388PressurePa(p_pa) :
              (activeSensors.baro_type == BARO_DPS310) ? readDps310PressurePa(p_pa) : readBme280PressurePa(p_pa);
    if (!ok) return false;
    raw_alt_m = pressureToAltitudeMeters(p_pa); return true;
  }

  void initBaroHardware() {
    if (!activeSensors.baro_found) return;
    baroCal.valid = false; ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE;
    if (activeSensors.baro_type == BARO_BMP280 || activeSensors.baro_type == BARO_BME280) {
      uint8_t cal[24];
      if (readReg(activeSensors.baro_addr, 0x88, cal, 24)) {
        baroCal.dig_T1 = (uint16_t)(cal[1] << 8 | cal[0]); baroCal.dig_T2 = (int16_t)(cal[3] << 8 | cal[2]); baroCal.dig_T3 = (int16_t)(cal[5] << 8 | cal[4]);
        baroCal.dig_P1 = (uint16_t)(cal[7] << 8 | cal[6]); baroCal.dig_P2 = (int16_t)(cal[9] << 8 | cal[8]); baroCal.dig_P3 = (int16_t)(cal[11] << 8 | cal[10]);
        baroCal.dig_P4 = (int16_t)(cal[13] << 8 | cal[12]); baroCal.dig_P5 = (int16_t)(cal[15] << 8 | cal[14]); baroCal.dig_P6 = (int16_t)(cal[17] << 8 | cal[16]);
        baroCal.dig_P7 = (int16_t)(cal[19] << 8 | cal[18]); baroCal.dig_P8 = (int16_t)(cal[21] << 8 | cal[20]); baroCal.dig_P9 = (int16_t)(cal[23] << 8 | cal[22]);
        baroCal.valid = (baroCal.dig_T1 != 0 && baroCal.dig_P1 != 0);
      }
      writeReg(activeSensors.baro_addr, 0xF4, 0x2F);
    } else if (activeSensors.baro_type == BARO_BMP180) {
      uint8_t cal[22];
      if (readReg(activeSensors.baro_addr, 0xAA, cal, 22)) {
        baroCal.ac1 = (int16_t)(cal[0] << 8 | cal[1]); baroCal.ac2 = (int16_t)(cal[2] << 8 | cal[3]); baroCal.ac3 = (int16_t)(cal[4] << 8 | cal[5]);
        baroCal.ac4 = (uint16_t)(cal[6] << 8 | cal[7]); baroCal.ac5 = (uint16_t)(cal[8] << 8 | cal[9]); baroCal.ac6 = (uint16_t)(cal[10] << 8 | cal[11]);
        baroCal.b1 = (int16_t)(cal[12] << 8 | cal[13]); baroCal.b2 = (int16_t)(cal[14] << 8 | cal[15]);
        baroCal.mc = (int16_t)(cal[18] << 8 | cal[19]); baroCal.md = (int16_t)(cal[20] << 8 | cal[21]);
        baroCal.valid = (baroCal.ac4 != 0 && baroCal.ac5 != 0 && baroCal.ac6 != 0);
      }
    } else if (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) {
      uint8_t nvm[21];
      if (readReg(activeSensors.baro_addr, 0x31, nvm, 21)) {
        uint16_t u_t1 = (uint16_t)(nvm[1] << 8 | nvm[0]), u_t2 = (uint16_t)(nvm[3] << 8 | nvm[2]);
        int8_t s_t3 = (int8_t)nvm[4]; int16_t s_p1 = (int16_t)(nvm[6] << 8 | nvm[5]), s_p2 = (int16_t)(nvm[8] << 8 | nvm[7]);
        int8_t s_p3 = (int8_t)nvm[9], s_p4 = (int8_t)nvm[10]; uint16_t u_p5 = (uint16_t)(nvm[12] << 8 | nvm[11]), u_p6 = (uint16_t)(nvm[14] << 8 | nvm[13]);
        int8_t s_p7 = (int8_t)nvm[15], s_p8 = (int8_t)nvm[16]; int16_t s_p9 = (int16_t)(nvm[18] << 8 | nvm[17]);
        int8_t s_p10 = (int8_t)nvm[19], s_p11 = (int8_t)nvm[20];
        baroCal.b3_t1 = (float)u_t1 / powf(2.0f, -8.0f); baroCal.b3_t2 = (float)u_t2 / powf(2.0f, 30.0f); baroCal.b3_t3 = (float)s_t3 / powf(2.0f, 48.0f);
        baroCal.b3_p1 = ((float)s_p1 - 16384.0f) / powf(2.0f, 20.0f); baroCal.b3_p2 = ((float)s_p2 - 16384.0f) / powf(2.0f, 29.0f);
        baroCal.b3_p3 = (float)s_p3 / powf(2.0f, 32.0f); baroCal.b3_p4 = (float)s_p4 / powf(2.0f, 37.0f);
        baroCal.b3_p5 = (float)u_p5 / powf(2.0f, -3.0f); baroCal.b3_p6 = (float)u_p6 / powf(2.0f, 6.0f);
        baroCal.b3_p7 = (float)s_p7 / powf(2.0f, 8.0f); baroCal.b3_p8 = (float)s_p8 / powf(2.0f, 15.0f);
        baroCal.b3_p9 = (float)s_p9 / powf(2.0f, 48.0f); baroCal.b3_p10 = (float)s_p10 / powf(2.0f, 48.0f); baroCal.b3_p11 = (float)s_p11 / powf(2.0f, 65.0f);
        baroCal.valid = (u_t1 != 0 && u_t2 != 0);
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
      constexpr uint8_t pm_prc = 4, tm_prc = 4;
      writeReg(activeSensors.baro_addr, 0x06, (uint8_t)(0x50 | pm_prc)); writeReg(activeSensors.baro_addr, 0x07, (uint8_t)(0x80 | tm_prc)); writeReg(activeSensors.baro_addr, 0x09, 0x0C);
      baroCal.dps_kp = dps310ScaleFromReg(pm_prc); baroCal.dps_kt = dps310ScaleFromReg(tm_prc);
      writeReg(activeSensors.baro_addr, 0x08, 0x07);
    } else if (activeSensors.baro_type == BARO_MS5611) {
      uint8_t prom[2]; bool ok = true;
      for (uint8_t i = 1; i <= 6; i++) {
        if (readReg(activeSensors.baro_addr, 0xA0 + (i * 2), prom, 2)) { baroCal.ms_c[i] = ((uint16_t)prom[0] << 8) | prom[1]; if (baroCal.ms_c[i] == 0) ok = false; }
        else ok = false;
      }
      baroCal.valid = ok;
    }
    if (!baroCal.valid && currentConfig.baro_en) markBaroError();
  }

  static void bindAndInitBaro(uint8_t addr, BaroType t) { activeSensors.baro_addr = addr; activeSensors.baro_type = t; activeSensors.baro_found = true; initBaroHardware(); }

  void detectBaro() {
    uint8_t id = 0, req_id = currentConfig.bar_id;
    activeSensors.baro_found = false; activeSensors.baro_type = BARO_UNKNOWN; baroCal.valid = false;
    for (uint8_t addr : (const uint8_t[]){0x76, 0x77}) {
      if ((req_id == BARO_DPS310 || req_id == BARO_UNKNOWN) && readReg(addr, 0x0D, &id, 1) && id == 0x10) { bindAndInitBaro(addr, BARO_DPS310); return; }
      if ((req_id == BARO_BMP180 || req_id == BARO_BMP388 || req_id == BARO_BMP280 || req_id == BARO_BME280 || req_id == BARO_BMP390 || req_id == BARO_UNKNOWN) && readReg(addr, 0xD0, &id, 1)) {
        if ((req_id == BARO_BMP180 || req_id == BARO_UNKNOWN) && id == 0x55) { bindAndInitBaro(addr, BARO_BMP180); return; }
        if ((req_id == BARO_BMP388 || req_id == BARO_UNKNOWN) && id == 0x50) { bindAndInitBaro(addr, BARO_BMP388); return; }
        if ((req_id == BARO_BMP280 || req_id == BARO_UNKNOWN) && id == 0x58) { bindAndInitBaro(addr, BARO_BMP280); return; }
        if (id == 0x60) {
          uint8_t ctrl_meas = 0; bool has_ctrl = readReg(addr, 0xF4, &ctrl_meas, 1);
          BaroType detected = (has_ctrl && ctrl_meas != 0x00) ? BARO_BME280 : BARO_BMP390;
          if (req_id == BARO_UNKNOWN || req_id == (uint8_t)detected) { bindAndInitBaro(addr, detected); return; }
        }
      }
      if (req_id == BARO_MS5611 || req_id == BARO_UNKNOWN) {
        uint8_t prom[2]; if (readReg(addr, 0xA2, prom, 2) && (prom[0] != 0 || prom[1] != 0)) { bindAndInitBaro(addr, BARO_MS5611); return; }
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
    if (!activeSensors.baro_found || !baroCal.valid) { Serial.println("ERR: baro"); return; }
    float sum_alt = 0.0f; uint8_t valid_samples = 0;
    for (uint8_t i = 0; i < 12; i++) { float raw_alt = 0.0f; if (readRawBaroAltitudeBlocking(raw_alt)) { sum_alt += raw_alt; valid_samples++; } }
    if (valid_samples > 0) {
      baro_ground_alt_m = sum_alt / (float)valid_samples;
      lpf_baro_alt.output = 0.0f; lpf_baro_vz.output = 0.0f; baro_alt_m = 0.0f; baro_vz_mps = 0.0f;
      ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE; baro_cal_ready = true;
      Serial.println("OK: cal_bar");
    } else Serial.println("ERR: cal_bar");
  }

  void initMagHardware() {
    if (!activeSensors.mag_found) return;
    magCal.asa_x = 1.0f; magCal.asa_y = 1.0f; magCal.asa_z = 1.0f;
    if (activeSensors.mag_type == MAG_AK8963) {
      writeReg(activeSensors.mag_addr, 0x0A, 0x00); delay(5); writeReg(activeSensors.mag_addr, 0x0A, 0x1F); delay(5);
      uint8_t asa[3] = {128, 128, 128};
      if (readReg(activeSensors.mag_addr, 0x10, asa, 3)) {
        magCal.asa_x = (((float)asa[0] - 128.0f) / 256.0f) + 1.0f; magCal.asa_y = (((float)asa[1] - 128.0f) / 256.0f) + 1.0f; magCal.asa_z = (((float)asa[2] - 128.0f) / 256.0f) + 1.0f;
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
      magCal.scale_x = magCal.scale_y = magCal.scale_z = (1.0f / 12000.0f) * 100.0f;
    } else if (activeSensors.mag_type == MAG_HMC5883L) {
      writeReg(activeSensors.mag_addr, 0x00, 0x78); writeReg(activeSensors.mag_addr, 0x01, 0x20); writeReg(activeSensors.mag_addr, 0x02, 0x00); delay(6);
      magCal.scale_x = magCal.scale_y = (1.0f / 1090.0f) * 100.0f; magCal.scale_z = (1.0f / 980.0f) * 100.0f;
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
      float pitch = asinf(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f));
      xh = x * cosf(pitch) + y * sinf(roll) * sinf(pitch) + z * cosf(roll) * sinf(pitch);
      yh = y * cosf(roll) - z * sinf(roll);
    }
    float h = atan2f(yh, xh) * FC_RAD_TO_DEG;
    if (isnan(h) || isinf(h)) { setFcErrorRam(FC_ERR_NAN_INF_MATH); return false; }
    heading_deg = wrapDeg0To360(h); return true;
  }

  void detectMag() {
    uint8_t id = 0, req_id = currentConfig.com_id;
    activeSensors.mag_found = false; activeSensors.mag_type = MAG_UNKNOWN;
    if (activeSensors.imu_found && (activeSensors.imu_type == IMU_MPU9250 || activeSensors.imu_type == IMU_MPU6500)) enableImuI2cBypass();
    if (req_id == MAG_AK8963 || req_id == MAG_UNKNOWN) {
      if (readReg(0x0C, 0x00, &id, 1) && id == 0x48) { bindAndInitMag(0x0C, MAG_AK8963); return; }
      if (req_id == MAG_AK8963) goto mag_not_found;
    }
    if (req_id == MAG_AK09911 || req_id == MAG_UNKNOWN) {
      for (uint8_t addr : (const uint8_t[]){0x0C, 0x0E}) {
        uint8_t wia[2] = {0, 0}; if (readReg(addr, 0x00, wia, 2) && wia[0] == 0x48 && wia[1] == 0x05) { bindAndInitMag(addr, MAG_AK09911); return; }
      }
      if (req_id == MAG_AK09911) goto mag_not_found;
    }
    if (req_id == MAG_HSCDTD008A || req_id == MAG_UNKNOWN) {
      for (uint8_t addr : (const uint8_t[]){0x0C, 0x0F}) if (readReg(addr, 0x0F, &id, 1) && id == 0x49) { bindAndInitMag(addr, MAG_HSCDTD008A); return; }
      if (req_id == MAG_HSCDTD008A) goto mag_not_found;
    }
    if (req_id == MAG_QMC5883L || req_id == MAG_UNKNOWN) {
      if (readReg(0x0D, 0x0D, &id, 1) && id != 0x00) { bindAndInitMag(0x0D, MAG_QMC5883L); return; }
      if (req_id == MAG_QMC5883L) goto mag_not_found;
    }
    if (req_id == MAG_HMC5883L || req_id == MAG_UNKNOWN) {
      uint8_t hmc_id[3] = {0}; if (readReg(0x1E, 0x0A, hmc_id, 3) && hmc_id[0] == 'H') { bindAndInitMag(0x1E, MAG_HMC5883L); return; }
    }
  mag_not_found:
    if (currentConfig.com_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_MAG_FAIL); }
  }

  void detectLidar() {
    uint8_t req_id = currentConfig.lid_id;
    activeSensors.lidar_found = false; activeSensors.lidar_type = LIDAR_NONE;
    checkBusAndRecover(); Wire.beginTransmission(0x29);
    if (Wire.endTransmission() != 0) { if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); } return; }
    uint8_t id_l0x = 0; bool is_l0x = (readReg(0x29, 0xC0, &id_l0x, 1) && (id_l0x == 0xEE || id_l0x == 0xAA));
    uint8_t model_id[2] = {0, 0}; bool has_model = readReg16Addr(0x29, 0x010F, model_id, 2);
    if (req_id == LIDAR_VL53L0X && is_l0x) { activeSensors.lidar_type = LIDAR_VL53L0X; activeSensors.lidar_max_m = 2.0f; activeSensors.lidar_found = true; return; }
    if (req_id == LIDAR_VL53L1X && has_model && model_id[0] == 0xEA) { activeSensors.lidar_type = LIDAR_VL53L1X; activeSensors.lidar_max_m = 4.0f; activeSensors.lidar_found = true; return; }
    if (req_id == LIDAR_VL53L3CX && has_model && (model_id[0] == 0xEB || model_id[0] == 0xED) && model_id[1] <= 0x40) { activeSensors.lidar_type = LIDAR_VL53L3CX; activeSensors.lidar_max_m = 3.0f; activeSensors.lidar_found = true; return; }
    if (req_id == LIDAR_VL53L4CX && has_model && (model_id[0] == 0xEB || model_id[0] == 0xED) && model_id[1] > 0x40) { activeSensors.lidar_type = LIDAR_VL53L4CX; activeSensors.lidar_max_m = 6.0f; activeSensors.lidar_found = true; return; }
    if (req_id == LIDAR_NONE) {
      if (is_l0x) { activeSensors.lidar_type = LIDAR_VL53L0X; activeSensors.lidar_max_m = 2.0f; activeSensors.lidar_found = true; return; }
      if (has_model && model_id[0] == 0xEA) { activeSensors.lidar_type = LIDAR_VL53L1X; activeSensors.lidar_max_m = 4.0f; activeSensors.lidar_found = true; return; }
      if (has_model && (model_id[0] == 0xEB || model_id[0] == 0xED)) {
        activeSensors.lidar_type = (model_id[1] > 0x40) ? LIDAR_VL53L4CX : LIDAR_VL53L3CX;
        activeSensors.lidar_max_m = (activeSensors.lidar_type == LIDAR_VL53L4CX) ? 6.0f : 3.0f; activeSensors.lidar_found = true; return;
      }
    }
    if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); }
  }

  void configureLidarROI() {
    if (!activeSensors.lidar_found) return;
    if (activeSensors.lidar_type == LIDAR_VL53L1X || activeSensors.lidar_type == LIDAR_VL53L3CX || activeSensors.lidar_type == LIDAR_VL53L4CX) {
      writeReg16Addr8Val(0x29, 0x007F, 0x7F); writeReg16Addr8Val(0x29, 0x0080, 0x77);
    }
  }

  void initLidarHardware() {
    if (!activeSensors.lidar_found || !currentConfig.lidar_en) return;
    checkBusAndRecover();
    if (activeSensors.lidar_type == LIDAR_VL53L0X) { writeReg(0x29, 0x00, 0x01); delay(10); }
    else if (activeSensors.lidar_type != LIDAR_NONE) {
      writeReg16Addr8Val(0x29, 0x0000, 0x00); delay(10); configureLidarROI();
      writeReg16Addr8Val(0x29, 0x0002, 0x00); writeReg16Addr8Val(0x29, 0x0086, 0x40);
    }
  }

  bool readRawLidarSlantMeters(float &slant_range_m) {
    if (!activeSensors.lidar_found) return false;
    uint16_t raw_dist_mm = 0;
    if (activeSensors.lidar_type == LIDAR_VL53L0X) {
      uint8_t d[3] = {0};
      if (!readReg(0x29, 0x13, d, 3)) { if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); } return false; }
      uint8_t st = (d[0] >> 3) & 0x0F; if (st != 0 && st != 11) return false;
      raw_dist_mm = ((uint16_t)d[1] << 8) | d[2];
    } else {
      uint8_t blk[15] = {0};
      if (!readReg16Addr(0x29, 0x0089, blk, 15)) { if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); } return false; }
      uint8_t st = blk[0] & 0x1F; if (st != 0 && st != 9) return false;
      raw_dist_mm = ((uint16_t)blk[13] << 8) | blk[14];
    }
    if (raw_dist_mm == 0 || raw_dist_mm > (uint16_t)(getEffectiveLidarMaxM() * 1000.0f)) return false;
    slant_range_m = (float)raw_dist_mm / 1000.0f; return true;
  }

  float computeCorrectedLidarMeters(float slant_range_m) {
    float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
    float vertical_factor = (2.0f * (q1 * q3 - q0 * q2) * SIN_45) + (((q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3)) * COS_45);
    if (vertical_factor < 0.15f) vertical_factor = 0.15f;
    float corrected_m = (slant_range_m * vertical_factor) + currentConfig.lidar_offset_m;
    return (corrected_m < 0.0f) ? 0.0f : corrected_m;
  }

  bool readLidar(float &distance_m) {
    if (!currentConfig.lidar_en || !activeSensors.lidar_found) return false;
    float slant_range_m = 0.0f;
    if (!readRawLidarSlantMeters(slant_range_m)) return false;
    dbg_lidar_m = adaptive_lidar.update(computeCorrectedLidarMeters(slant_range_m), currentConfig.lidar_filter_alpha);
    distance_m = dbg_lidar_m; last_lidar_packet_ms = millis(); return true;
  }

  void calibrateLidar() {
    if (!activeSensors.lidar_found) { Serial.println("ERR: lidar"); return; }
    float sum_raw_m = 0.0f; uint8_t ok_cnt = 0;
    for (uint8_t i = 0; i < 20; i++) { float slant_m = 0.0f; if (readRawLidarSlantMeters(slant_m)) { sum_raw_m += slant_m; ok_cnt++; } delay(15); }
    if (ok_cnt > 0) { currentConfig.lidar_offset_m = -(sum_raw_m / (float)ok_cnt); adaptive_lidar.reset(); saveSettings(); Serial.println("OK: cal_lid"); }
    else Serial.println("ERR: cal_lid");
  }

  void calibrateBatVoltage(float actual_voltage) {
    if (actual_voltage < 1.0f || actual_voltage > 60.0f || isnan(actual_voltage) || isinf(actual_voltage)) { Serial.println("ERR: cal_bat"); return; }
    long sum_adc = 0;
    for (uint8_t i = 0; i < 32; i++) { sum_adc += analogRead(BATTERY_PIN); delay(2); }
    float bat_pin_v = (((float)sum_adc / 32.0f) / 4095.0f) * 3.3f;
    if (bat_pin_v < 0.01f) { Serial.println("ERR: adc"); return; }
    currentConfig.bat_scale_v = actual_voltage / bat_pin_v;
    v_batt = actual_voltage; batt_valid = true; lpf_battery.output = actual_voltage; power_w = v_batt * current_a;
    saveSettings(); Serial.println("OK: cal_bat");
  }

  void calibrateCurrent(bool has_value, float actual_current_a) {
    long sum_curr_adc = 0;
    for (uint8_t i = 0; i < 32; i++) { sum_curr_adc += analogRead(CURRENT_PIN); delay(2); }
    float shunt_a = ((((float)sum_curr_adc / 32.0f) / 4095.0f) * 3.3f) / SHUNT_RESISTANCE_OHM;
    if (!has_value || fabsf(actual_current_a) < 0.001f) {
      currentConfig.bat_offset_a = shunt_a; current_a = 0.0f; power_w = 0.0f; lpf_current.output = 0.0f;
      saveSettings(); Serial.println("OK: cal_curr"); return;
    }
    if (isnan(actual_current_a) || isinf(actual_current_a) || actual_current_a < 0.0f || actual_current_a > 250.0f) { Serial.println("ERR: cal_curr"); return; }
    float delta_a = shunt_a - currentConfig.bat_offset_a;
    if (fabsf(delta_a) < 0.01f) { Serial.println("ERR: cal_curr"); return; }
    currentConfig.bat_scale_a = constrain(actual_current_a / delta_a, 0.05f, 20.0f);
    current_a = actual_current_a; lpf_current.output = actual_current_a; power_w = v_batt * current_a;
    saveSettings(); Serial.println("OK: cal_curr");
  }

  bool calibrateGyro(bool silent) {
    if (!activeSensors.imu_found) { if (!silent) Serial.println("ERR: imu"); return false; }
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f, sum_temp = 0.0f; uint16_t ok_cnt = 0;
    for (uint16_t i = 0; i < 200; i++) {
      uint8_t buf[14];
      if (readMPU6050(buf)) {
        float gx = (float)((int16_t)(buf[8] << 8 | buf[9])) / activeSensors.gyro_scale;
        float gy = (float)((int16_t)(buf[10] << 8 | buf[11])) / activeSensors.gyro_scale;
        float gz = (float)((int16_t)(buf[12] << 8 | buf[13])) / activeSensors.gyro_scale;
        rotateXYByYawAlign(gx, gy, currentConfig.imu_align);
        sum_gx += gx; sum_gy += gy; sum_gz += gz; sum_temp += extractTempFromImuBuf(buf); ok_cnt++;
      }
      delay(4);
    }
    if (ok_cnt == 0) { if (!silent) Serial.println("ERR: cal_gyro"); return false; }
    currentConfig.gyro_bias_x = sum_gx / (float)ok_cnt; currentConfig.gyro_bias_y = sum_gy / (float)ok_cnt;
    currentConfig.gyro_bias_z = sum_gz / (float)ok_cnt; currentConfig.base_temperature = sum_temp / (float)ok_cnt;
    if (!silent) { saveSettings(); Serial.println("OK: cal_gyro"); }
    return true;
  }

  bool calibrateAcc(bool silent) {
    if (!activeSensors.imu_found) { if (!silent) Serial.println("ERR: imu"); return false; }
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f; uint16_t ok_cnt = 0;
    for (uint16_t i = 0; i < 200; i++) {
      uint8_t buf[14];
      if (readMPU6050(buf)) {
        float ax = (float)((int16_t)(buf[0] << 8 | buf[1])) / activeSensors.acc_scale;
        float ay = (float)((int16_t)(buf[2] << 8 | buf[3])) / activeSensors.acc_scale;
        float az = (float)((int16_t)(buf[4] << 8 | buf[5])) / activeSensors.acc_scale;
        rotateXYByYawAlign(ax, ay, currentConfig.imu_align);
        sum_ax += ax; sum_ay += ay; sum_az += az; ok_cnt++;
      }
      delay(4);
    }
    if (ok_cnt == 0) { if (!silent) Serial.println("ERR: cal_acc"); return false; }
    currentConfig.acc_bias_x = sum_ax / (float)ok_cnt; currentConfig.acc_bias_y = sum_ay / (float)ok_cnt; currentConfig.acc_bias_z = (sum_az / (float)ok_cnt) - 1.0f;
    if (!silent) { saveSettings(); Serial.println("OK: cal_acc"); }
    return true;
  }

  void calibrateIMU() {
    if (!activeSensors.imu_found) { Serial.println("ERR: imu"); return; }
    if (!calibrateGyro(true) || !calibrateAcc(true)) { Serial.println("ERR: cal_imu"); return; }
    saveSettings(); Serial.println("OK: cal_imu");
  }

  void autoDetectAll() {
    err_aux_i2c_fail = false; detectIMU();
    if (currentConfig.baro_en) detectBaro(); else { activeSensors.baro_found = false; activeSensors.baro_type = BARO_UNKNOWN; }
    if (currentConfig.com_en) detectMag(); else { activeSensors.mag_found = false; activeSensors.mag_type = MAG_UNKNOWN; }
    if (currentConfig.lidar_en) { detectLidar(); initLidarHardware(); } else { activeSensors.lidar_found = false; activeSensors.lidar_type = LIDAR_NONE; }
  }
} // namespace SensorDriver

void startMagCalibrationNonBlocking() {
  if (!SensorDriver::activeSensors.mag_found) { Serial.println("ERR: mag"); return; }
  mag_cal_active = true; mag_cal_start_ms = millis(); mag_cal_last_sample_ms = 0;
  mag_cal_min_x = mag_cal_min_y = mag_cal_min_z = 1e9f; mag_cal_max_x = mag_cal_max_y = mag_cal_max_z = -1e9f;
  Serial.println("OK: cal_mag");
}

void stepMagCalibrationNonBlocking(uint32_t now_ms) {
  if (!mag_cal_active) return;
  if ((now_ms - mag_cal_last_sample_ms) >= MAG_CAL_SAMPLE_MS) {
    mag_cal_last_sample_ms = now_ms;
    float mx_u, my_u, mz_u;
    if (SensorDriver::readMagUnified(mx_u, my_u, mz_u)) {
      if (mx_u < mag_cal_min_x) mag_cal_min_x = mx_u; if (mx_u > mag_cal_max_x) mag_cal_max_x = mx_u;
      if (my_u < mag_cal_min_y) mag_cal_min_y = my_u; if (my_u > mag_cal_max_y) mag_cal_max_y = my_u;
      if (mz_u < mag_cal_min_z) mag_cal_min_z = mz_u; if (mz_u > mag_cal_max_z) mag_cal_max_z = mz_u;
    }
  }
  if ((now_ms - mag_cal_start_ms) >= MAG_CAL_DURATION_MS) {
    mag_cal_active = false;
    if (mag_cal_max_x > mag_cal_min_x && mag_cal_max_y > mag_cal_min_y && mag_cal_max_z > mag_cal_min_z) {
      currentConfig.mag_bias_x = 0.5f * (mag_cal_min_x + mag_cal_max_x);
      currentConfig.mag_bias_y = 0.5f * (mag_cal_min_y + mag_cal_max_y);
      currentConfig.mag_bias_z = 0.5f * (mag_cal_min_z + mag_cal_max_z);
      saveSettings(); Serial.println("DONE: cal_mag");
    } else Serial.println("ERR: cal_mag");
  }
}

void stepMonitorStreamNonBlocking(uint32_t now_ms) {
  if (!mon_stream_active || (now_ms - mon_last_send_ms) < MON_STREAM_PERIOD_MS) return;
  mon_last_send_ms = now_ms;
  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float r = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * FC_RAD_TO_DEG;
  float p = asinf(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * FC_RAD_TO_DEG;
  float y = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * FC_RAD_TO_DEG;
  Serial.print("MON: err="); Serial.print((int)getActiveStatusErrorCode());
  Serial.print(" arm="); Serial.print(is_armed ? 1 : 0);
  Serial.print(" fs="); Serial.print(is_failsafe ? 1 : 0);
  Serial.print(" cpu="); Serial.print(cpu_load_pct, 1); Serial.print("/"); Serial.print(cpu_max_load_pct, 1);
  Serial.print(" vib="); Serial.print(vibration_index, 1);
  Serial.print(" rpy="); Serial.print(r, 1); Serial.print(","); Serial.print(p, 1); Serial.print(","); Serial.print(y, 1);
  Serial.print(" alt="); Serial.print(baro_alt_m, 2); Serial.print(" lid="); Serial.print(dbg_lidar_m, 2);
  Serial.print(" hdg="); Serial.print(compass_heading_deg, 1); Serial.print(" v="); Serial.print(v_batt, 2);
  Serial.print(" i="); Serial.print(current_a, 2); Serial.print(" w="); Serial.print(power_w, 1);
  Serial.print(" lq="); Serial.println((int)current_link_quality);
}

void readBatteryAndCurrent() {
  float instant_v = (((float)analogRead(BATTERY_PIN) / 4095.0f) * 3.3f) * SensorDriver::getBatScaleV();
  if (isnan(instant_v) || isinf(instant_v) || instant_v < 0.0f || instant_v > 60.0f) {
    batt_valid = false; err_batt_low = false; setFcErrorRam(FC_ERR_BAT_SENSOR_FAIL); return;
  }
  if (instant_v > 0.5f) {
    if (!batt_valid) lpf_battery.output = instant_v;
    batt_valid = true;
  }
  v_batt = lpf_battery.update(instant_v);
  float instant_a = SensorDriver::computeCurrentAmpsFromAdc(analogRead(CURRENT_PIN));
  current_a = lpf_current.update(instant_a);
  power_w = v_batt * current_a;
  if (is_armed) consumed_wh += (power_w * DT) / 3600.0f;
  if (currentConfig.fs_bat_en && batt_valid && v_batt < currentConfig.bat_fs_v) { err_batt_low = true; setFcErrorRam(FC_ERR_BAT_LOW); }
  else err_batt_low = false;
}

void updateAuxSensorsNonBlocking(uint32_t now_ms) {
  static uint32_t last_aux_ms = 0, last_baro_vz_ms = 0;
  float alt_m = 0.0f;
  if (SensorDriver::stepBaroNonBlocking(now_ms, alt_m)) {
    baro_alt_m = alt_m;
    if (last_baro_vz_ms > 0 && (now_ms - last_baro_vz_ms) >= 10) {
      baro_vz_mps = lpf_baro_vz.update(constrain((baro_alt_m - lpf_baro_alt.output) / ((float)(now_ms - last_baro_vz_ms) * 0.001f), -4.0f, 4.0f));
    }
    last_baro_vz_ms = now_ms;
  }
  if ((now_ms - last_aux_ms) < AUX_SENSOR_PERIOD_MS) return;
  last_aux_ms = now_ms;
  float lidar_m = 0.0f;
  if (SensorDriver::readLidar(lidar_m)) {
    current_vz = constrain((lidar_m - adaptive_lidar.output) / (AUX_SENSOR_PERIOD_MS * 0.001f), -4.0f, 4.0f);
  }
  float hdg_deg = 0.0f;
  if (SensorDriver::readCompassHeadingDeg(hdg_deg)) { compass_heading_deg = hdg_deg; compass_valid = true; }
  else compass_valid = false;
  readBatteryAndCurrent();
}

// ==============================================================================
// 9. АВТОКАЛИБРОВКА И ВЗЛЁТ/ПОСАДКА (CliAutoCal)
// ==============================================================================
namespace CliAutoCal {
  constexpr uint8_t BTN_PIN = PB8;
  constexpr uint32_t BTN_DEBOUNCE_MS = 25;
  constexpr uint32_t BTN_HOLD_MS = 1200;

  constexpr float TARGET_ALT_M = 1.10f;
  constexpr float TAKEOFF_VZ_MPS = 0.40f;
  constexpr float LANDING_VZ_MPS = -0.35f;
  constexpr float GROUND_EFFECT_M = 0.35f;
  constexpr float TOUCHDOWN_M = 0.07f;
  constexpr float TILT_LIMIT_DEG = 45.0f;
  constexpr float SHOCK_G_DELTA = 1.35f;
  constexpr uint32_t HOVER_CAL_MS = 5000;
  constexpr uint32_t SPIN_RAMP_MS = 1500;
  constexpr uint32_t PHASE_TIMEOUT_MS = 12000;

  enum State : uint8_t {
    IDLE = 0, PRECHECK, CAL_GROUND, SPIN_RAMP, TAKEOFF, HOVER_CAL, LANDING, SAVE_DONE, ABORT
  };

  enum AbortReason : uint8_t {
    REASON_NONE = 0, REASON_USER_CANCEL, REASON_ARMED_CONFLICT, REASON_SENSOR_FAIL,
    REASON_BAT_LOW, REASON_TILT_DETECTED, REASON_TIMEOUT
  };

  static bool btn_raw = false;
  static bool btn_debounced = false;
  static uint32_t btn_debounce_ms = 0;
  static uint32_t btn_press_ms = 0;
  static bool btn_hold_fired = false;

  static State state = IDLE;
  static AbortReason abort_reason = REASON_NONE;
  static uint32_t state_start_ms = 0;
  static bool flight_active = false;
  static bool hazard_latched = false;

  static float hover_sum = 0.0f;
  static uint16_t hover_samples = 0;
  static float prev_roll_kp = 0.0f, prev_pitch_kp = 0.0f, prev_roll_kd = 0.0f, prev_pitch_kd = 0.0f;
  static float peak_vib = 0.0f;
  static bool pid_step_applied = false;
  static float lidar_prev_m = 0.0f, lidar_vz_mps = 0.0f;
  static uint32_t lidar_prev_ms = 0;

  static void restoreTempPid() {
    if (!pid_step_applied) return;
    currentConfig.roll_pid.kp = prev_roll_kp;
    currentConfig.pitch_pid.kp = prev_pitch_kp;
    currentConfig.roll_pid.kd = prev_roll_kd;
    currentConfig.pitch_pid.kd = prev_pitch_kd;
    updatePidTunings();
    pid_step_applied = false;
  }

  static void enterState(State next_state, uint32_t now_ms) {
    state = next_state;
    state_start_ms = now_ms;
    peak_vib = 0.0f;
  }

  void stop(uint8_t reason_code, uint8_t fc_err_code) {
    restoreTempPid();
    abort_reason = (AbortReason)reason_code;
    flight_active = false;
    is_armed = false;
    arm_ramp_factor = 0.0f;
    vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
    resetMotorSlewState();
    setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);
    if (fc_err_code != FC_ERR_NONE) setFcErrorRam(fc_err_code);
    state = (abort_reason == REASON_USER_CANCEL) ? IDLE : ABORT;
  }

  void start() {
    uint32_t now_ms = millis();
    if (is_armed) { stop(REASON_ARMED_CONFLICT, FC_ERR_NONE); return; }
    stopBenchTest();
    hazard_latched = false; abort_reason = REASON_NONE;
    flight_active = false; hover_sum = 0.0f; hover_samples = 0;
    lidar_prev_m = 0.0f; lidar_vz_mps = 0.0f; lidar_prev_ms = now_ms;
    restoreTempPid();
    vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
    enterState(PRECHECK, now_ms);
  }

  void init() {
    pinMode(BTN_PIN, INPUT_PULLUP);
    bool pressed_now = (digitalRead(BTN_PIN) == LOW);
    uint32_t now_ms = millis();
    btn_raw = pressed_now; btn_debounced = pressed_now; btn_debounce_ms = now_ms;
    btn_press_ms = pressed_now ? now_ms : 0; btn_hold_fired = false;
  }

  void pollButton(uint32_t now_ms) {
    bool raw_now = (digitalRead(BTN_PIN) == LOW);
    if (raw_now != btn_raw) { btn_raw = raw_now; btn_debounce_ms = now_ms; }
    if ((now_ms - btn_debounce_ms) >= BTN_DEBOUNCE_MS && raw_now != btn_debounced) {
      btn_debounced = raw_now;
      btn_press_ms = raw_now ? now_ms : 0;
      btn_hold_fired = false;
    }
    if (btn_debounced && !btn_hold_fired && (now_ms - btn_press_ms) >= BTN_HOLD_MS) {
      btn_hold_fired = true;
      if (state != IDLE && state != ABORT && state != SAVE_DONE) {
        stop(REASON_USER_CANCEL, FC_ERR_NONE);
        Serial.println("OK: auto_stop");
      } else {
        start();
        Serial.println("OK: auto_cal");
      }
    }
  }

  bool isFlightActive() { return flight_active; }

  void onDisarmRequested() {
    if (flight_active) stop(REASON_USER_CANCEL, FC_ERR_NONE);
  }

  void onArmRequested() {
    if (state != IDLE) stop(REASON_ARMED_CONFLICT, FC_ERR_NONE);
  }

  void onAttitudeAndMotion(float ax_g, float ay_g, float az_g, float roll_deg, float pitch_deg, float gx_dps, float gy_dps) {
    float acc_mag_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    float shock_delta = fabsf(acc_mag_g - 1.0f);
    float tilt_peak_deg = fmaxf(fabsf(roll_deg), fabsf(pitch_deg));
    float gyro_xy_dps = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps);
    bool active_motion = is_armed || flight_active;
    bool tilt_exceeded = (tilt_peak_deg >= TILT_LIMIT_DEG) || (tilt_peak_deg >= 32.0f && gyro_xy_dps >= 260.0f);
    if (active_motion && (tilt_exceeded || shock_delta >= SHOCK_G_DELTA)) {
      hazard_latched = true;
      setFcErrorRam(FC_ERR_TILT_DISARM);
      is_armed = false;
      arm_ramp_factor = 0.0f;
      resetMotorSlewState();
      setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);
      if (state != IDLE && state != ABORT && state != SAVE_DONE) {
        stop(REASON_TILT_DETECTED, FC_ERR_TILT_DISARM);
      }
    }
  }

  static bool readFreshLidar(uint32_t now_ms, float &dist_m) {
    if (!currentConfig.lidar_en || !SensorDriver::activeSensors.lidar_found) return false;
    if ((now_ms - last_lidar_packet_ms) > 250) return false;
    if (isnan(dbg_lidar_m) || isinf(dbg_lidar_m) || dbg_lidar_m < 0.0f) return false;
    if (dbg_lidar_m > SensorDriver::getEffectiveLidarMaxM()) return false;
    dist_m = dbg_lidar_m;
    return true;
  }

  static void updateLidarVz(uint32_t now_ms, float lidar_m) {
    if (lidar_prev_ms == 0) { lidar_prev_m = lidar_m; lidar_prev_ms = now_ms; lidar_vz_mps = 0.0f; return; }
    uint32_t dt_ms = now_ms - lidar_prev_ms;
    if (dt_ms >= 10) {
      float raw_vz = constrain((lidar_m - lidar_prev_m) / ((float)dt_ms * 0.001f), -3.5f, 3.5f);
      lidar_vz_mps = 0.35f * raw_vz + 0.65f * lidar_vz_mps;
      lidar_prev_m = lidar_m; lidar_prev_ms = now_ms;
    }
  }

  bool stepFlight(uint32_t now_ms, float gx_dps, float gy_dps, float gz_dps,
                  float gx_rad, float gy_rad, float gz_rad,
                  float roll_deg, float pitch_deg,
                  float &m1, float &m2, float &m3, float &m4) {
    (void)gz_dps;
    if (state == IDLE || state == ABORT || state == SAVE_DONE) return false;
    if (hazard_latched) { stop(REASON_TILT_DETECTED, FC_ERR_TILT_DISARM); m1 = m2 = m3 = m4 = OUTPUT_MIN; return true; }
    if (currentConfig.fs_bat_en && batt_valid && v_batt < currentConfig.bat_fs_v) { stop(REASON_BAT_LOW, FC_ERR_BAT_LOW); m1 = m2 = m3 = m4 = OUTPUT_MIN; return true; }

    float lidar_m = 0.0f;
    bool lidar_ok = readFreshLidar(now_ms, lidar_m);
    if (lidar_ok) updateLidarVz(now_ms, lidar_m);
    if (vibration_index > peak_vib) peak_vib = vibration_index;
    uint32_t elapsed_ms = now_ms - state_start_ms;

    if (state == PRECHECK) {
      m1 = m2 = m3 = m4 = OUTPUT_MIN;
      if (!SensorDriver::activeSensors.imu_found || err_imu_fail) { stop(REASON_SENSOR_FAIL, FC_ERR_IMU_FAIL); return true; }
      if (!lidar_ok) { if (elapsed_ms >= 2000) stop(REASON_SENSOR_FAIL, FC_ERR_LIDAR_FAIL); return true; }
      enterState(CAL_GROUND, now_ms);
      return true;
    }

    if (state == CAL_GROUND) {
      m1 = m2 = m3 = m4 = OUTPUT_MIN;
      if (!lidar_ok) { stop(REASON_SENSOR_FAIL, FC_ERR_LIDAR_FAIL); return true; }
      if (!SensorDriver::calibrateGyro(true) || !SensorDriver::calibrateAcc(true)) { stop(REASON_SENSOR_FAIL, FC_ERR_IMU_FAIL); return true; }
      currentConfig.lidar_offset_m = -lidar_m;
      adaptive_lidar.reset();
      orientation.q0 = 1.0f; orientation.q1 = 0.0f; orientation.q2 = 0.0f; orientation.q3 = 0.0f;
      vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
      hover_sum = 0.0f; hover_samples = 0;
      flight_active = true;
      enterState(SPIN_RAMP, now_ms);
      return true;
    }

    if (!lidar_ok) {
      if (state == TAKEOFF || state == HOVER_CAL) enterState(LANDING, now_ms);
      else if (elapsed_ms >= 1500) { stop(REASON_SENSOR_FAIL, FC_ERR_LIDAR_FAIL); return true; }
    }

    if (state == SPIN_RAMP) {
      float frac = constrain((float)elapsed_ms / (float)SPIN_RAMP_MS, 0.0f, 1.0f);
      float ramp_thr = base_hover_throttle * 0.45f * frac;
      float r_cmd = rollRatePID.update(0.0f - gx_rad, gx_rad, DT, false);
      float p_cmd = pitchRatePID.update(0.0f - gy_rad, gy_rad, DT, false);
      float y_cmd = yawRatePID.update(0.0f - gz_rad, gz_rad, DT, false);
      m1 = constrain(ramp_thr + p_cmd + r_cmd - y_cmd, OUTPUT_MIN, 450.0f);
      m2 = constrain(ramp_thr + p_cmd - r_cmd + y_cmd, OUTPUT_MIN, 450.0f);
      m3 = constrain(ramp_thr - p_cmd - r_cmd - y_cmd, OUTPUT_MIN, 450.0f);
      m4 = constrain(ramp_thr - p_cmd + r_cmd + y_cmd, OUTPUT_MIN, 450.0f);
      if (elapsed_ms >= SPIN_RAMP_MS) enterState(TAKEOFF, now_ms);
      return true;
    }

    float desired_vz = 0.0f;
    if (state == TAKEOFF) {
      desired_vz = constrain((TARGET_ALT_M - lidar_m) * 1.2f, LANDING_VZ_MPS, TAKEOFF_VZ_MPS);
      if (lidar_ok && lidar_m >= (TARGET_ALT_M - 0.08f) && fabsf(lidar_vz_mps) < 0.25f) enterState(HOVER_CAL, now_ms);
      else if (elapsed_ms >= PHASE_TIMEOUT_MS) enterState(LANDING, now_ms);
    } else if (state == HOVER_CAL) {
      desired_vz = constrain((TARGET_ALT_M - lidar_m) * 1.1f, -0.45f, 0.45f);
      if (elapsed_ms < (HOVER_CAL_MS / 2)) {
        if (!pid_step_applied) {
          prev_roll_kp = currentConfig.roll_pid.kp; prev_pitch_kp = currentConfig.pitch_pid.kp;
          prev_roll_kd = currentConfig.roll_pid.kd; prev_pitch_kd = currentConfig.pitch_pid.kd;
          currentConfig.roll_pid.kp *= 1.12f; currentConfig.pitch_pid.kp *= 1.12f;
          currentConfig.roll_pid.kd *= 1.10f; currentConfig.pitch_pid.kd *= 1.10f;
          updatePidTunings();
          pid_step_applied = true;
          peak_vib = vibration_index;
        }
      } else if (pid_step_applied) {
        if (peak_vib > 35.0f) restoreTempPid();
        else pid_step_applied = false;
      }
      if (elapsed_ms >= HOVER_CAL_MS) {
        if (hover_samples > 0) {
          float avg_hover = hover_sum / (float)hover_samples;
          if (avg_hover > 50.0f && avg_hover < 850.0f) base_hover_throttle = avg_hover;
        }
        restoreTempPid();
        enterState(LANDING, now_ms);
      }
    } else if (state == LANDING) {
      desired_vz = LANDING_VZ_MPS * ((lidar_ok && lidar_m < GROUND_EFFECT_M) ? 0.55f : 1.0f);
      if (lidar_ok && lidar_m <= TOUCHDOWN_M && fabsf(lidar_vz_mps) < 0.30f) {
        stop(REASON_NONE, FC_ERR_NONE);
        saveSettings();
        state = SAVE_DONE;
        m1 = m2 = m3 = m4 = OUTPUT_MIN;
        return true;
      }
      if (elapsed_ms >= PHASE_TIMEOUT_MS) {
        stop(REASON_TIMEOUT, FC_ERR_NONE);
        m1 = m2 = m3 = m4 = OUTPUT_MIN;
        return true;
      }
    }

    float vz_meas = lidar_ok ? lidar_vz_mps : baro_vz_mps;
    float thr_cmd = constrain(base_hover_throttle + vzPID.update(desired_vz - vz_meas, vz_meas, DT, false), 40.0f, 850.0f);
    if (state == HOVER_CAL && lidar_ok && fabsf(TARGET_ALT_M - lidar_m) < 0.15f && fabsf(vz_meas) < 0.20f) {
      hover_sum += thr_cmd; hover_samples++;
    }

    float target_gx_rad = constrain(constrain(-roll_deg, -currentConfig.fm_roll_max, currentConfig.fm_roll_max) * 2.5f * FC_DEG_TO_RAD, -3.0f, 3.0f);
    float target_gy_rad = constrain(constrain(-pitch_deg, -currentConfig.fm_pitch_max, currentConfig.fm_pitch_max) * 2.5f * FC_DEG_TO_RAD, -3.0f, 3.0f);
    float r_cmd = rollRatePID.update(target_gx_rad - gx_rad, gx_rad, DT, false);
    float p_cmd = pitchRatePID.update(target_gy_rad - gy_rad, gy_rad, DT, false);
    float y_cmd = yawRatePID.update(0.0f - gz_rad, gz_rad, DT, false);

    m1 = constrain(thr_cmd + p_cmd + r_cmd - y_cmd, OUTPUT_MIN, OUTPUT_MAX);
    m2 = constrain(thr_cmd + p_cmd - r_cmd + y_cmd, OUTPUT_MIN, OUTPUT_MAX);
    m3 = constrain(thr_cmd - p_cmd - r_cmd - y_cmd, OUTPUT_MIN, OUTPUT_MAX);
    m4 = constrain(thr_cmd - p_cmd + r_cmd + y_cmd, OUTPUT_MIN, OUTPUT_MAX);
    return true;
  }
} // namespace CliAutoCal

// ==============================================================================
// 10. ИНТЕРФЕЙС КОМАНДНОЙ СТРОКИ (CLI)
// ==============================================================================
enum CliValType : uint8_t { CLI_U8 = 0, CLI_I8 = 1, CLI_U16 = 2, CLI_F32 = 3 };

struct CliParam {
  const char* name;
  void* ptr;
  CliValType type;
  int min_v;
  int max_v;
};

using CliSideEffectFn = void (*)();
struct CliSideEffectRule { const char* key; CliSideEffectFn fn; };

using CliActionFn = void (*)(const char* args);
struct CliActionEntry { const char* name; bool has_args; bool requires_disarm; CliActionFn fn; };

const CliParam cliParams[] = {
  {"esc_prot", &currentConfig.esc_prot, CLI_U8, 0, 1},
  {"imu_align", &currentConfig.imu_align, CLI_U8, 0, 3},
  {"com_align", &currentConfig.com_align, CLI_U8, 0, 3},
  {"imu_id", &currentConfig.imu_id, CLI_U8, 0, 6},
  {"bar_id", &currentConfig.bar_id, CLI_U8, 0, 7},
  {"com_id", &currentConfig.com_id, CLI_U8, 0, 6},
  {"lid_id", &currentConfig.lid_id, CLI_U8, 0, 4},
  {"baro_en", &currentConfig.baro_en, CLI_U8, 0, 1},
  {"com_en", &currentConfig.com_en, CLI_U8, 0, 1},
  {"lidar_en", &currentConfig.lidar_en, CLI_U8, 0, 1},
  {"lidar_filter_alpha", &currentConfig.lidar_filter_alpha, CLI_F32, 0, 0},
  {"lidar_offset_m", &currentConfig.lidar_offset_m, CLI_F32, 0, 0},
  {"mag_bias_x", &currentConfig.mag_bias_x, CLI_F32, 0, 0},
  {"mag_bias_y", &currentConfig.mag_bias_y, CLI_F32, 0, 0},
  {"mag_bias_z", &currentConfig.mag_bias_z, CLI_F32, 0, 0},

  {"bat_scale_v", &currentConfig.bat_scale_v, CLI_F32, 0, 0},
  {"bat_filter_alpha", &currentConfig.bat_filter_alpha, CLI_F32, 0, 0},
  {"bat_offset_a", &currentConfig.bat_offset_a, CLI_F32, 0, 0},
  {"bat_scale_a", &currentConfig.bat_scale_a, CLI_F32, 0, 0},
  {"bat_comp_en", &currentConfig.bat_comp_en, CLI_U8, 0, 1},
  {"bat_v_max", &currentConfig.bat_v_max, CLI_F32, 70, 250},
  {"flt_esc_limit", &currentConfig.flt_esc_limit, CLI_F32, 1, 100},

  {"vtx_en", &currentConfig.vtx_en, CLI_U8, 0, 1},
  {"vtx_prot", &currentConfig.vtx_prot, CLI_U8, 0, 2},
  {"vtx_mhz", &currentConfig.vtx_mhz, CLI_U16, 5000, 6000},
  {"vtx_mw", &currentConfig.vtx_mw, CLI_U16, 1, 10000},

  {"rtx_id", &currentConfig.rtx_id, CLI_U8, 0, 3},
  {"rtx_en", &currentConfig.rtx_en, CLI_U8, 0, 1},
  {"rtx_band", &currentConfig.rtx_band, CLI_U8, 1, 9},
  {"rtx_hw_crc_en", &currentConfig.rtx_hw_crc_en, CLI_U8, 0, 1},
  {"rtx_addr_en", &currentConfig.rtx_addr_en, CLI_U8, 0, 1},
  {"rtx_addr", &currentConfig.rtx_addr, CLI_U16, 0, 65535},
  {"rtx_sync_word", &currentConfig.rtx_sync_word, CLI_U16, 0, 65535},
  {"rtx_iq_inv", &currentConfig.rtx_iq_inv, CLI_U8, 0, 1},
  {"rtx_pri_chan", &currentConfig.rtx_pri_chan, CLI_U8, 0, 99},
  {"rtx_pri_sf", &currentConfig.rtx_pri_sf, CLI_U8, 5, 12},
  {"rtx_pri_bw", &currentConfig.rtx_pri_bw, CLI_U8, 0, 9},
  {"rtx_pri_cr", &currentConfig.rtx_pri_cr, CLI_U8, 1, 4},
  {"rtx_pri_power", &currentConfig.rtx_pri_power, CLI_I8, -9, 22},
  {"rtx_bak_chan", &currentConfig.rtx_bak_chan, CLI_U8, 0, 99},
  {"rtx_bak_sf", &currentConfig.rtx_bak_sf, CLI_U8, 5, 12},
  {"rtx_bak_bw", &currentConfig.rtx_bak_bw, CLI_U8, 0, 9},
  {"rtx_bak_cr", &currentConfig.rtx_bak_cr, CLI_U8, 1, 4},
  {"rtx_bak_power", &currentConfig.rtx_bak_power, CLI_I8, -9, 22},

  {"fm_mode", &currentConfig.fm_mode, CLI_U8, 0, 3},
  {"fm_roll_max", &currentConfig.fm_roll_max, CLI_F32, 0, 0},
  {"fm_pitch_max", &currentConfig.fm_pitch_max, CLI_F32, 0, 0},
  {"fm_vz_speed_max", &currentConfig.fm_vz_speed_max, CLI_F32, 0, 0},
  {"rc_rate", &currentConfig.rc_rate, CLI_F32, 0, 0},
  {"rc_super_expo", &currentConfig.rc_super_expo, CLI_F32, 0, 0},
  {"rc_stick_expo", &currentConfig.rc_stick_expo, CLI_F32, 0, 0},
  {"rc_yaw_rate", &currentConfig.rc_yaw_rate, CLI_F32, 0, 0},
  {"rc_yaw_super_expo", &currentConfig.rc_yaw_super_expo, CLI_F32, 0, 0},
  {"rc_yaw_expo", &currentConfig.rc_yaw_expo, CLI_F32, 0, 0},
  {"rc_deadband", &currentConfig.rc_deadband, CLI_F32, 0, 0},
  {"rc_horizon_transition", &currentConfig.rc_horizon_transition, CLI_F32, 0, 0},
  {"rc_filter_smoothing", &currentConfig.rc_filter_smoothing, CLI_F32, 0, 0},

  {"fs_bat_en", &currentConfig.fs_bat_en, CLI_U8, 0, 1},
  {"fs_rtx_en", &currentConfig.fs_rtx_en, CLI_U8, 0, 1},
  {"fs_vz_speed", &currentConfig.fs_vz_speed, CLI_F32, 0, 0},
  {"bat_fs_v", &currentConfig.bat_fs_v, CLI_F32, 0, 0},

  {"flt_amb_en", &currentConfig.flt_amb_en, CLI_U8, 0, 1},
  {"flt_lpf_en", &currentConfig.flt_lpf_en, CLI_U8, 0, 1},
  {"flt_not_en", &currentConfig.flt_not_en, CLI_U8, 0, 1},
  {"gyro_filter_alpha", &currentConfig.gyro_filter_alpha, CLI_F32, 0, 0},
  {"acc_filter_alpha", &currentConfig.acc_filter_alpha, CLI_F32, 0, 0},
  {"filter_notch_freq", &currentConfig.filter_notch_freq, CLI_F32, 0, 0},
  {"filter_notch_bw", &currentConfig.filter_notch_bw, CLI_F32, 0, 0},
  {"flt_amb", &currentConfig.flt_amb, CLI_F32, 0, 0},
  {"flt_amb_min", &currentConfig.flt_amb_min, CLI_F32, 0, 0},

  {"roll_kp", &currentConfig.roll_pid.kp, CLI_F32, 0, 0},
  {"roll_ki", &currentConfig.roll_pid.ki, CLI_F32, 0, 0},
  {"roll_kd", &currentConfig.roll_pid.kd, CLI_F32, 0, 0},
  {"pitch_kp", &currentConfig.pitch_pid.kp, CLI_F32, 0, 0},
  {"pitch_ki", &currentConfig.pitch_pid.ki, CLI_F32, 0, 0},
  {"pitch_kd", &currentConfig.pitch_pid.kd, CLI_F32, 0, 0},
  {"yaw_kp", &currentConfig.yaw_pid.kp, CLI_F32, 0, 0},
  {"yaw_ki", &currentConfig.yaw_pid.ki, CLI_F32, 0, 0},
  {"yaw_kd", &currentConfig.yaw_pid.kd, CLI_F32, 0, 0},
  {"vz_kp", &currentConfig.vz_pid.kp, CLI_F32, 0, 0},
  {"vz_ki", &currentConfig.vz_pid.ki, CLI_F32, 0, 0},
  {"vz_kd", &currentConfig.vz_pid.kd, CLI_F32, 0, 0}
};

static void onVtxEnChanged() {
  if (!currentConfig.vtx_en) {
    vtx_link_ok = false;
    vtx_probe_step = VTX_STEP_IDLE;
  } else {
    vtxInit();
  }
  saveSettings();
}

static void onVtxProtChanged() {
  vtxInit();
  saveSettings();
}

static void onVtxMhzChanged() {
  vtxSetFrequencyMhz(currentConfig.vtx_mhz, true, true);
}

static void onVtxMwChanged() {
  vtxSetPowerMw(currentConfig.vtx_mw, true, true);
}

static void onRtxCfgChanged() {
  main_rtx_chan = constrain(currentConfig.rtx_pri_chan, 0, 99);
  if (!is_on_backup) active_rtx_chan = main_rtx_chan;
  if (!is_armed && currentConfig.rtx_en) applyRtxConfiguration(is_on_backup);
}

static void onImuCfgChanged() { if (!is_armed) SensorDriver::detectIMU(); }
static void onBaroCfgChanged() {
  if (!is_armed) {
    if (currentConfig.baro_en) SensorDriver::detectBaro();
    else { SensorDriver::activeSensors.baro_found = false; SensorDriver::baroCal.valid = false; }
  }
}
static void onMagCfgChanged() {
  if (!is_armed) {
    if (currentConfig.com_en) SensorDriver::detectMag();
    else { SensorDriver::activeSensors.mag_found = false; compass_valid = false; }
  }
}
static void onLidarCfgChanged() {
  if (!is_armed) {
    if (currentConfig.lidar_en) { SensorDriver::detectLidar(); SensorDriver::initLidarHardware(); }
    else { SensorDriver::activeSensors.lidar_found = false; }
  }
}
static void onDspOrPidChanged() {
  currentConfig.flt_amb = constrain(currentConfig.flt_amb, 0.001f, 1.0f);
  currentConfig.flt_amb_min = constrain(currentConfig.flt_amb_min, 0.0005f, currentConfig.flt_amb);
  currentConfig.flt_esc_limit = getEffectiveEscDeltaLimit();
  currentConfig.bat_v_max = getEffectiveBatVMax();
  applyFilterConfig();
  updatePidTunings();
}

static const CliSideEffectRule cliSideEffects[] = {
  {"bat_v_max", onDspOrPidChanged},
  {"flt_esc_limit", onDspOrPidChanged},
  {"vtx_en", onVtxEnChanged},
  {"vtx_prot", onVtxProtChanged},
  {"vtx_mhz", onVtxMhzChanged},
  {"vtx_mw", onVtxMwChanged},
  {"rtx_id", onRtxCfgChanged},
  {"rtx_en", onRtxCfgChanged},
  {"rtx_band", onRtxCfgChanged},
  {"rtx_hw_crc_en", onRtxCfgChanged},
  {"rtx_addr_en", onRtxCfgChanged},
  {"rtx_addr", onRtxCfgChanged},
  {"rtx_sync_word", onRtxCfgChanged},
  {"rtx_iq_inv", onRtxCfgChanged},
  {"rtx_pri_chan", onRtxCfgChanged},
  {"rtx_pri_sf", onRtxCfgChanged},
  {"rtx_pri_bw", onRtxCfgChanged},
  {"rtx_pri_cr", onRtxCfgChanged},
  {"rtx_pri_power", onRtxCfgChanged},
  {"rtx_bak_chan", onRtxCfgChanged},
  {"rtx_bak_sf", onRtxCfgChanged},
  {"rtx_bak_bw", onRtxCfgChanged},
  {"rtx_bak_cr", onRtxCfgChanged},
  {"rtx_bak_power", onRtxCfgChanged},
  {"imu_id", onImuCfgChanged},
  {"bar_id", onBaroCfgChanged},
  {"baro_en", onBaroCfgChanged},
  {"com_id", onMagCfgChanged},
  {"com_en", onMagCfgChanged},
  {"lid_id", onLidarCfgChanged},
  {"lidar_en", onLidarCfgChanged},
  {"flt_amb_en", onDspOrPidChanged},
  {"flt_lpf_en", onDspOrPidChanged},
  {"flt_not_en", onDspOrPidChanged},
  {"gyro_filter_alpha", onDspOrPidChanged},
  {"acc_filter_alpha", onDspOrPidChanged},
  {"filter_notch_freq", onDspOrPidChanged},
  {"filter_notch_bw", onDspOrPidChanged},
  {"flt_amb", onDspOrPidChanged},
  {"flt_amb_min", onDspOrPidChanged},
  {"bat_filter_alpha", onDspOrPidChanged},
  {"roll_kp", onDspOrPidChanged}, {"roll_ki", onDspOrPidChanged}, {"roll_kd", onDspOrPidChanged},
  {"pitch_kp", onDspOrPidChanged}, {"pitch_ki", onDspOrPidChanged}, {"pitch_kd", onDspOrPidChanged},
  {"yaw_kp", onDspOrPidChanged}, {"yaw_ki", onDspOrPidChanged}, {"yaw_kd", onDspOrPidChanged},
  {"vz_kp", onDspOrPidChanged}, {"vz_ki", onDspOrPidChanged}, {"vz_kd", onDspOrPidChanged}
};

static const char* skipSpaces(const char* s) {
  while (s && (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')) s++;
  return s;
}

static bool parseNumberStrict(const char* s, float &out) {
  s = skipSpaces(s);
  if (!s || *s == '\0') return false;
  char* endptr = nullptr;
  float v = 0.0f;
  if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    unsigned long hex_v = strtoul(s, &endptr, 16);
    if (endptr == s) return false;
    v = (float)hex_v;
  } else {
    v = strtof(s, &endptr);
    if (endptr == s) return false;
  }
  if (isnan(v) || isinf(v)) return false;
  endptr = (char*)skipSpaces(endptr);
  if (*endptr != '\0') return false;
  out = v;
  return true;
}

static const CliParam* findCliParam(const char* name) {
  if (!name || *name == '\0') return nullptr;
  constexpr size_t count = sizeof(cliParams) / sizeof(cliParams[0]);
  for (size_t i = 0; i < count; i++) {
    if (strcmp(name, cliParams[i].name) == 0) return &cliParams[i];
  }
  return nullptr;
}

static void printCliParam(const CliParam* p) {
  if (!p) return;
  Serial.print(p->name); Serial.print("=");
  if (strcmp(p->name, "bat_v_max") == 0) {
    Serial.println(*(float*)p->ptr, 1);
    return;
  }
  switch (p->type) {
    case CLI_F32: Serial.println(*(float*)p->ptr, 4); break;
    case CLI_U8:  Serial.println((int)*(uint8_t*)p->ptr); break;
    case CLI_I8:  Serial.println((int)*(int8_t*)p->ptr); break;
    case CLI_U16: Serial.println((unsigned int)*(uint16_t*)p->ptr); break;
  }
}

static void writeCliParamValue(const CliParam* p, float val) {
  if (!p || isnan(val) || isinf(val)) return;
  if (strcmp(p->name, "bat_v_max") == 0) {
    *(float*)p->ptr = roundToTenthVolt(constrain(val, 7.0f, 25.0f));
    return;
  }
  if (strcmp(p->name, "flt_esc_limit") == 0) {
    *(float*)p->ptr = constrain(val, 1.0f, 100.0f);
    return;
  }
  switch (p->type) {
    case CLI_F32: *(float*)p->ptr = val; break;
    case CLI_U8:  *(uint8_t*)p->ptr = (uint8_t)constrain((int)lroundf(val), p->min_v, p->max_v); break;
    case CLI_I8:  *(int8_t*)p->ptr = (int8_t)constrain((int)lroundf(val), p->min_v, p->max_v); break;
    case CLI_U16: *(uint16_t*)p->ptr = (uint16_t)constrain((long)llroundf(val), (long)p->min_v, (long)p->max_v); break;
  }
}

static void applySideEffectsForKey(const char* key) {
  constexpr size_t count = sizeof(cliSideEffects) / sizeof(cliSideEffects[0]);
  for (size_t i = 0; i < count; i++) {
    if (strcmp(key, cliSideEffects[i].key) == 0) { cliSideEffects[i].fn(); return; }
  }
}

static const char* rtxChipName(uint8_t chip_id) {
  switch (chip_id) {
    case RTX_CHIP_SX1262: return "SX1262";
    case RTX_CHIP_SX1268: return "SX1268";
    case RTX_CHIP_LLCC68: return "LLCC68";
    default: return "AUTO";
  }
}

static const char* imuChipName(uint8_t t) {
  switch (t) {
    case 1: return "MPU6050"; case 2: return "MPU6500"; case 3: return "MPU9250";
    case 4: return "ICM20602"; case 5: return "BMI160"; case 6: return "QMI8658";
    default: return "NONE";
  }
}

static const char* magChipName(uint8_t t) {
  switch (t) {
    case 1: return "HMC5883L"; case 2: return "QMC5883L"; case 3: return "AK8963";
    case 4: return "IST8310"; case 5: return "HSCDTD008A"; case 6: return "AK09911";
    default: return "NONE";
  }
}

static const char* baroChipName(uint8_t t) {
  switch (t) {
    case 1: return "BMP280"; case 2: return "BME280"; case 3: return "MS5611";
    case 4: return "BMP180"; case 5: return "BMP388"; case 6: return "BMP390"; case 7: return "DPS310";
    default: return "NONE";
  }
}

static const char* lidarChipName(uint8_t t) {
  switch (t) {
    case 1: return "VL53L0X"; case 2: return "VL53L1X"; case 3: return "VL53L3CX"; case 4: return "VL53L4CX";
    default: return "NONE";
  }
}

static void actHelp(const char*) {
  Serial.println("OK: set, get, dump, save, default, cal_imu, cal_gyro, cal_acc, cal_mag, cal_bar, cal_lid, cal_bat <v>, cal_curr [a], auto_cal, auto_stop, mon, test_imu, test_mag, test_bar, test_lid, test_bat, test_rtx, test_vtx, test_mot, test_pid");
}

static void actSet(const char* args) {
  args = skipSpaces(args);
  char key[48] = {0};
  size_t k_len = 0;
  while (args[k_len] != '\0' && args[k_len] != ' ' && args[k_len] != '\t' && args[k_len] != '=') {
    if (k_len < sizeof(key) - 1) key[k_len] = args[k_len];
    k_len++;
  }
  key[k_len] = '\0';
  if (k_len == 0) { Serial.println("ERR: set <k> <v>"); return; }

  const char* val_start = skipSpaces(args + k_len);
  if (*val_start == '=') val_start = skipSpaces(val_start + 1);

  float val = 0.0f;
  if (!parseNumberStrict(val_start, val)) {
    Serial.println("ERR: set <k> <v>"); return;
  }
  const CliParam* p = findCliParam(key);
  if (!p) { Serial.println("ERR: param"); return; }
  writeCliParamValue(p, val);
  applySideEffectsForKey(key);
  printCliParam(p);
}

static void actGet(const char* args) {
  char clean_name[48] = {0};
  const char* s = skipSpaces(args);
  size_t idx = 0;
  while (s[idx] != '\0' && s[idx] != ' ' && s[idx] != '\t' && s[idx] != '\r' && s[idx] != '\n') {
    if (idx < sizeof(clean_name) - 1) clean_name[idx] = s[idx];
    idx++;
  }
  clean_name[idx] = '\0';
  const CliParam* p = findCliParam(clean_name);
  if (!p) { Serial.println("ERR: param"); return; }
  printCliParam(p);
}

static void actDump(const char*) {
  constexpr size_t count = sizeof(cliParams) / sizeof(cliParams[0]);
  for (size_t i = 0; i < count; i++) printCliParam(&cliParams[i]);
}

static void actSave(const char*) {
  if (CliAutoCal::isFlightActive()) { Serial.println("ERR: busy"); return; }
  saveSettings();
  Serial.println("OK: save");
}

static void actDefault(const char*) {
  if (CliAutoCal::isFlightActive()) { Serial.println("ERR: busy"); return; }
  resetToFactoryDefaults();
  vtxInit();
  if (currentConfig.rtx_en) applyRtxConfiguration(false);
  Serial.println("OK: default");
}

static void actCalImu(const char*)  { SensorDriver::calibrateIMU(); }
static void actCalGyro(const char*) { (void)SensorDriver::calibrateGyro(false); }
static void actCalAcc(const char*)  { (void)SensorDriver::calibrateAcc(false); }
static void actCalMag(const char*)  { startMagCalibrationNonBlocking(); }
static void actCalBar(const char*)  { SensorDriver::calibrateBaro(); }
static void actCalLid(const char*)  { SensorDriver::calibrateLidar(); }

static void actCalBat(const char* args) {
  float v = 0.0f;
  if (!parseNumberStrict(args, v)) { Serial.println("ERR: cal_bat <v>"); return; }
  SensorDriver::calibrateBatVoltage(v);
}

static void actCalCurr(const char* args) {
  const char* s = skipSpaces(args);
  if (!s || *s == '\0') { SensorDriver::calibrateCurrent(false, 0.0f); return; }
  float a = 0.0f;
  if (!parseNumberStrict(s, a)) { Serial.println("ERR: cal_curr [a]"); return; }
  SensorDriver::calibrateCurrent(true, a);
}

static void actAutoCal(const char*) { CliAutoCal::start(); Serial.println("OK: auto_cal"); }
static void actAutoStop(const char*) { CliAutoCal::stop(CliAutoCal::REASON_USER_CANCEL, FC_ERR_NONE); Serial.println("OK: auto_stop"); }

static void actMon(const char*) {
  mon_stream_active = true;
  mon_last_send_ms = 0;
  Serial.println("OK: mon");
}

static void actTestImu(const char*) {
  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float r = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * FC_RAD_TO_DEG;
  float p = asinf(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * FC_RAD_TO_DEG;
  float y = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * FC_RAD_TO_DEG;
  Serial.print("TEST_IMU: found="); Serial.print(SensorDriver::activeSensors.imu_found ? 1 : 0);
  Serial.print(" chip="); Serial.print(imuChipName((uint8_t)SensorDriver::activeSensors.imu_type));
  Serial.print(" err="); Serial.print((int)getActiveStatusErrorCode());
  Serial.print(" gyro="); Serial.print(last_cal_gx_dps, 2); Serial.print(","); Serial.print(last_cal_gy_dps, 2); Serial.print(","); Serial.print(last_cal_gz_dps, 2);
  Serial.print(" acc="); Serial.print(last_cal_ax_g, 3); Serial.print(","); Serial.print(last_cal_ay_g, 3); Serial.print(","); Serial.print(last_cal_az_g, 3);
  Serial.print(" rpy="); Serial.print(r, 1); Serial.print(","); Serial.print(p, 1); Serial.print(","); Serial.print(y, 1);
  Serial.print(" temp="); Serial.println(current_chip_temp, 1);
}

static void actTestMag(const char*) {
  float mx_u = 0.0f, my_u = 0.0f, mz_u = 0.0f;
  bool ok = SensorDriver::readMagUnified(mx_u, my_u, mz_u);
  Serial.print("TEST_MAG: found="); Serial.print(SensorDriver::activeSensors.mag_found ? 1 : 0);
  Serial.print(" chip="); Serial.print(magChipName((uint8_t)SensorDriver::activeSensors.mag_type));
  Serial.print(" ok="); Serial.print(ok ? 1 : 0);
  Serial.print(" xyz="); Serial.print(mx_u, 1); Serial.print(","); Serial.print(my_u, 1); Serial.print(","); Serial.print(mz_u, 1);
  Serial.print(" hdg="); Serial.println(compass_heading_deg, 1);
}

static void actTestBar(const char*) {
  float raw_alt = 0.0f;
  bool ok = SensorDriver::readRawBaroAltitudeBlocking(raw_alt);
  Serial.print("TEST_BAR: found="); Serial.print(SensorDriver::activeSensors.baro_found ? 1 : 0);
  Serial.print(" chip="); Serial.print(baroChipName((uint8_t)SensorDriver::activeSensors.baro_type));
  Serial.print(" ok="); Serial.print(ok ? 1 : 0);
  Serial.print(" raw_alt="); Serial.print(raw_alt, 2);
  Serial.print(" alt="); Serial.println(baro_alt_m, 2);
}

static void actTestLid(const char*) {
  float slant_m = 0.0f;
  bool ok = SensorDriver::readRawLidarSlantMeters(slant_m);
  Serial.print("TEST_LID: found="); Serial.print(SensorDriver::activeSensors.lidar_found ? 1 : 0);
  Serial.print(" chip="); Serial.print(lidarChipName((uint8_t)SensorDriver::activeSensors.lidar_type));
  Serial.print(" ok="); Serial.print(ok ? 1 : 0);
  Serial.print(" slant="); Serial.print(slant_m, 3);
  Serial.print(" lid="); Serial.println(dbg_lidar_m, 3);
}

static void actTestBat(const char*) {
  Serial.print("TEST_BAT: valid="); Serial.print(batt_valid ? 1 : 0);
  Serial.print(" v="); Serial.print(v_batt, 2);
  Serial.print(" i="); Serial.print(current_a, 2);
  Serial.print(" w="); Serial.print(power_w, 1);
  Serial.print(" wh="); Serial.println(consumed_wh, 3);
}

static void actTestRtx(const char*) {
  Serial.print("TEST_RTX: en="); Serial.print(currentConfig.rtx_en ? 1 : 0);
  Serial.print(" chip="); Serial.print(rtxChipName(rtx_active_chip_id));
  Serial.print(" chan="); Serial.print((int)active_rtx_chan);
  Serial.print(" bak="); Serial.print(is_on_backup ? 1 : 0);
  Serial.print(" lq="); Serial.print((int)current_link_quality);
  Serial.print(" tx_flight="); Serial.print(rtx_tx_in_flight ? 1 : 0);
  Serial.print(" scan="); Serial.println(rtx_scan_active ? 1 : 0);
}

static void actTestVtx(const char*) {
  Serial.print("TEST_VTX: en="); Serial.print(currentConfig.vtx_en ? 1 : 0);
  Serial.print(" link="); Serial.print(vtx_link_ok ? 1 : 0);
  Serial.print(" proto="); Serial.print((int)vtx_detected_proto);
  Serial.print(" mhz="); Serial.print(currentConfig.vtx_mhz);
  Serial.print(" mw="); Serial.print(currentConfig.vtx_mw);
  Serial.print(" act_mw="); Serial.println(vtx_actual_mw);
}

static void startBenchTestMode(BenchTestMode mode, const char* tag) {
  if (is_armed || CliAutoCal::isFlightActive()) { Serial.println("ERR: armed"); return; }
  if (!SensorDriver::activeSensors.imu_found || err_imu_fail) { Serial.println("ERR: imu"); return; }
  rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
  resetMotorSlewState();
  bench_test_mode = mode;
  bench_test_start_ms = millis();
  Serial.print("OK: "); Serial.println(tag);
}

void stopBenchTest() {
  if (bench_test_mode == BENCH_TEST_NONE) return;
  bench_test_mode = BENCH_TEST_NONE;
  resetMotorSlewState();
  setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);
}

void stepBenchTestNonBlocking(uint32_t now_ms, float gx_rad, float gy_rad, float gz_rad,
                              float &m1, float &m2, float &m3, float &m4) {
  if (bench_test_mode == BENCH_TEST_NONE) return;
  if (is_armed || CliAutoCal::isFlightActive() || (now_ms - bench_test_start_ms) >= BENCH_TEST_DURATION_MS) {
    stopBenchTest();
    m1 = m2 = m3 = m4 = OUTPUT_MIN;
    return;
  }
  if (bench_test_mode == BENCH_TEST_MOTORS) {
    m1 = m2 = m3 = m4 = BENCH_TEST_THROTTLE;
    return;
  }
  float r_cmd = rollRatePID.update(0.0f - gx_rad, gx_rad, DT, false);
  float p_cmd = pitchRatePID.update(0.0f - gy_rad, gy_rad, DT, false);
  float y_cmd = yawRatePID.update(0.0f - gz_rad, gz_rad, DT, false);
  m1 = constrain(BENCH_TEST_THROTTLE + p_cmd + r_cmd - y_cmd, OUTPUT_MIN, 300.0f);
  m2 = constrain(BENCH_TEST_THROTTLE + p_cmd - r_cmd + y_cmd, OUTPUT_MIN, 300.0f);
  m3 = constrain(BENCH_TEST_THROTTLE - p_cmd - r_cmd - y_cmd, OUTPUT_MIN, 300.0f);
  m4 = constrain(BENCH_TEST_THROTTLE - p_cmd + r_cmd + y_cmd, OUTPUT_MIN, 300.0f);
}

static void actTestMot(const char*) { startBenchTestMode(BENCH_TEST_MOTORS, "test_mot"); }
static void actTestPid(const char*) { startBenchTestMode(BENCH_TEST_PID, "test_pid"); }

static const CliActionEntry cliActions[] = {
  {"help", false, false, actHelp},
  {"set", true, false, actSet},
  {"get", true, false, actGet},
  {"dump", false, false, actDump},
  {"save", false, true, actSave},
  {"default", false, true, actDefault},
  {"cal_imu", false, true, actCalImu},
  {"cal_gyro", false, true, actCalGyro},
  {"cal_acc", false, true, actCalAcc},
  {"cal_mag", false, true, actCalMag},
  {"cal_bar", false, true, actCalBar},
  {"cal_lid", false, true, actCalLid},
  {"cal_bat", true, true, actCalBat},
  {"cal_curr", true, true, actCalCurr},
  {"auto_cal", false, true, actAutoCal},
  {"auto_stop", false, false, actAutoStop},
  {"mon", false, false, actMon},
  {"test_imu", false, false, actTestImu},
  {"test_mag", false, false, actTestMag},
  {"test_bar", false, false, actTestBar},
  {"test_lid", false, false, actTestLid},
  {"test_bat", false, false, actTestBat},
  {"test_rtx", false, false, actTestRtx},
  {"test_vtx", false, false, actTestVtx},
  {"test_mot", false, true, actTestMot},
  {"test_pid", false, true, actTestPid}
};

static bool matchActionPrefix(const char* s, const char* name, bool has_args, const char* &args_out) {
  size_t len = strlen(name);
  if (strncmp(s, name, len) != 0) return false;
  char next = s[len];
  if (next == '\0') { args_out = ""; return true; }
  if (has_args && (next == ' ' || next == '\t' || next == '=')) {
    args_out = s + len;
    return true;
  }
  return false;
}

void executeCommand(const char* cmd) {
  const char* s = skipSpaces(cmd);
  if (!s || *s == '\0') return;
  mon_stream_active = false;

  constexpr size_t count = sizeof(cliActions) / sizeof(cliActions[0]);
  for (size_t i = 0; i < count; i++) {
    const char* args = nullptr;
    if (matchActionPrefix(s, cliActions[i].name, cliActions[i].has_args, args)) {
      if (cliActions[i].requires_disarm && is_armed) {
        Serial.println("ERR: armed");
        return;
      }
      cliActions[i].fn(args);
      return;
    }
  }
  Serial.println("ERR: cmd");
}

void parseCli() {
  static char line_buf[128];
  static size_t line_idx = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (line_idx > 0) {
        line_buf[line_idx] = '\0';
        line_idx = 0;
        executeCommand(line_buf);
      }
    } else if (c == '\b' || (uint8_t)c == 127) {
      if (line_idx > 0) line_idx--;
    } else if ((uint8_t)c >= 32 && (uint8_t)c < 127) {
      if (line_idx < sizeof(line_buf) - 1) {
        line_buf[line_idx++] = c;
      } else {
        line_idx = 0;
        Serial.println("ERR: line_len");
      }
    }
  }
}

// ==============================================================================
// 11. РАСЧЁТ КУРСОВ, ТАЙМЕРЫ PWM/ONESHOT125 И ПОЛЁТНЫЙ ЦИКЛ
// ==============================================================================
float calculateSuperExpoRate(float in, float r, float s, float e) {
  float x = constrain(in, -1.0f, 1.0f);
  float exp_term = x * (e * powf(fabsf(x), 3.0f) + (1.0f - e));
  float denom = fmaxf(1.0f - (fabsf(exp_term) * s), 0.1f);
  return (r * exp_term) / denom;
}

void setupHardwareTimers() {
  uint32_t timer_clk = tim2.getTimerClkFreq();
  bool is_oneshot = (currentConfig.esc_prot == 1);
  uint32_t target_tick_hz = is_oneshot ? TIM2_ONESHOT_HZ : TIM2_TICK_HZ;
  uint32_t prescaler = (timer_clk >= target_tick_hz && target_tick_hz > 0) ? (timer_clk / target_tick_hz) : 1;
  if (prescaler < 1) prescaler = 1;
  uint32_t period_us = is_oneshot ? 125 : 2500;
  tim2.pause();
  tim2.setPrescaleFactor(prescaler);
  tim2.setOverflow(period_us, MICROSEC_FORMAT);
  tim2.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, MOTOR1_PIN);
  tim2.setMode(2, TIMER_OUTPUT_COMPARE_PWM1, MOTOR2_PIN);
  tim2.setMode(3, TIMER_OUTPUT_COMPARE_PWM1, MOTOR3_PIN);
  tim2.setMode(4, TIMER_OUTPUT_COMPARE_PWM1, MOTOR4_PIN);
  uint32_t pulse_min = is_oneshot ? 3000 : 1000;
  tim2.setCaptureCompare(1, pulse_min, MICROSEC_COMPARE_FORMAT);
  tim2.setCaptureCompare(2, pulse_min, MICROSEC_COMPARE_FORMAT);
  tim2.setCaptureCompare(3, pulse_min, MICROSEC_COMPARE_FORMAT);
  tim2.setCaptureCompare(4, pulse_min, MICROSEC_COMPARE_FORMAT);
  tim2.resume();
}

void resetMotorSlewState() {
  prev_m1_out = OUTPUT_MIN;
  prev_m2_out = OUTPUT_MIN;
  prev_m3_out = OUTPUT_MIN;
  prev_m4_out = OUTPUT_MIN;
}

static inline float applyMotorSlewRate(float target, float prev) {
  float max_delta = getEffectiveEscDeltaLimit();
  float delta = constrain(target - prev, -max_delta, max_delta);
  return constrain(prev + delta, OUTPUT_MIN, OUTPUT_MAX);
}

void setMotorsPower(float m1, float m2, float m3, float m4, float max_cap) {
  if (isnan(m1) || isnan(m2) || isnan(m3) || isnan(m4) ||
      isinf(m1) || isinf(m2) || isinf(m3) || isinf(m4)) {
    setFcErrorRam(FC_ERR_NAN_INF_MATH);
    m1 = m2 = m3 = m4 = OUTPUT_MIN;
    is_armed = false;
  }
  float cap = constrain(max_cap, OUTPUT_MIN, OUTPUT_MAX);
  bool allow_slew = (is_armed || CliAutoCal::isFlightActive() || bench_test_mode != BENCH_TEST_NONE) && !is_failsafe;
  float c1 = constrain(m1, OUTPUT_MIN, cap);
  float c2 = constrain(m2, OUTPUT_MIN, cap);
  float c3 = constrain(m3, OUTPUT_MIN, cap);
  float c4 = constrain(m4, OUTPUT_MIN, cap);

  if (allow_slew) {
    c1 = applyMotorSlewRate(c1, prev_m1_out);
    c2 = applyMotorSlewRate(c2, prev_m2_out);
    c3 = applyMotorSlewRate(c3, prev_m3_out);
    c4 = applyMotorSlewRate(c4, prev_m4_out);
  } else {
    c1 = OUTPUT_MIN; c2 = OUTPUT_MIN; c3 = OUTPUT_MIN; c4 = OUTPUT_MIN;
  }
  prev_m1_out = c1; prev_m2_out = c2; prev_m3_out = c3; prev_m4_out = c4;

  bool is_oneshot = (currentConfig.esc_prot == 1);
  uint32_t pulse_min = is_oneshot ? 3000 : 1000;
  uint32_t pulse_span = is_oneshot ? 3000 : 1000;
  tim2.setCaptureCompare(1, pulse_min + (uint32_t)((c1 / OUTPUT_MAX) * (float)pulse_span), MICROSEC_COMPARE_FORMAT);
  tim2.setCaptureCompare(2, pulse_min + (uint32_t)((c2 / OUTPUT_MAX) * (float)pulse_span), MICROSEC_COMPARE_FORMAT);
  tim2.setCaptureCompare(3, pulse_min + (uint32_t)((c3 / OUTPUT_MAX) * (float)pulse_span), MICROSEC_COMPARE_FORMAT);
  tim2.setCaptureCompare(4, pulse_min + (uint32_t)((c4 / OUTPUT_MAX) * (float)pulse_span), MICROSEC_COMPARE_FORMAT);
}

void flightControlTask() {
  uint32_t now_ms = millis();
  if (!SensorDriver::readMPU6050(mpu_dma_buffer)) {
    err_imu_fail = true;
    setFcErrorRam(FC_ERR_IMU_FAIL);
    if (is_armed || CliAutoCal::isFlightActive()) {
      is_armed = false;
      CliAutoCal::stop(CliAutoCal::REASON_SENSOR_FAIL, FC_ERR_IMU_FAIL);
      setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);
    }
    return;
  }

  float raw_ax = (float)((int16_t)(mpu_dma_buffer[0] << 8 | mpu_dma_buffer[1])) / SensorDriver::activeSensors.acc_scale;
  float raw_ay = (float)((int16_t)(mpu_dma_buffer[2] << 8 | mpu_dma_buffer[3])) / SensorDriver::activeSensors.acc_scale;
  float raw_az = (float)((int16_t)(mpu_dma_buffer[4] << 8 | mpu_dma_buffer[5])) / SensorDriver::activeSensors.acc_scale;
  float raw_temp = SensorDriver::extractTempFromImuBuf(mpu_dma_buffer);
  float raw_gx = (float)((int16_t)(mpu_dma_buffer[8] << 8 | mpu_dma_buffer[9])) / SensorDriver::activeSensors.gyro_scale;
  float raw_gy = (float)((int16_t)(mpu_dma_buffer[10] << 8 | mpu_dma_buffer[11])) / SensorDriver::activeSensors.gyro_scale;
  float raw_gz = (float)((int16_t)(mpu_dma_buffer[12] << 8 | mpu_dma_buffer[13])) / SensorDriver::activeSensors.gyro_scale;

  SensorDriver::rotateXYByYawAlign(raw_ax, raw_ay, currentConfig.imu_align);
  SensorDriver::rotateXYByYawAlign(raw_gx, raw_gy, currentConfig.imu_align);

  if (isnan(raw_ax) || isnan(raw_ay) || isnan(raw_az) ||
      isnan(raw_gx) || isnan(raw_gy) || isnan(raw_gz) ||
      isinf(raw_ax) || isinf(raw_ay) || isinf(raw_az) ||
      isinf(raw_gx) || isinf(raw_gy) || isinf(raw_gz)) {
    setFcErrorRam(FC_ERR_NAN_INF_MATH);
    is_armed = false;
    CliAutoCal::stop(CliAutoCal::REASON_SENSOR_FAIL, FC_ERR_NAN_INF_MATH);
    setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);
    return;
  }

  current_chip_temp = lpf_temp.update(raw_temp);
  float temp_delta = current_chip_temp - currentConfig.base_temperature;
  float gx_dps = raw_gx - (currentConfig.gyro_bias_x + currentConfig.temp_coeff_x * temp_delta);
  float gy_dps = raw_gy - (currentConfig.gyro_bias_y + currentConfig.temp_coeff_y * temp_delta);
  float gz_dps = raw_gz - (currentConfig.gyro_bias_z + currentConfig.temp_coeff_z * temp_delta);
  float ax_g = raw_ax - currentConfig.acc_bias_x;
  float ay_g = raw_ay - currentConfig.acc_bias_y;
  float az_g = raw_az - currentConfig.acc_bias_z;

  last_cal_gx_dps = gx_dps; last_cal_gy_dps = gy_dps; last_cal_gz_dps = gz_dps;
  last_cal_ax_g = ax_g; last_cal_ay_g = ay_g; last_cal_az_g = az_g;

  if (currentConfig.flt_lpf_en) {
    gx_dps = lpf_gx.update(gx_dps); gy_dps = lpf_gy.update(gy_dps); gz_dps = lpf_gz.update(gz_dps);
    ax_g = lpf_ax.update(ax_g); ay_g = lpf_ay.update(ay_g); az_g = lpf_az.update(az_g);
  }
  if (currentConfig.flt_not_en) {
    gx_dps = notch_gx.update(gx_dps); gy_dps = notch_gy.update(gy_dps); gz_dps = notch_gz.update(gz_dps);
  }

  float hp_x = lpf_xy_hp_x.update(gx_dps);
  float hp_y = lpf_xy_hp_y.update(gy_dps);
  vibration_index = sqrtf(hp_x * hp_x + hp_y * hp_y);

  float gx_rad = gx_dps * FC_DEG_TO_RAD;
  float gy_rad = gy_dps * FC_DEG_TO_RAD;
  float gz_rad = gz_dps * FC_DEG_TO_RAD;

  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float qDot0 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
  float qDot1 = 0.5f * (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
  float qDot2 = 0.5f * (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
  float qDot3 = 0.5f * (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

  float acc_norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  if (acc_norm > 0.001f) {
    float inv_acc = 1.0f / acc_norm;
    float ax_n = ax_g * inv_acc, ay_n = ay_g * inv_acc, az_n = az_g * inv_acc;
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float ex = (ay_n * vz - az_n * vy);
    float ey = (az_n * vx - ax_n * vz);
    float ez = (ax_n * vy - ay_n * vx);

    float beta = currentConfig.flt_amb_en ? currentConfig.flt_amb : DEFAULT_FLT_AMB_MIN;
    if (currentConfig.flt_amb_en) {
      float acc_err = fabsf(acc_norm - 1.0f);
      float dynamic_scale = 1.0f / (1.0f + (acc_err * 6.0f) + (vibration_index * 0.05f));
      beta = constrain(currentConfig.flt_amb * dynamic_scale, currentConfig.flt_amb_min, currentConfig.flt_amb);
    }
    qDot0 -= beta * (-q1 * ex - q2 * ey - q3 * ez);
    qDot1 -= beta * (q0 * ex + q2 * ez - q3 * ey);
    qDot2 -= beta * (q0 * ey - q1 * ez + q3 * ex);
    qDot3 -= beta * (q0 * ez + q1 * ey - q2 * ex);
  }

  orientation.q0 += qDot0 * DT;
  orientation.q1 += qDot1 * DT;
  orientation.q2 += qDot2 * DT;
  orientation.q3 += qDot3 * DT;
  orientation.normalize();

  q0 = orientation.q0; q1 = orientation.q1; q2 = orientation.q2; q3 = orientation.q3;
  float roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * FC_RAD_TO_DEG;
  float pitch_deg = asinf(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * FC_RAD_TO_DEG;

  CliAutoCal::onAttitudeAndMotion(ax_g, ay_g, az_g, roll_deg, pitch_deg, gx_dps, gy_dps);
  if (!is_armed && !CliAutoCal::isFlightActive() && bench_test_mode == BENCH_TEST_NONE) {
    setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);
    return;
  }

  float m1 = OUTPUT_MIN, m2 = OUTPUT_MIN, m3 = OUTPUT_MIN, m4 = OUTPUT_MIN;
  if (CliAutoCal::stepFlight(now_ms, gx_dps, gy_dps, gz_dps, gx_rad, gy_rad, gz_rad, roll_deg, pitch_deg, m1, m2, m3, m4)) {
    setMotorsPower(m1, m2, m3, m4);
    return;
  }

  if (bench_test_mode != BENCH_TEST_NONE) {
    stepBenchTestNonBlocking(now_ms, gx_rad, gy_rad, gz_rad, m1, m2, m3, m4);
    setMotorsPower(m1, m2, m3, m4, 300.0f);
    return;
  }

  bool rc_timed_out = (last_rc_packet_ms == 0) || ((now_ms - last_rc_packet_ms) > FAILSAFE_TIMEOUT_MS);
  if (currentConfig.fs_rtx_en && rc_timed_out) {
    err_rc_loss = true;
    is_failsafe = true;
    setFcErrorRam(FC_ERR_RC_LOSS);
  } else {
    err_rc_loss = false;
    is_failsafe = false;
  }

  if (is_failsafe) {
    float desc_vz = -fabsf(currentConfig.fs_vz_speed);
    float thr_cmd = constrain(base_hover_throttle + vzPID.update(desc_vz - baro_vz_mps, baro_vz_mps, DT, false), 0.0f, 700.0f);
    float r_cmd = rollRatePID.update(0.0f - gx_rad, gx_rad, DT, false);
    float p_cmd = pitchRatePID.update(0.0f - gy_rad, gy_rad, DT, false);
    float y_cmd = yawRatePID.update(0.0f - gz_rad, gz_rad, DT, false);
    m1 = thr_cmd + p_cmd + r_cmd - y_cmd;
    m2 = thr_cmd + p_cmd - r_cmd + y_cmd;
    m3 = thr_cmd - p_cmd - r_cmd - y_cmd;
    m4 = thr_cmd - p_cmd + r_cmd + y_cmd;
    setMotorsPower(m1, m2, m3, m4, 750.0f);
    return;
  }

  if (arm_ramp_factor < 1.0f) {
    arm_ramp_factor = fminf(1.0f, arm_ramp_factor + (DT / 0.25f));
  }

  float r_norm = (smooth_roll - STICK_CENTER) / STICK_CENTER;
  float p_norm = (smooth_pitch - STICK_CENTER) / STICK_CENTER;
  float y_norm = (smooth_yaw - STICK_CENTER) / STICK_CENTER;
  if (fabsf(smooth_roll - STICK_CENTER) < currentConfig.rc_deadband) r_norm = 0.0f;
  if (fabsf(smooth_pitch - STICK_CENTER) < currentConfig.rc_deadband) p_norm = 0.0f;
  if (fabsf(smooth_yaw - STICK_CENTER) < currentConfig.rc_deadband) y_norm = 0.0f;

  float target_gx_rad = calculateSuperExpoRate(r_norm, currentConfig.rc_rate, currentConfig.rc_super_expo, currentConfig.rc_stick_expo) * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD;
  float target_gy_rad = calculateSuperExpoRate(p_norm, currentConfig.rc_rate, currentConfig.rc_super_expo, currentConfig.rc_stick_expo) * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD;
  float target_gz_rad = calculateSuperExpoRate(y_norm, currentConfig.rc_yaw_rate, currentConfig.rc_yaw_super_expo, currentConfig.rc_yaw_expo) * MAX_RATE_DPS_PER_UNIT * FC_DEG_TO_RAD;

  if (currentConfig.fm_mode == 1 || currentConfig.fm_mode == 2) {
    float angle_weight = (currentConfig.fm_mode == 1) ? 1.0f : constrain(fmaxf(fabsf(r_norm), fabsf(p_norm)) / fmaxf(currentConfig.rc_horizon_transition, 0.05f), 0.0f, 1.0f);
    float desired_roll = constrain(-r_norm * currentConfig.fm_roll_max, -currentConfig.fm_roll_max, currentConfig.fm_roll_max);
    float desired_pitch = constrain(-p_norm * currentConfig.fm_pitch_max, -currentConfig.fm_pitch_max, currentConfig.fm_pitch_max);
    float angle_gx = (desired_roll - roll_deg) * 3.0f * FC_DEG_TO_RAD;
    float angle_gy = (desired_pitch - pitch_deg) * 3.0f * FC_DEG_TO_RAD;
    target_gx_rad = angle_weight * angle_gx + (1.0f - angle_weight) * target_gx_rad;
    target_gy_rad = angle_weight * angle_gy + (1.0f - angle_weight) * target_gy_rad;
  }

  float thr_base = constrain(smooth_throttle, OUTPUT_MIN, OUTPUT_MAX) * getBatteryCompensationFactor() * arm_ramp_factor;
  float r_cmd = rollRatePID.update(target_gx_rad - gx_rad, gx_rad, DT, false);
  float p_cmd = pitchRatePID.update(target_gy_rad - gy_rad, gy_rad, DT, false);
  float y_cmd = yawRatePID.update(target_gz_rad - gz_rad, gz_rad, DT, false);

  m1 = thr_base + p_cmd + r_cmd - y_cmd;
  m2 = thr_base + p_cmd - r_cmd + y_cmd;
  m3 = thr_base - p_cmd - r_cmd - y_cmd;
  m4 = thr_base - p_cmd + r_cmd + y_cmd;
  setMotorsPower(m1, m2, m3, m4);
}

// ==============================================================================
// 12. SETUP И MAIN LOOP
// ==============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(POWER_ON_SETTLE_MS);

  analogReadResolution(12);
  pinMode(BATTERY_PIN, INPUT_ANALOG);
  pinMode(CURRENT_PIN, INPUT_ANALOG);

  loadSettings();
  applyFilterConfig();
  setupHardwareTimers();
  setMotorsPower(OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN, OUTPUT_MIN);

  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  Wire.setClock(400000);

  SensorDriver::autoDetectAll();
  CliAutoCal::init();
  vtxInit();

  main_rtx_chan = constrain(currentConfig.rtx_pri_chan, 0, 99);
  active_rtx_chan = main_rtx_chan;
  is_on_backup = false;
  if (currentConfig.rtx_en) {
    rtx_init_hardware();
    applyRtxConfiguration(false);
  }
}

void loop() {
  static uint32_t last_loop_us = micros();
  uint32_t now_us = micros();
  if ((now_us - last_loop_us) >= LOOP_INTERVAL_US) {
    uint32_t jitter_us = (now_us - last_loop_us) - LOOP_INTERVAL_US;
    last_loop_us = now_us;
    if (jitter_us > LOOP_OVERRUN_MARGIN_US) {
      setFcErrorRam(FC_ERR_LOOP_OVERRUN);
    }

    uint32_t task_start_us = micros();
    handleMotorArmStateAndEepromError();
    flightControlTask();
    uint32_t elapsed_us = micros() - task_start_us;
    cpu_load_pct = ((float)elapsed_us / (float)LOOP_INTERVAL_US) * 100.0f;
    if (cpu_load_pct > cpu_max_load_pct) cpu_max_load_pct = cpu_load_pct;
  }

  uint32_t now_ms = millis();
  CliAutoCal::pollButton(now_ms);
  updateAuxSensorsNonBlocking(now_ms);
  stepMagCalibrationNonBlocking(now_ms);
  vtxStepNonBlocking(now_ms);
  serviceError202Event(now_ms);

  pollTelemetryTxState();
  stepChannelScanNonBlocking();
  readRx();
  if (send_telemetry_pending) startTelemetryNonBlocking();

  parseCli();
  stepMonitorStreamNonBlocking(now_ms);
}
