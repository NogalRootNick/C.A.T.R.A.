#include <Wire.h>

// Define I2C communication pins
#define I2C_SDA 21
#define I2C_SCL 22

// Define I2C address of the motor controller
#define I2C_ADDR 0x34

// Define register addresses for motor controller features
#define ADC_BAT_ADDR 0x00                 // Battery ADC address
#define MOTOR_TYPE_ADDR 0x14              // Motor type setting address
#define MOTOR_ENCODER_POLARITY_ADDR 0x15  // Motor encoder polarity setting address
#define MOTOR_FIXED_PWM_ADDR 0x1F         // Fixed PWM control address
#define MOTOR_FIXED_SPEED_ADDR 0x33       // Fixed speed control address
#define MOTOR_ENCODER_TOTAL_ADDR 0x3C     // Motor encoder total count address

#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

/**
 * @brief Writes an array of data to a specific register on the I2C device.
 *
 * @param reg The register address to write to.
 * @param val A pointer to the array of uint8_t values to write.
 * @param len The number of bytes to write from the array.
 * @return True if the transmission was successful, false otherwise.
 */
bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) 
{
  unsigned int i;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for(i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  if ( Wire.endTransmission() != 0 ) {
    return false;
  }
  return true;
}

uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
uint8_t MotorEncoderPolarity = 0;

uint8_t car_forward[4] = {0, 233, 0, 23};
uint8_t car_turnleft[4] = {0, 20, 0, 20};
uint8_t car_stop[4] = {0, 0, 0, 0};
uint8_t car_retreat[4] = {0, 23, 0, 233};

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(200);
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);
}

void loop() {
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
  delay(5000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  delay(5000);
}