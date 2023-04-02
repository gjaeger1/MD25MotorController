#pragma once

template<typename I2CDeviceType>
class MD25MotorController
{

  public:
    enum Modes
    {
      INDIVIDUAL_128_STOP = 0,
      INDIVIDUAL_0_STOP = 1,
      COMBINED_128_STOP = 2,
      COMBINED_0_STOP = 3
    };

  private:

  uint8_t acceleration_limit = 10;  // highest possible acceleration
  uint8_t mode = Modes::COMBINED_0_STOP;                 // Speed1 sets speeds for both motors, Speed2 is the turn value
  std::shared_ptr<I2CDeviceType> i2c_dev;
  

  public:
    MD25MotorController(const std::shared_ptr<I2CDeviceType> dev) : i2c_dev(dev) {}

    int begin();

    int enableTimeout() const;
    int disableTimeout() const;

    int enableSpeedRegulation() const;
    int disableSpeedRegulation() const;

    int resetEncoders() const;

    int setAccelerationRate(const uint8_t accel);
    int setMode(const uint8_t m);
    int setSpeed(const uint8_t speed) const;
    int setSpeedMotorOne(const uint8_t speed) const;
    int setSpeedMotorTwo(const uint8_t speed) const;
    int stop() const;
    int setTurning(const uint8_t turning) const;

    float getBatteryVolts() const;
    float getMotorOneCurrent() const;
    float getMotorTwoCurrent() const;
    uint8_t getSoftwareRevisionNumber() const;

    long getEncoderOne() const;
    long getEncoderTwo() const;

  private:
    int write_registers(const uint8_t* data, const std::size_t num) const;
    int read_registers(const uint8_t register_address, uint8_t* data, const std::size_t num) const;
};
