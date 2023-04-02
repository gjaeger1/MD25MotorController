#pragma once

#include <stdint.h>
#include <memory>



#define CMD_REG             0x10u                        // Values of 0 eing sent using write have to be cast as a byte to stop them being misinterperted as NULL
                                                              // This is a but with arduino 1
#define MD25ADDRESS         0x58u                              // Address of the MD25
#define SOFTWAREREG         0x0Du                              // Byte to read the software version
#define SPEED1              0x00u                        // Byte to send speed to first motor
#define SPEED2              0x01u                              // Byte to send speed to second motor
#define ENCODERONE          0x02u                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06u                              // Byte to read motor encoder 2
#define VOLTREAD            0x0Au                              // Byte to read battery volts
#define MOTORCURRENTONE     0x0Bu                              // Byte to read motor current 1
#define MOTORCURRENTTWO     0x0Cu                              // Byte to read motor current 2
#define ACCELERATION_REG    0x0Eu                              // Byte to set acceleration register
#define MODE_REG            0x0Fu                              // Byte to set mode register
#define RESETENCODERS       0x20u


#define return_on_error(a) if(a<0) return -1;
#define return_check_error(a) if(a<0){return -1;}else{return 0;}



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

    int begin()
    {
      return_on_error(this->setMode(this->mode))
      return_on_error(this->setAccelerationRate(this->acceleration_limit))
      return_on_error(this->setSpeed(0))
      return_on_error(this->setTurning(0))
      return_on_error(this->resetEncoders())
      return 0;
    }

    int enableTimeout() const;
    int disableTimeout() const;

    int enableSpeedRegulation() const;
    int disableSpeedRegulation() const;

    int resetEncoders() const
    {
      uint8_t data[] = {CMD_REG, RESETENCODERS};
      return_check_error(this->write_registers(data, 2))
    }

    int setAccelerationRate(const uint8_t accel)
    {
      uint8_t data[2] = {ACCELERATION_REG, accel}; // no turning
      return_on_error(this->write_registers(data, 2));
      this->acceleration_limit = accel;
      return 0;
    }

    int setMode(const uint8_t m)
    {
      uint8_t data[2] = {MODE_REG, m}; // no turning
      return_on_error(this->write_registers(data, 2));
      this->mode = m;
      return 0;
    }

    int setSpeed(const uint8_t speed) const
    {
      switch(this->mode)
      {
        case(Modes::INDIVIDUAL_128_STOP):
        case(Modes::INDIVIDUAL_0_STOP):
        {
          uint8_t data1[2] = {SPEED1, speed};
          this->write_registers(data1, 2);
          uint8_t data2[2] = {SPEED2, speed};
          this->write_registers(data2, 2);
          break;
        }
        case(Modes::COMBINED_128_STOP):
        case(Modes::COMBINED_0_STOP):
        {
          uint8_t data[2] = {SPEED1, speed}; // set for both motors
          this->write_registers(data, 2);
          break;
        }
        default:
        {
        }
      }

      return 0;
    }

    int setSpeedMotorOne(const uint8_t speed) const
    {
      uint8_t data[2] = {SPEED1, speed}; // set for both motors
      return_on_error(this->write_registers(data, 2));
      return 0;
    }

    int setSpeedMotorTwo(const uint8_t speed) const
    {
      uint8_t data[2] = {SPEED2, speed}; // set for both motors
      return_on_error(this->write_registers(data, 2));
      return 0;
    }

    int stop() const
    {
      switch(this->mode)
      {
        case Modes::INDIVIDUAL_128_STOP:
        {
          uint8_t data1[2] = {SPEED1, 128}; // Sends a value of 128 to motor 1 this value stops the motor
          return_on_error(this->write_registers(data1, 2));

          uint8_t data2[2] = {SPEED2, 128}; // Sends a value of 128 to motor 2 this value stops the motor
          return_on_error(this->write_registers(data2, 2));
          break;
        }
        case Modes::INDIVIDUAL_0_STOP:
        {
          uint8_t data1[2] = {SPEED1, 0}; // Sends a value of 0 to motor 1 this value stops the motor
          return_on_error(this->write_registers(data1, 2));
          uint8_t data2[2] = {SPEED2, 0}; // Sends a value of 0 to motor 2 this value stops the motor
          return_on_error(this->write_registers(data2, 2));
          break;
        }
        case Modes::COMBINED_128_STOP:
        {
          uint8_t data1[2] = {SPEED1, 128}; // both motors stop
          return_on_error(this->write_registers(data1, 2));
          uint8_t data2[2] = {SPEED2, 0}; // no turning
          return_on_error(this->write_registers(data2, 2));
          break;
        }
        case Modes::COMBINED_0_STOP:
        {
          uint8_t data1[2] = {SPEED1, 0}; // both motors stop
          return_on_error(this->write_registers(data1, 2));
          uint8_t data2[2] = {SPEED2, 0}; // no turning
          return_on_error(this->write_registers(data2, 2));
          break;
        }
        default:
        {
        }
      }

      return 0;
    }


    int setTurning(const uint8_t turning) const
    {
      switch(this->mode)
      {
        case(Modes::INDIVIDUAL_128_STOP):
        case(Modes::INDIVIDUAL_0_STOP):
        {
          break;
        }
        case(Modes::COMBINED_128_STOP):
        case(Modes::COMBINED_0_STOP):
        {
          uint8_t data[2] = {SPEED2, turning}; // set for both motors
          this->write_registers(data, 2);
          break;
        }
        default:
        {
        }
      }

      return 0;
    }

    float getBatteryVolts() const
    {
      uint8_t volts;
      this->read_registers(VOLTREAD, &volts, 1);
      return static_cast<float>(*reinterpret_cast<int8_t*>(&volts)/10.0f);
    }


    float getMotorOneCurrent() const
    {
      uint8_t current;
      this->read_registers(MOTORCURRENTONE, &current, 1);
      return static_cast<float>(*reinterpret_cast<int8_t*>(&current)/10.0f);
    }

    float getMotorTwoCurrent() const
    {
      uint8_t current;
      this->read_registers(MOTORCURRENTTWO, &current, 1);
      return static_cast<float>(*reinterpret_cast<int8_t*>(&current)/10.0f);
    }

    uint8_t getSoftwareRevisionNumber() const
    { // Function that gets the software version

      uint8_t software;
      this->read_registers(SOFTWAREREG, &software, 1);
      return(software);
    }

    long getEncoderOne() const
    {
      uint8_t data[4];
      this->read_registers(ENCODERONE, data, 4);

      long pos = data[0];                                 // First byte for encoder 1, HH.
      pos <<= 8;
      pos += data[1];                                     // Second byte for encoder 1, HL
      pos <<= 8;
      pos += data[2];                                     // Third byte for encoder 1, LH
      pos <<= 8;
      pos += data[3];                                     // Fourth byte for encoder 1, LL

      return(pos);
    }

    long getEncoderTwo() const
    {
      uint8_t data[4];
      this->read_registers(ENCODERTWO, data, 4);

      long pos = data[0];                                 // First byte for encoder 1, HH.
      pos <<= 8;
      pos += data[1];                                     // Second byte for encoder 1, HL
      pos <<= 8;
      pos += data[2];                                     // Third byte for encoder 1, LH
      pos <<= 8;
      pos += data[3];                                     // Fourth byte for encoder 1, LL

      return(pos);
    }

  private:
    int write_registers(const uint8_t* data, const std::size_t num) const
    {
      if(this->i2c_dev->send_bytes_blocking(MD25ADDRESS, data, num, false) < 0)
        return -1;

      return 0;
    }

    int read_registers(const uint8_t register_address, uint8_t* data, const std::size_t num) const
    {
      if(this->i2c_dev->receive_bytes_blocking(MD25ADDRESS, &register_address, sizeof(uint8_t), true) < 0)
        return -1;

      if(this->i2c_dev->receive_bytes_blocking(MD25ADDRESS, data, (uint8_t) num, false) < 0)
        return -1;

      return 0;
    }

};
