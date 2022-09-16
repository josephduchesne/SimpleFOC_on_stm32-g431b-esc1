#ifndef MAGNETICSENSORSPI_G431B_H
#define MAGNETICSENSORSPI_G431B_H


#include "Arduino.h"
#include <SPI.h>
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

// Bringing online unidirectional SPI for AS5047 encoder
// 3.3V from the SWD pads
#define AS5047_SS PA15  // PWM pin, used as SPI CS pin, exposed on the pad 2nd from the VDC+ pad
#define PIN_SPI_MISO PC11  // CAN shdn isn't needed, SPI3_MISO. Exposed on TP2 pad by the STM32G micro silkscreen
#define PIN_SPI_SCK PC10   // the button, SPI3_SCK
#define PIN_SPI_MOSI NC  // No MOSI

class MagneticSensorSPI_G431b: public Sensor{
 public:
    /**
     *  MagneticSensorSPI class constructor
     */
    MagneticSensorSPI_G431b();

    /** sensor initialise pins */
    void init();

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;

    // returns the spi mode (phase/polarity of read/writes) i.e one of SPI_MODE0 | SPI_MODE1 | SPI_MODE2 | SPI_MODE3
    int spi_mode;
    
    /* returns the speed of the SPI clock signal */
    long clock_speed;
    

  private:
    float cpr; //!< Maximum range of the magnetic sensor

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();
};


#endif
