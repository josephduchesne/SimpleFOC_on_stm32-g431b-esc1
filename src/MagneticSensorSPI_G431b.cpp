
#include "MagneticSensorSPI_G431b.h"
#include "pinconfig.h"

SPI_HandleTypeDef spi3_handle;
SPI_TypeDef *spi3;

MagneticSensorSPI_G431b::MagneticSensorSPI_G431b(){
    cpr = pow(2, 14);
}

void MagneticSensorSPI_G431b::init(){

    // init pins
    pinMode(AS5047_SS, OUTPUT);
    digitalWrite(AS5047_SS, HIGH);


    // ini SPI
  // the following is mostly just a stripped down version of spi_com.c's spi_init

    spi3 = (SPI_TypeDef*)SPI3;
    spi3_handle.State = HAL_SPI_STATE_RESET;
    __HAL_RCC_SPI3_FORCE_RESET();

    //
        // 170Mhz / 16 ~= 10.625Mhz, which seems to work OK, despite being over 10
    spi3_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    // no SSEL defined in the HAL
    spi3_handle.Init.NSS = SPI_NSS_SOFT;

    spi3_handle.Instance               = spi3;
    spi3_handle.Init.Mode              = SPI_MODE_MASTER;
    spi3_handle.Init.Direction         = SPI_DIRECTION_2LINES;  // we're only actually using one

    // SPI mode 1
    spi3_handle.Init.CLKPhase          = SPI_PHASE_2EDGE;
    spi3_handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    
    // more settings
    spi3_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    spi3_handle.Init.CRCPolynomial     = 7;
    spi3_handle.Init.DataSize          = SPI_DATASIZE_16BIT;
    spi3_handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    spi3_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    spi3_handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

    // init pins
    pinmap_pinout(digitalPinToPinName(PIN_SPI_MISO), PinMap_SPI_MISO);
    pinmap_pinout(digitalPinToPinName(PIN_SPI_SCK), PinMap_SPI_SCLK);
    // from arduino spi_com.c
    uint32_t pull = (spi3_handle.Init.CLKPolarity == SPI_POLARITY_LOW) ? GPIO_PULLDOWN : GPIO_PULLUP;
    pin_PullConfig(get_GPIO_Port(STM_PORT(digitalPinToPinName(PIN_SPI_SCK))), STM_LL_GPIO_PIN(digitalPinToPinName(PIN_SPI_SCK)), pull);

    // enable SPI clock
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();

    // init HAL peripheral
    HAL_SPI_Init(&spi3_handle);
    __HAL_SPI_ENABLE(&spi3_handle);
    delay(100);
    //this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSPI_G431b::getSensorAngle(){
    return (getRawCount() / (float)cpr) * _2PI;
    //return (float)getRawCount();
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorSPI_G431b::getRawCount(){
    // force ready SPI
    if((spi3_handle.State>HAL_SPI_STATE_READY) && (spi3_handle.State<HAL_SPI_STATE_ERROR)){	// Handler BUSY in any mode (But not error or reset);
        spi3_handle.State=HAL_SPI_STATE_READY;	// Force ready state
    }

    // force reset SPI if it's busy :s
    // as per https://github.com/SimpleMethod/STM32-MAX31855/issues/1
    if((spi3_handle.Instance->SR & SPI_SR_BSY) || (spi3_handle.State!=HAL_SPI_STATE_READY)){	// If peripheral is actually busy or handler not ready
        spi3_handle.State=HAL_SPI_STATE_RESET;	// Force reset state (HAL_SPI_Init will fail if not in reset state)
    
        __HAL_RCC_SPI3_FORCE_RESET();		// Change to the SPI used in the project
        asm("nop\nnop\nnop\nnop");		// Wait few clocks just in case
        while(spi3_handle.Instance->SR & SPI_SR_BSY);	// Wait until Busy is gone
        __HAL_RCC_SPI3_RELEASE_RESET();		// Change to the SPI used in the project
        asm("nop\nnop\nnop\nnop");		// Wait few clocks just in case
        while(spi3_handle.Instance->SR & SPI_SR_BSY);	// Check again
    }		// Re-init SPI


    digitalWrite(AS5047_SS, LOW); // SS low
    uint16_t value = 0;
    HAL_SPI_Receive(&spi3_handle, (uint8_t *)&value, 2, 1000);
    digitalWrite(AS5047_SS, HIGH);  // SS high

    // trim the extra 2 bits off the 14 bit value
    value = (( value ) & (0xFFFF >> 2));  // Return the data, stripping the non data (e.g parity) bits


    return (int)value;
}
