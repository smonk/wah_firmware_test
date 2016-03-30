#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"
#include "wm8731.h"

void codec_init(SPI_HandleTypeDef* hspi){
     uint8_t pData[2] = { 0x10, 0x0f}; 
    uint16_t temp = 0;

/*POWER UP SEQUENCE
        
        Switch on power supplies. By default the WM8731 is in Standby Mode, the DAC is
        digitally muted and the Audio Interface and Outputs are all OFF.
        
        Set all required bits in the Power Down register (0Ch) to ‘0’; EXCEPT the OUTPD bit,
        this should be set to ‘1’ (Default).
        
        Set required values in all other
        registers except 12h (Active).
        
        Set the ‘Active’ bit in register 12h.
        
        The last write of the sequence should be se
        tting OUTPD to ‘0’ (active) in register
        0Ch, enabling the DAC signal path, free
        of any significant power-up noise*/

        //Reset teh device
    // temp = 0x1E << 8;
    // temp |= 0x00;

    // pData[0] = temp >> 8;
    // pData[1] = temp;

    // HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    // HAL_SPI_Transmit(& hspi, pData, 2, 100);
    // HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    // HAL_Delay(25);


    //power down register

    int32_t spi_delay = 100;
    temp = 0x0c << 8;
    temp |= 0x00;

    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);


    // Left line in register
    temp = 0x00 << 8;
    temp |= 0x17; //0db gain

    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);
    // Right line in register
    temp = 0x02 << 8;
    temp |= 0x17; //0db gain

    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);

    //left hp and right hp out dont matter

    //analog audio path control
    temp = 0x08 << 8;
    temp |= 0x12; //Dac selected, mute mic input, no sidetone

    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);

    //digital audio path control
    temp = 0x0A << 8;
    temp |= 0x00; //mute disabled

    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);

    //Digital audio interface format
    temp = 0x0E << 8;
    temp |= 0x02; //16 bit input word, I2S format
    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);
  
    //Sampling control
    temp = 0x10 << 8;
    temp |= 0x00; //
    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);

    //Active control
    temp = 0x12 << 8;
    temp |= 0x01; //
    pData[0] = temp >> 8;
    pData[1] = temp;

    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 0 );
    HAL_SPI_Transmit( hspi, pData, 2, 100);
    HAL_GPIO_WritePin(WM8731_NSS_PORT, WM8731_NSS_PIN, 1 );
    HAL_Delay(spi_delay);
    
}