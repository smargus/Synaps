/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <stdint.h>

#ifndef FIRMWARE_VERSION_H
#define FIRMWARE_VERSION_H
    
/**
 * The STM32 factory-programmed UUID memory.
 * Three values of 32 bits each starting at this address
 * Use like this: STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]
 */
    #define STM32_UUID ((uint32_t *)0x1FFF7A10)

    #define FIRMWARE_VERSION "0001"
    //const char str_version[] = "{{FIRMWARE-VERSION:0001}}";
    
#endif

/* [] END OF FILE */
