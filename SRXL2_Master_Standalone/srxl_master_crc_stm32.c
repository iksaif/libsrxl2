/*
 * Hardware CRC-16 (XMODEM) for STM32F7/H7 using HAL -- optional plug-in for
 * SRXL2_Master_Standalone.
 *
 * The STM32F7/H7 CRC unit supports a configurable polynomial and data size,
 * so it can compute CRC-16 XMODEM (poly 0x1021, seed 0) natively. This lets
 * an ArduPilot-class board reuse hardware it already has instead of the 512B
 * table.
 *
 * Usage:
 *   1. Compile srxl_master_standalone.c with -DSRXL_MASTER_CRC_EXTERN
 *   2. Add this file to your build
 *   3. Enable the CRC clock and init hcrc in your startup code before using
 *      the master:
 *
 *        __HAL_RCC_CRC_CLK_ENABLE();
 *        hcrc.Instance = CRC;
 *        hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_DISABLE;
 *        hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_DISABLE;
 *        hcrc.Init.GeneratingPolynomial    = 0x1021;
 *        hcrc.Init.CRCLength               = CRC_POLYLENGTH_16B;
 *        hcrc.Init.InitValue               = 0;
 *        hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
 *        hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
 *        hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;
 *        HAL_CRC_Init(&hcrc);
 *
 * Note: STM32F3 has a fixed CRC-32 unit -- this file does NOT work on F3.
 *       For F3/F4 or any target without a configurable polynomial, use the
 *       default table-based implementation instead.
 *
 * MIT License
 */

#include "srxl_master_standalone.h"

#if defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#else
#error "srxl_master_crc_stm32.c requires STM32F7 or STM32H7 (configurable CRC polynomial)"
#endif

extern CRC_HandleTypeDef hcrc;

uint16_t srxlMasterCrc16(const uint8_t *data, size_t len)
{
    return (uint16_t)HAL_CRC_Calculate(&hcrc, (uint32_t *)data, len);
}
