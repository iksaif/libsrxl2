/*
 * SRXL2 Configuration for Tests
 *
 * Clone of programs/spm_srxl_config.h but uses tests/uart_adapter.h
 * for UART capture instead of fakeuart.
 *
 * MIT License
 */

#ifndef _SRXL_CONFIG_H_
#define _SRXL_CONFIG_H_

#include "uart_adapter.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations for callbacks (test provides implementations)
void userProvidedFillSrxlTelemetry(SrxlTelemetryData *pTelemetry);
void userProvidedReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafeData);
void userProvidedHandleVtxData(SrxlVtxData *pVtxData);

// Configuration
#define SRXL_NUM_OF_BUSES           1
#define SRXL_DEVICE_ID              0x10
#define SRXL_DEVICE_PRIORITY        20
#define SRXL_DEVICE_INFO            (SRXL_DEVINFO_NO_RF)
#define SRXL_SUPPORTED_BAUD_RATES   0
#define SRXL_CRC_OPTIMIZE_MODE      SRXL_CRC_OPTIMIZE_SPEED

// Interface functions
static inline void srxlChangeBaudRate(uint8_t uart, uint32_t baudRate)
{
    uartSetBaud(uart, baudRate);
}

static inline void srxlSendOnUart(uint8_t uart, uint8_t *pBuffer, uint8_t length)
{
    uartTransmit(uart, pBuffer, length);
}

static inline void srxlFillTelemetry(SrxlTelemetryData *pTelemetryData)
{
    userProvidedFillSrxlTelemetry(pTelemetryData);
}

static inline void srxlReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafe)
{
    userProvidedReceivedChannelData(pChannelData, isFailsafe);
}

static inline bool srxlOnBind(SrxlFullID device, SrxlBindData info)
{
    (void)device;
    (void)info;
    return true;
}

static inline void srxlOnVtx(SrxlVtxData *pVtxData)
{
    userProvidedHandleVtxData(pVtxData);
}

#ifdef SRXL_INCLUDE_FWD_PGM_CODE
static inline void srxlOnFwdPgm(uint8_t *pData, uint8_t dataLength)
{
    (void)pData;
    (void)dataLength;
}
#endif

static inline void srxlEnterCriticalSection(void) { }
static inline void srxlExitCriticalSection(void) { }

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _SRXL_CONFIG_H_
