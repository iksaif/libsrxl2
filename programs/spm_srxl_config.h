/*
MIT License

SRXL2 Configuration for Simulation Programs

This configuration file is used by all simulation programs.
*/

#ifndef _SRXL_CONFIG_H_
#define _SRXL_CONFIG_H_

#include "uart_adapter.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations for user callbacks
void userProvidedFillSrxlTelemetry(SrxlTelemetryData* pTelemetry);
void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafeData);
void userProvidedHandleVtxData(SrxlVtxData* pVtxData);

//### USER CONFIGURATION ###

#define SRXL_NUM_OF_BUSES           1
#define SRXL_DEVICE_ID              0x31  // Will be overridden in each program
#define SRXL_DEVICE_PRIORITY        20
#define SRXL_DEVICE_INFO            (SRXL_DEVINFO_NO_RF)
#define SRXL_SUPPORTED_BAUD_RATES   0  // 115200 only
#define SRXL_CRC_OPTIMIZE_MODE      SRXL_CRC_OPTIMIZE_SPEED

//### USER PROVIDED INTERFACE FUNCTIONS ###

static inline void srxlChangeBaudRate(uint8_t uart, uint32_t baudRate)
{
    uartSetBaud(uart, baudRate);
}

static inline void srxlSendOnUart(uint8_t uart, uint8_t* pBuffer, uint8_t length)
{
    uartTransmit(uart, pBuffer, length);
}

static inline void srxlFillTelemetry(SrxlTelemetryData* pTelemetryData)
{
    userProvidedFillSrxlTelemetry(pTelemetryData);
}

static inline void srxlReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
{
    userProvidedReceivedChannelData(pChannelData, isFailsafe);
}

static inline bool srxlOnBind(SrxlFullID device, SrxlBindData info)
{
    (void)device;
    (void)info;
    return true;
}

static inline void srxlOnVtx(SrxlVtxData* pVtxData)
{
    userProvidedHandleVtxData(pVtxData);
}

#ifdef SRXL_INCLUDE_FWD_PGM_CODE
static inline void srxlOnFwdPgm(uint8_t* pData, uint8_t dataLength)
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
