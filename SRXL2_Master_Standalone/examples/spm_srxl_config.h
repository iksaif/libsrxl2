/*
 * spm_srxl.h config for the coexistence example's *slave* engine instance.
 *
 * The example runs the real upstream spm_srxl.c compiled WITHOUT
 * SRXL_INCLUDE_MASTER_CODE, i.e. as a pure slave. That only requires the
 * handful of interface hooks below; the example defines the routing
 * functions (example_slave_*) in coexist_sim.c.
 *
 * MIT License
 */

#ifndef _SRXL_CONFIG_H_
#define _SRXL_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Routing functions provided by coexist_sim.c */
void example_slave_uart_send(uint8_t uart, uint8_t *buf, uint8_t len);
void example_slave_set_baud(uint8_t uart, uint32_t baud);
void example_slave_fill_telemetry(SrxlTelemetryData *pTelemetry);
void example_slave_recv_channel(SrxlChannelData *pChannelData, bool isFailsafe);

#define SRXL_NUM_OF_BUSES           1
#define SRXL_DEVICE_ID              0x40  /* ESC (overridden at init) */
#define SRXL_DEVICE_PRIORITY        10
#define SRXL_DEVICE_INFO            (SRXL_DEVINFO_NO_RF)
#define SRXL_SUPPORTED_BAUD_RATES   0
#define SRXL_CRC_OPTIMIZE_MODE      SRXL_CRC_OPTIMIZE_SPEED

static inline void srxlChangeBaudRate(uint8_t uart, uint32_t baudRate)
{
    example_slave_set_baud(uart, baudRate);
}

static inline void srxlSendOnUart(uint8_t uart, uint8_t *pBuffer, uint8_t length)
{
    example_slave_uart_send(uart, pBuffer, length);
}

static inline void srxlFillTelemetry(SrxlTelemetryData *pTelemetryData)
{
    example_slave_fill_telemetry(pTelemetryData);
}

static inline void srxlReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafe)
{
    example_slave_recv_channel(pChannelData, isFailsafe);
}

static inline bool srxlOnBind(SrxlFullID device, SrxlBindData info)
{
    (void)device;
    (void)info;
    return true;
}

static inline void srxlOnVtx(SrxlVtxData *pVtxData)
{
    (void)pVtxData;
}

static inline void srxlEnterCriticalSection(void) { }
static inline void srxlExitCriticalSection(void) { }

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _SRXL_CONFIG_H_
