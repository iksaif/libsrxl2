/*
 * Minimal spm_srxl.h config for SRXL2_Master_Standalone.
 *
 * This module never links spm_srxl.c, so none of the srxlXxx() callback
 * hooks that file requires (srxlSendOnUart, srxlFillTelemetry, ...) are
 * needed here -- spm_srxl.h only requires SRXL_NUM_OF_BUSES to be defined.
 * (Already satisfied globally by the top-level CMakeLists.txt via
 * -DSRXL_NUM_OF_BUSES=1; this file just needs to exist since spm_srxl.h
 * unconditionally #includes "spm_srxl_config.h".)
 *
 * MIT License
 */

#ifndef _SRXL_CONFIG_H_
#define _SRXL_CONFIG_H_

#ifndef SRXL_NUM_OF_BUSES
#define SRXL_NUM_OF_BUSES 1
#endif

#endif /* _SRXL_CONFIG_H_ */
