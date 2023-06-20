/*!
 * \file      nvmc.h
 *
 * \brief     Nvrom implementation
 *
 * \author    extern
 */
 
#ifndef _NVMC_H_
#define _NVMC_H_

/*********************************************************************
 * INCLUDES
 */
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "nrf_nvmc.h"
#include "fds.h"
#include "sdk_errors.h"
#include "user_struct.h"
/*********************************************************************
 * DEFINITIONS
 */
#define PDS_FIRST_RESERVED_FILE_ID    (0x0000)  /**< The beginning of the range of file IDs reserved for Peer Manager. */
#define PDS_LAST_RESERVED_FILE_ID     (0xBFFF)  /**< The end of the range of file IDs reserved for Peer Manager. */

#define FDS_READ            	0x00
#define FDS_WRITE				0x01

#define DATA_ID_TO_RECORD_KEY   0x00B0

#define FDS_BUFFER_SIZE			64

/****************************API FUNCTIONS***************************/
void nvmc_init( USER_DATA_t *ptr_sruct );
bool xfds_init(void);
void xfds_contrl(uint16_t fileId, uint16_t keyId, uint8_t readWriteFlag, uint8_t *pData, uint32_t dataLen);
#endif /* _NVMC_H_ */
