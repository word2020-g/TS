/*!
 * \file      nvmc.c
 *
 * \brief     Nvrom implementation
 *
 * \author    extern
 */

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nvmc.h"
#include "nrf_nvmc.h"
#include "fds.h"
#include "crc16.h"

static void fdsCallbackFunc(fds_evt_t const *pFdsEvent);
static bool readFdsData(uint16_t fileId, uint16_t key, uint8_t *pData, uint8_t dataLen);
static bool writeFdsData(uint16_t fileId, uint16_t key, uint8_t *pData, uint32_t dataLen);
static uint8_t getWordLength(uint32_t dataLen);
static uint32_t flashUpdateWithWaiting(fds_record_desc_t *pDesc, fds_record_t *pRec);
static uint32_t flashWriteWithWaiting(fds_record_desc_t *pDesc, fds_record_t *pRec);
static uint32_t flashDelete(fds_record_desc_t *pDesc);
static void waitForFlashOperationToComplete(void);

/*********************************************************************
 * LOCAL VARIABLES
 */
static bool volatile s_fdsIfInitialized;												// FDS初始化完成标志
static fds_record_desc_t s_recordDesc;
static uint8_t s_dataBuffer[FDS_BUFFER_SIZE];

/******************************PUBLIC FUNCTIONS*********************************/
bool xfds_init(void)
{
    ret_code_t retCode;
	fds_record_t record;
    
    fds_record_desc_t   recordDesc = {0};
    fds_find_token_t    token = {0};
		
	(void)fds_register(fdsCallbackFunc);										
	
	retCode = fds_init();																		
	APP_ERROR_CHECK(retCode);
	while(!s_fdsIfInitialized)														
    {
        sd_app_evt_wait();																
    }

    //Fine the previous records. 
	retCode = fds_record_find(PDS_FIRST_RESERVED_FILE_ID, DATA_ID_TO_RECORD_KEY, 
                        &recordDesc, &token);	
    
    if( retCode != NRF_SUCCESS)
    {
        s_recordDesc.record_id = 0;
        record.file_id         = PDS_FIRST_RESERVED_FILE_ID;
        record.key             = DATA_ID_TO_RECORD_KEY;
        record.data.p_data     = s_dataBuffer;
        record.data.length_words = sizeof( USER_DATA_t ); 
        
        //Just create a fds project.
        retCode = fds_record_write(&s_recordDesc, &record);
        
        if(retCode == NRF_SUCCESS)
        {
            return true;
        }
        else
        {
            fds_gc();
            return false;
        }
    }
    
	return true;
}

/*!
 * \brief Fds read/write the memory.
 *
 * \param [IN]：
 * \retval none
 */
void xfds_contrl(uint16_t fileId, uint16_t keyId, uint8_t readWriteFlag, uint8_t *pData, uint32_t dataLen)
{
    if(readWriteFlag == FDS_READ)                       								
    {
		readFdsData(fileId, keyId, pData, dataLen);
    }
    else if(readWriteFlag == FDS_WRITE) 																				
    {
		writeFdsData(fileId, keyId, pData, dataLen);
    }
}

void nvmc_init( USER_DATA_t *ptr_sruct )
{
    if( xfds_init( ) )
    {
        xfds_contrl(PDS_FIRST_RESERVED_FILE_ID, DATA_ID_TO_RECORD_KEY,
                FDS_READ, s_dataBuffer, sizeof(USER_DATA_t));
    }        
    
    if( ( (USER_DATA_t *)s_dataBuffer)->producted != DEV_PRODUCTED )
    {
        ptr_sruct->active = DEV_ACTIVE_DISABLE;
        ptr_sruct->stor_flg = 0;
    } 
    else
    {   
        (void)memcpy( (void *)ptr_sruct, s_dataBuffer, sizeof(USER_DATA_t) );
    }  
}

/******************************LOCAL FUNCTIONS*********************************/
/*!
 * \brief Fds event callback.
 *
 * \param [IN]: none
 * \retval none
 */
static void fdsCallbackFunc(fds_evt_t const *pFdsEvent)
{
    switch(pFdsEvent->id)
    {
        case FDS_EVT_INIT:
		{
            if(pFdsEvent->result == NRF_SUCCESS)
            {
               s_fdsIfInitialized = true;
            }
		} 
        break;

        case FDS_EVT_WRITE:
        {
			if(pFdsEvent->result == NRF_SUCCESS)
			{
                //NRF_LOG_INFO("Write success.");
			}
        } 
        break;
		
		case FDS_EVT_UPDATE:
        {
			if(pFdsEvent->result == NRF_SUCCESS)
			{
                //NRF_LOG_INFO("Update success.");
			}
        } 
        break;

        case FDS_EVT_DEL_RECORD:
        {
            if(pFdsEvent->result == NRF_SUCCESS)
            {
                //NRF_LOG_INFO("Record success.");
            }
        } 
        break;

        default:
			break;
    }
}

static bool readFdsData(uint16_t fileId, uint16_t key, uint8_t *pData, uint8_t dataLen)
{
	ret_code_t retCode;
	fds_record_desc_t recordDesc = {0};
	fds_find_token_t token = {0};

	retCode = fds_record_find(fileId, key, &recordDesc, &token);

	if(retCode == NRF_SUCCESS)
	{
		fds_flash_record_t record = {0};
		fds_record_open(&recordDesc, &record);

		memcpy(pData, record.p_data, dataLen);
		fds_record_close(&recordDesc);

		return true;
	}
	return false;
}

static bool writeFdsData(uint16_t fileId, uint16_t key, uint8_t *pData, uint32_t dataLen)
{
	ret_code_t retCode;
	fds_record_desc_t recordDesc = {0};
	fds_find_token_t token = {0};
	fds_record_t record = {0};
		
    //Fine the previous records. 
	retCode = fds_record_find(fileId, key, &recordDesc, &token);
		
	record.file_id = fileId;
	record.key = key;		
	memcpy(s_dataBuffer, pData, dataLen);
	record.data.p_data = (uint8_t *) s_dataBuffer;
	record.data.length_words = getWordLength(dataLen);

	if(retCode == NRF_SUCCESS)			
	{
		retCode = flashUpdateWithWaiting(&recordDesc, &record);
	}
	else 
	{
		flashDelete(&recordDesc);
		retCode = flashWriteWithWaiting(&recordDesc, &record);
	}
	return true;
}

static uint8_t getWordLength(uint32_t dataLen)
{
	uint8_t word_length ;
	
	word_length = dataLen / 4;
	
	if(dataLen % 4)
	{
		word_length += 1;	
	}
	return word_length;
}

static uint32_t flashUpdateWithWaiting(fds_record_desc_t *pDesc, fds_record_t *pRec)
{
	ret_code_t retCode; 
	
	retCode = fds_record_update(pDesc, pRec);
	waitForFlashOperationToComplete();
	
	return retCode;
}

static uint32_t flashWriteWithWaiting(fds_record_desc_t *pDesc, fds_record_t *pRec)
{
	ret_code_t retCode; 
	
	retCode = fds_record_write(pDesc, pRec);
	waitForFlashOperationToComplete();

	return retCode;
}

static uint32_t flashDelete(fds_record_desc_t *pDesc)
{
	ret_code_t retCode; 
	
	retCode = fds_record_delete(pDesc);
	waitForFlashOperationToComplete();

	return retCode;
}

static void waitForFlashOperationToComplete(void)
{
	nrf_delay_ms(10);
}
