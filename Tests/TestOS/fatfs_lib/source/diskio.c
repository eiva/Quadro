/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "diskio.h"

#include <string.h> // memcpy

#include "stm32f4xx.h"
#include "sdio_sd.h"

#define BLOCK_SIZE            512 /* Block Size in Bytes */



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
        BYTE drv                                /* Physical drive nmuber (0..) */
)
{
        SD_Error  Status;

//	printf("disk_initialize %d\n", drv);

 	/* Supports only single drive */
        if (drv)
        {
                return STA_NOINIT;
        }
/*-------------------------- SD Init ----------------------------- */
  Status = SD_Init();
        if (Status!=SD_OK )
        {
//					puts("Initialization Fail");
                return STA_NOINIT;

        }
        else
        {

                return RES_OK;
        }

}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
        BYTE drv                /* Physical drive nmuber (0..) */
)
{
	DSTATUS stat = 0;
	
	if (SD_Detect() != SD_PRESENT)
		stat |= STA_NODISK;

	// STA_NOTINIT - Subsystem not initailized
	// STA_PROTECTED - Write protected, MMC/SD switch if available
	
	return(stat);
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
DRESULT disk_read (
        BYTE drv,               /* Physical drive nmuber (0..) */
        BYTE *buff,             /* Data buffer to store read data */
        DWORD sector,   				/* Sector address (LBA) */
        UINT count              /* Number of sectors to read (1..255) */
)
{
  SDTransferState state;
//  DRESULT res = RES_OK;

  if ( SD_Detect( ) != SD_PRESENT)
    return RES_NOTRDY;

  if ( count == 0)
    return RES_PARERR;

  //SD_ReadMultiBlocks ( buff, sector, 512, 1 );
  SD_ReadMultiBlocksFIXED ( buff, sector, 512, 1 );
  SD_WaitReadOperation ( );

  while ( 1 )
  {
    state = SD_GetStatus ( );
    if ( state == SD_TRANSFER_OK )
      break;
  } // while

  return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
        BYTE drv,                       /* Physical drive nmuber (0..) */
        const BYTE *buff,       /* Data to be written */
        DWORD sector,           /* Sector address (LBA) */
        UINT count                      /* Number of sectors to write (1..255) */
)
{
  SDTransferState state;
//  DRESULT res = RES_OK;

  if ( SD_Detect( ) != SD_PRESENT )
    return RES_NOTRDY;

  if ( count == 0)
    return RES_PARERR;


//  SD_WriteMultiBlocks ( (uint8_t *)buff, sector, 512, 1 );
  SD_WriteMultiBlocksFIXED ( (uint8_t *)buff, sector, 512, 1 );
  SD_WaitWriteOperation ( );

  while ( 1 )
  {
    state = SD_GetStatus ( );
    if ( state == SD_TRANSFER_OK )
      break;
  } // while

  return RES_OK;
}
#endif /* _READONLY */




/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
        BYTE drv,               /* Physical drive nmuber (0..) */
        BYTE ctrl,              /* Control code */
        void *buff              /* Buffer to send/receive control data */
)
{
        return RES_OK;
}

