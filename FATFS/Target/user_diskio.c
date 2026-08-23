/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   SPI SD card diskio driver for FatFs (STM32F4)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"   /* for hspi1, SPI1_CS_GPIO_Port / SPI1_CS_Pin, HAL functions */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* SD card commands (SPI mode) */
#define CMD0    (0x40+0)    /* GO_IDLE_STATE */
#define CMD1    (0x40+1)    /* SEND_OP_COND (MMC) */
#define CMD8    (0x40+8)    /* SEND_IF_COND */
#define CMD9    (0x40+9)    /* SEND_CSD */
#define CMD10   (0x40+10)   /* SEND_CID */
#define CMD12   (0x40+12)   /* STOP_TRANSMISSION */
#define CMD16   (0x40+16)   /* SET_BLOCKLEN */
#define CMD17   (0x40+17)   /* READ_SINGLE_BLOCK */
#define CMD18   (0x40+18)   /* READ_MULTIPLE_BLOCK */
#define CMD23   (0x40+23)   /* SET_BLOCK_COUNT (MMC) */
#define ACMD23  (0xC0+23)   /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24   (0x40+24)   /* WRITE_BLOCK */
#define CMD25   (0x40+25)   /* WRITE_MULTIPLE_BLOCK */
#define CMD41   (0x40+41)   /* SEND_OP_COND (SDC) */
#define CMD55   (0x40+55)   /* APP_CMD */
#define CMD58   (0x40+58)   /* READ_OCR */

/* Card type flags (CardType) */
#define CT_MMC      0x01
#define CT_SD1      0x02
#define CT_SD2      0x04
#define CT_SDC      (CT_SD1|CT_SD2)
#define CT_BLOCK    0x08

/* SPI handle used for the SD card */
extern SPI_HandleTypeDef hspi1;
#define SD_SPI_HANDLE   hspi1

#define SD_CS_LOW()     HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;
static volatile UINT Timer1, Timer2;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/* USER CODE BEGIN DECL2 */

/* Low-level SPI byte transfer */
static BYTE SPI_RxTx(BYTE data)
{
    BYTE rx = 0xFF;
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &data, &rx, 1, 100);
    return rx;
}

static void SPI_RxData(BYTE *buff, UINT len)
{
    memset(buff, 0xFF, len);
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, buff, buff, len, 200);
}

/* Wait until card releases DO (goes to 0xFF), up to ~500ms */
static int SD_WaitReady(void)
{
    BYTE res;
    uint32_t start = HAL_GetTick();
    do {
        res = SPI_RxTx(0xFF);
    } while ((res != 0xFF) && ((HAL_GetTick() - start) < 500));
    return (res == 0xFF) ? 1 : 0;
}

static void SD_Deselect(void)
{
    SD_CS_HIGH();
    SPI_RxTx(0xFF); /* extra clock */
}

static int SD_Select(void)
{
    SD_CS_LOW();
    SPI_RxTx(0xFF);
    if (SD_WaitReady()) return 1;
    SD_Deselect();
    return 0;
}

/* Receive a data block */
static int SD_RxDataBlock(BYTE *buff, UINT len)
{
    BYTE token;
    uint32_t start = HAL_GetTick();

    do {
        token = SPI_RxTx(0xFF);
    } while ((token == 0xFF) && ((HAL_GetTick() - start) < 200));

    if (token != 0xFE) return 0; /* not a valid data token */

    SPI_RxData(buff, len);

    /* discard CRC (2 bytes) */
    SPI_RxTx(0xFF);
    SPI_RxTx(0xFF);

    return 1;
}

#if _USE_WRITE == 1
/* Send a data block */
static int SD_TxDataBlock(const BYTE *buff, BYTE token)
{
    BYTE resp;
    BYTE tmp[2] = {0xFF, 0xFF};

    if (!SD_WaitReady()) return 0;

    SPI_RxTx(token);

    if (token != 0xFD) /* not stop token */
    {
        HAL_SPI_Transmit(&SD_SPI_HANDLE, (uint8_t *)buff, 512, 200);
        HAL_SPI_Transmit(&SD_SPI_HANDLE, tmp, 2, 100); /* dummy CRC */

        resp = SPI_RxTx(0xFF);
        if ((resp & 0x1F) != 0x05) return 0; /* data rejected */
    }
    return 1;
}
#endif

/* Send a command and return the R1 response */
static BYTE SD_SendCmd(BYTE cmd, DWORD arg)
{
    BYTE n, res;

    if (cmd & 0x80) { /* ACMD<n> = CMD55 + CMD<n> */
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);
        if (res > 1) return res;
    }

    if (cmd != CMD12) {
        SD_Deselect();
        if (!SD_Select()) return 0xFF;
    }

    /* send command packet: start bit + cmd, 4 arg bytes, CRC */
    SPI_RxTx(cmd);
    SPI_RxTx((BYTE)(arg >> 24));
    SPI_RxTx((BYTE)(arg >> 16));
    SPI_RxTx((BYTE)(arg >> 8));
    SPI_RxTx((BYTE)arg);

    n = 0x01;                 /* dummy CRC */
    if (cmd == CMD0) n = 0x95; /* valid CRC for CMD0 */
    if (cmd == CMD8) n = 0x87; /* valid CRC for CMD8 (arg = 0x1AA) */
    SPI_RxTx(n);

    if (cmd == CMD12) SPI_RxTx(0xFF); /* skip a stuff byte on stop read */

    n = 10;
    do {
        res = SPI_RxTx(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

/* USER CODE END DECL2 */

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    BYTE n, cmd, ty, ocr[4];
    uint32_t start;

    if (pdrv != 0) return STA_NOINIT;

    /* 80 dummy clocks with CS high, card not yet selected */
    SD_CS_HIGH();
    for (n = 0; n < 10; n++) SPI_RxTx(0xFF);

    ty = 0;
    BYTE r = SD_SendCmd(CMD0, 0);           /* enter Idle state */
    if (r == 1) {
        start = HAL_GetTick();
        if (SD_SendCmd(CMD8, 0x1AA) == 1) { /* SDv2? */
            for (n = 0; n < 4; n++) ocr[n] = SPI_RxTx(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                /* card supports 2.7-3.6V, wait for leave idle (ACMD41 with HCS bit) */
                while ((HAL_GetTick() - start) < 1000 && SD_SendCmd(41 | 0x80, 1UL << 30));
                if ((HAL_GetTick() - start) < 1000 && SD_SendCmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = SPI_RxTx(0xFF);
                    ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
                }
            }
        } else { /* SDv1 or MMC */
            if (SD_SendCmd(41 | 0x80, 0) <= 1) {
                ty = CT_SD1;
                cmd = 41 | 0x80;
            } else {
                ty = CT_MMC;
                cmd = CMD1;
            }
            while ((HAL_GetTick() - start) < 1000 && SD_SendCmd(cmd, 0));
            if (!((HAL_GetTick() - start) < 1000) || SD_SendCmd(CMD16, 512) != 0) {
                ty = 0;
            }
        }
    }
    CardType = ty;
    SD_Deselect();

    if (ty) {
        Stat &= ~STA_NOINIT;
    } else {
        Stat = STA_NOINIT;
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    if (pdrv != 0) return STA_NOINIT;
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    if (pdrv != 0 || count == 0) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    /* byte addressing cards need address in bytes, not blocks */
    if (!(CardType & CT_BLOCK)) sector *= 512;

    if (count == 1) {
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) {
            count = 0;
        }
    } else {
        if (SD_SendCmd(CMD18, sector) == 0) {
            do {
                if (!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while (--count);
            SD_SendCmd(CMD12, 0); /* stop transmission */
        }
    }
    SD_Deselect();

    return count ? RES_ERROR : RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
    if (pdrv != 0 || count == 0) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (!(CardType & CT_BLOCK)) sector *= 512;

    if (count == 1) {
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE)) {
            count = 0;
        }
    } else {
        if (CardType & CT_SDC) SD_SendCmd(ACMD23, count);
        if (SD_SendCmd(CMD25, sector) == 0) {
            do {
                if (!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!SD_TxDataBlock(0, 0xFD)) count = 1; /* stop token */
        }
    }
    SD_Deselect();

    return count ? RES_ERROR : RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
    BYTE n, csd[16];
    DWORD csize;

    if (pdrv != 0) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd) {
    case CTRL_SYNC:
        if (SD_Select()) res = RES_OK;
        SD_Deselect();
        break;

    case GET_SECTOR_COUNT:
        if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16)) {
            if ((csd[0] >> 6) == 1) { /* SDC ver 2.00 (CSD v2) */
                csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(DWORD *)buff = csize << 10; /* sectors (512B each) */
            } else { /* CSD v1 */
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                *(DWORD *)buff = csize << (n - 9);
            }
            res = RES_OK;
        }
        SD_Deselect();
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 512;
        res = RES_OK;
        break;

    case CTRL_TRIM:
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */
