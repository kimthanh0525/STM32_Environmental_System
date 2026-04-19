#define TRUE 1
#define FALSE 0
#define bool BYTE
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "diskio.h"
#include "fatfs_sd.h"


extern uint16_t Timer1, Timer2;

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;
static uint8_t PowerFlag = 0;

/***************************************
 * SPI Low Level Functions
 **************************************/
static void SELECT(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

static void DESELECT(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

static void SPI_TxByte(uint8_t data) {
    HAL_SPI_Transmit(HSPI_SDCARD, &data, 1, SPI_TIMEOUT);
}

static uint8_t SPI_RxByte(void) {
    uint8_t dummy = 0xFF, data;
    HAL_SPI_TransmitReceive(HSPI_SDCARD, &dummy, &data, 1, SPI_TIMEOUT);
    return data;
}

/***************************************
 * SD Card Helper Functions
 **************************************/
static uint8_t SD_ReadyWait(void) {
    uint8_t res;
    Timer2 = 500;
    do {
        res = SPI_RxByte();
    } while ((res != 0xFF) && Timer2);
    return res;
}

uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
    uint8_t crc, res;
    uint8_t n = 255;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);
        if (res > 1) return res;
    }

    DESELECT();
    SPI_TxByte(0xFF);
    if (SD_ReadyWait() != 0xFF) return 0xFF;

    SELECT();
    SPI_TxByte(cmd | 0x40);
    SPI_TxByte((uint8_t)(arg >> 24));
    SPI_TxByte((uint8_t)(arg >> 16));
    SPI_TxByte((uint8_t)(arg >> 8));
    SPI_TxByte((uint8_t)arg);

    if (cmd == CMD0) crc = 0x95;
    else if (cmd == CMD8) crc = 0x87;
    else crc = 0x01;
    SPI_TxByte(crc);

    do {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);

    return res;
}

static void SD_PowerOn(void) {
    uint8_t n;
    PowerFlag = 0;
    DESELECT();
    for (n = 0; n < 20; n++) SPI_TxByte(0xFF);

    HAL_Delay(50);

    Timer1 = 1000;
    while (Timer1) {
        if (SD_SendCmd(CMD0, 0) == 1) {
            PowerFlag = 1;
            break;
        }
        HAL_Delay(10);
    }
}

static bool SD_RxDataBlock(BYTE *buff, UINT len) {
    uint8_t token;
    Timer2 = 500;
    do {
        token = SPI_RxByte();
    } while ((token == 0xFF) && Timer2);

    if (token != 0xFE) return FALSE;

    do {
        *buff++ = SPI_RxByte();
    } while (--len);

    SPI_RxByte(); // CRC
    SPI_RxByte();
    return TRUE;
}

static bool SD_TxDataBlock(const uint8_t *buff, BYTE token) {
    uint8_t resp;
    if (SD_ReadyWait() != 0xFF) return FALSE;

    SPI_TxByte(token);
    if (token != 0xFD) {
        for (int i = 0; i < 512; i++) SPI_TxByte(buff[i]);
        SPI_TxByte(0xFF);
        SPI_TxByte(0xFF);
        resp = SPI_RxByte();
        if ((resp & 0x1F) != 0x05) return FALSE;
    }
    while (SPI_RxByte() == 0);
    return TRUE;
}

/***************************************
 * Public FatFs Disk Functions
 **************************************/
DSTATUS SD_disk_initialize(BYTE pdrv) {
    uint8_t n, ty, ocr[4];

    if (pdrv != 0) return STA_NOINIT;

    SD_PowerOn();
    if (PowerFlag == 0) return STA_NOINIT;

    ty = 0;
    if (SD_SendCmd(CMD8, 0x1AA) == 1) {        // SD Version 2
        for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
        if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
            Timer1 = 1000;
            while (Timer1) {
                if (SD_SendCmd(ACMD41, 0x40000000) == 0) break;
                HAL_Delay(10);
            }
            if (Timer1 && SD_SendCmd(CMD58, 0) == 0) {
                for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
                ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
            }
        }
    } else {    // SD V1 or MMC
        if (SD_SendCmd(ACMD41, 0) <= 1) {
            ty = CT_SD1;
            Timer1 = 1000;
            while (Timer1 && SD_SendCmd(ACMD41, 0));
        } else {
            ty = CT_MMC;
            Timer1 = 1000;
            while (Timer1 && SD_SendCmd(CMD1, 0));
        }
    }

    CardType = ty;
    DESELECT();
    SPI_TxByte(0xFF);

    if (ty) {
        Stat &= ~STA_NOINIT;
    }

    return Stat;
}

DSTATUS SD_disk_status(BYTE pdrv) {
    if (pdrv) return STA_NOINIT;
    return Stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;

    SELECT();
    if (count == 1) {
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) 
            count = 0;
    } else {
        if (SD_SendCmd(CMD18, sector) == 0) {
            do {
                if (!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while (--count);
            SD_SendCmd(CMD12, 0);
        }
    }
    DESELECT();
    SPI_TxByte(0xFF);

    return count ? RES_ERROR : RES_OK;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;

    SELECT();
    if (count == 1) {
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE)) 
            count = 0;
    } else {
        if (CardType & CT_SD2) SD_SendCmd(ACMD23, count);
        if (SD_SendCmd(CMD25, sector) == 0) {
            do {
                if (!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!SD_TxDataBlock(0, 0xFD)) count = 1;
        }
    }
    DESELECT();
    SPI_TxByte(0xFF);

    return count ? RES_ERROR : RES_OK;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE ctrl, void *buff) {
    DRESULT res = RES_ERROR;
    uint8_t csd[16];
    DWORD csize;

    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    SELECT();
    switch (ctrl) {
        case GET_SECTOR_COUNT:
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16)) {
                if ((csd[0] >> 6) == 1) { // SDC Ver 2.00
                    csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                    *(DWORD*)buff = (DWORD)csize << 10;
                } else {
                    uint8_t n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                    *(DWORD*)buff = (DWORD)csize << (n - 9);
                }
                res = RES_OK;
            }
            break;

        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            res = RES_OK;
            break;

        case CTRL_SYNC:
            if (SD_ReadyWait() == 0xFF) res = RES_OK;
            break;
    }
    DESELECT();
    SPI_TxByte(0xFF);

    return res;
}