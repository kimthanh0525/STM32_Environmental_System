#ifndef __FATFS_SD_H
#define __FATFS_SD_H

#include "stm32f1xx_hal.h"
#include "diskio.h"

/* Definitions for MMC/SDC command - Sửa lại để khớp với hàm SendCmd */
#define CMD0     (0)           /* GO_IDLE_STATE */
#define CMD1     (1)           /* SEND_OP_COND */
#define CMD8     (8)           /* SEND_IF_COND */
#define CMD9     (9)           /* SEND_CSD */
#define CMD10    (10)          /* SEND_CID */
#define CMD12    (12)          /* STOP_TRANSMISSION */
#define CMD16    (16)          /* SET_BLOCKLEN */
#define CMD17    (17)          /* READ_SINGLE_BLOCK */
#define CMD18    (18)          /* READ_MULTIPLE_BLOCK */
#define CMD23    (23)          /* SET_BLOCK_COUNT */
#define CMD24    (24)          /* WRITE_BLOCK */
#define CMD25    (25)          /* WRITE_MULTIPLE_BLOCK */
#define CMD55    (55)          /* APP_CMD */
#define CMD58    (58)          /* READ_OCR */

/* Các lệnh ACMD (Sử dụng bit 0x80 để hàm SendCmd tự nhận diện và gửi CMD55 trước) */
#define ACMD41   (0x80+41)     /* SEND_OP_COND (ACMD) */
#define ACMD23   (0x80+23)     /* SET_WR_BLK_ERASE_COUNT (ACMD) */

/* MMC card type flags */
#define CT_MMC      0x01
#define CT_SD1      0x02
#define CT_SD2      0x04
#define CT_SDC      0x06
#define CT_BLOCK    0x08

/* Functions */
DSTATUS SD_disk_initialize (BYTE pdrv);
DSTATUS SD_disk_status (BYTE pdrv);
DRESULT SD_disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg);

#define SPI_TIMEOUT 100

extern SPI_HandleTypeDef    hspi1;
#define HSPI_SDCARD         &hspi1
#define SD_CS_PORT          GPIOA
#define SD_CS_PIN           GPIO_PIN_4

void UART_Send(char *str);

#endif