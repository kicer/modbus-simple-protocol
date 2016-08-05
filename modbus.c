#include "modbus.h"

static uint8_t  g_modbus_dev_id = 0xFF;
static uint16_t g_modbus_regs[MODBUS_REGS_CNT];

static uint8_t  g_modbus_wait_send = 0;
static uint8_t  g_modbus_send_idx = 0;
static uint8_t  g_modbus_send[MODBUS_SEND_BUF_SIZE];

#define MODBUS_WAIT_SEND(n)  do { \
    g_modbus_wait_send = (n);     \
    g_modbus_send_idx = 0;        \
} while(0)

enum {
    LOCK_READ,
    LOCK_WRITE
};

static uint8_t g_lock_st   = 0;
static uint8_t g_lock_code = 0;
static uint16_t g_lock_reg  = 0;
static uint16_t g_lock_val  = 0;

#define DATA_LOCK(t) do { \
    g_lock_st = (t);      \
} while(0)

#define IS_DATA_LOCK() (g_lock_st != 0)

#define DATA_LOCK_PUSH(c,r,v) do { \
    g_lock_code = (c);             \
    g_lock_reg  = (r);             \
    g_lock_val  = (v);             \
} while(0)

#define DATA_UNLOCK(t) do {                           \
    if(IS_DATA_LOCK()) {                              \
        g_lock_st = (0);                              \
        switch(g_lock_code) {                         \
            case MODBUS_FC_READ_HOLDING_REGISTERS:    \
            modbus_ack_read(g_lock_reg, g_lock_val);  \
            break;                                    \
            case MODBUS_FC_WRITE_SINGLE_REGISTER:     \
            modbus_ack_write(g_lock_reg, g_lock_val); \
            break;                                    \
        }                                             \
    }                                                 \
} while(0)

/* Table of CRC values for highorder byte */
static unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40 };

/* Table of CRC values for loworder byte */
static char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40 };

/*
 * Compute CRC-16
 */
void CRC16 ( uint8_t *puchMsg, int usDataLen, uint8_t *CRCHi, uint8_t *CRCLo ) {
    uint8_t uchCRCHi = 0xFF ;
    uint8_t uchCRCLo = 0xFF ;

    int uIndex ;

    while (usDataLen--) {
        uIndex = uchCRCLo ^ *puchMsg++;
        uchCRCLo = (uint8_t)(uchCRCHi ^ auchCRCHi[uIndex]);
        uchCRCHi = auchCRCLo[uIndex];
    }

    *CRCHi = uchCRCHi;
    *CRCLo = uchCRCLo;
}

static int modbus_ack_write(uint16_t reg_addr, uint16_t reg_val) {
    if(reg_addr < MODBUS_REGS_CNT) {
        g_modbus_regs[reg_addr] = reg_val;

        g_modbus_send[1] = MODBUS_FC_WRITE_SINGLE_REGISTER;
        g_modbus_send[2] = (uint8_t)((reg_addr>>8) & 0xFF);
        g_modbus_send[3] = (uint8_t)(reg_addr & 0xFF);
        g_modbus_send[4] = (uint8_t)((reg_val>>8) & 0xFF);
        g_modbus_send[5] = (uint8_t)(reg_val & 0xFF);

        CRC16(g_modbus_send, 6, g_modbus_send+6, g_modbus_send+7);
        MODBUS_WAIT_SEND(8);
    }

    return 0;
}

static int modbus_ack_read(uint16_t reg_addr, uint16_t cnt) {
    int i;

    if(reg_addr < MODBUS_REGS_CNT) {
        if(reg_addr+cnt <= MODBUS_REGS_CNT) {
            g_modbus_send[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
            g_modbus_send[2] = (uint8_t)(cnt+cnt);

            for(i=0; i<cnt; i++) {
                g_modbus_send[i+i+3] = (uint8_t)((g_modbus_regs[reg_addr+i]>>8)&0xFF);
                g_modbus_send[i+i+4] = (uint8_t)((g_modbus_regs[reg_addr+i])&0xFF);
            }

            CRC16(g_modbus_send, cnt+cnt+3, g_modbus_send+cnt+cnt+3, g_modbus_send+cnt+cnt+4);
            MODBUS_WAIT_SEND((uint8_t)(5+cnt+cnt));
        }
    }

    return 0;
}

int modbus_init(uint8_t dev_id) {
    int i;

    g_modbus_dev_id = dev_id;
    g_modbus_send[0] = dev_id;

    for(i=0; i<MODBUS_REGS_CNT; i++) {
        g_modbus_regs[i] = 0;
    }

    return 0;
}

int modbus_write_reg(uint16_t reg_addr, uint16_t reg_val) {
    if(reg_addr < MODBUS_REGS_CNT) {
        DATA_LOCK(LOCK_WRITE);
        g_modbus_regs[reg_addr] = reg_val;
        DATA_UNLOCK(LOCK_WRITE);
    }

    return 0;
}

uint16_t modbus_read_reg(uint16_t reg_addr) {
    uint16_t reg_val = 0;

    if(reg_addr < MODBUS_REGS_CNT) {
        DATA_LOCK(LOCK_READ);
        reg_val = g_modbus_regs[reg_addr];
        DATA_UNLOCK(LOCK_READ);
    }

    return reg_val;
}

void modbus_recv_byte(uint8_t ch) {
    static int idx = 0;
    static uint8_t uchCRCHi, uchCRCLo;
    static uint8_t dev, code;
    static uint16_t reg_addr, value;

    /* Todo: use sys-clock find header? */

    switch(idx) {
        case 0:
            idx += 1;
            uchCRCHi = 0xFF;
            uchCRCLo = 0xFF;
            dev = ch;
            if(dev != g_modbus_dev_id) idx = 0;
            break;
        case 1:
            idx += 1;
            code = ch;
            if(code != MODBUS_FC_READ_HOLDING_REGISTERS) {
                if(code != MODBUS_FC_WRITE_SINGLE_REGISTER) {
                    idx = 0;
                }
            }
            break;
        case 2:
            idx += 1;
            reg_addr = (ch<<8);
            break;
        case 3:
            idx += 1;
            reg_addr += ch;
            break;
        case 4:
            idx += 1;
            value = (ch<<8);
            break;
        case 5:
            idx += 1;
            value += ch;
            break;
        case 6:
            idx += 1;
            if(ch != uchCRCHi) idx = 0;
            break;
        case 7:
            idx = 0;
            if(ch == uchCRCLo) {
            if(IS_DATA_LOCK()) {
            DATA_LOCK_PUSH(code, reg_addr, value);
            } else {
                switch(code) {
                    case MODBUS_FC_READ_HOLDING_REGISTERS:
                    modbus_ack_read(reg_addr, value);
                    break;
                    case MODBUS_FC_WRITE_SINGLE_REGISTER:
                    modbus_ack_write(reg_addr, value);
                    break;
                }
            }
            }
            break;
    }

    if(idx <= 6) {
        int uIndex = uchCRCLo ^ ch;
        uchCRCLo = (uint8_t)(uchCRCHi ^ auchCRCHi[uIndex]);
        uchCRCHi = auchCRCLo[uIndex];
    }
}

int modbus_ack_byte(void) {
    int data = -1;

    if(g_modbus_wait_send > 0) {
        data = g_modbus_send[g_modbus_send_idx];
        g_modbus_wait_send -= 1;
        g_modbus_send_idx += 1;
    }

    return data;
}


/* slaver demo~ */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define MODBUS_DEBUG  printf

int main(int argc, char **argv) {
    int dev_id;

    srand(time(NULL));

    dev_id = rand() % 0x100;
    modbus_init(dev_id);

    while(1) {
        int i, byte;
        uint16_t reg_addr;
        uint16_t reg_value;
        uint8_t pkg[8];

        sleep(1);

        switch(rand()%3) {
            case 0:
                reg_addr = rand() % MODBUS_REGS_CNT;
                reg_value = rand() % 0x10000;
                MODBUS_DEBUG("SLAVER: modbus_write_reg(%d, 0x%04X)\n", reg_addr, reg_value);
                modbus_write_reg(reg_addr, reg_value);
                break;
            case 1:
                reg_addr = rand() % MODBUS_REGS_CNT;
                reg_value = rand() % 0x10000;
                pkg[0] = dev_id;
                pkg[1] = MODBUS_FC_WRITE_SINGLE_REGISTER;
                pkg[2] = (reg_addr >> 8) & 0xFF;
                pkg[3] = (reg_addr >> 0) & 0xFF;
                pkg[4] = (reg_value >> 8) & 0xFF;
                pkg[5] = (reg_value >> 0) & 0xFF;
                CRC16(pkg, 6, pkg+6, pkg+7);

                MODBUS_DEBUG("MASTER: MODBUS_FC_WRITE_SINGLE_REGISTER(%d, 0x%04x)\n", reg_addr, reg_value);
                for(i=0; i<8; i++) {
                    modbus_recv_byte(pkg[i]);
                }

                MODBUS_DEBUG("SLAVER: modbus_ack_byte():");
                while((byte = modbus_ack_byte()) != -1) {
                    printf("%02X ", byte);
                }
                MODBUS_DEBUG("\n");
                break;
            case 2:
                reg_addr = rand() % MODBUS_REGS_CNT;
                reg_value = rand() % (MODBUS_REGS_CNT-reg_addr) + 1;
                pkg[0] = dev_id;
                pkg[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
                pkg[2] = (reg_addr >> 8) & 0xFF;
                pkg[3] = (reg_addr >> 0) & 0xFF;
                pkg[4] = (reg_value >> 8) & 0xFF;
                pkg[5] = (reg_value >> 0) & 0xFF;
                CRC16(pkg, 6, pkg+6, pkg+7);

                MODBUS_DEBUG("MASTER: MODBUS_FC_READ_HOLDING_REGISTERS(%d, %d)\n", reg_addr, reg_value);
                for(i=0; i<8; i++) {
                    modbus_recv_byte(pkg[i]);
                }

                MODBUS_DEBUG("SLAVER: modbus_ack_byte():");
                while((byte = modbus_ack_byte()) != -1) {
                    printf("%02X ", byte);
                }
                MODBUS_DEBUG("\n");
                break;
        }
    }

    return 0;
}
