#ifndef _MODEBUS_H_
#define _MODEBUS_H_


/* Modbus Simple Protocol (MSP)
 *
 * only support 03/06 function code,
 * register address from 0~N
 */


#include <stdint.h>

#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06

#define MODBUS_REGS_CNT                     16
#define MODBUS_RECV_BUF_SIZE                (8)
#define MODBUS_SEND_BUF_SIZE                (5+MODBUS_REGS_CNT+MODBUS_REGS_CNT)

int modbus_init(uint8_t dev_id);
int modbus_write_reg(uint16_t reg_addr, uint16_t reg_val);
uint16_t modbus_read_reg(uint16_t reg_addr);

void modbus_recv_byte(uint8_t ch);
int  modbus_ack_byte(void);


#endif /* _MODEBUS_H_ */
