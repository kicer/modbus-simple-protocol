# Modbus-Simple-Protocol (MSP)

* Modbus 03/06 code support only
* Register address from 0~N only
* Use interrupt recv/send mode
* MCU call modbus_read_reg() to readback reg_value
* MCU call modbus_write_reg() to set reg_value
* ISR call modbus_recv_byte() to process recv-byte
* ISR call modbus_ack_byte() to ack master 
