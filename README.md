# Modbus-Simple-Protocol (MSP)

* Only support modbus 03/06 code
* Use interrupt recv/send mode
* MCU call modbus_set_reg() to set reg_value
* ISR call modbus_recv_byte() to process recv-byte
* ISR call modbus_ack_byte() to ack master 
