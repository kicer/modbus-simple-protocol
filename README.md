# Modbus-Simple-Protocol (MSP)

* modbus 03/06 code support only
* register address from 0~N only
* use interrupt recv/send mode
* support data-lock to avoid conflct of app/ISR
* app call modbus_read_reg() to readback reg_value
* app call modbus_write_reg() to set reg_value
* ISR call modbus_recv_byte() to process recv-byte
* ISR call modbus_ack_byte() to ack master 
