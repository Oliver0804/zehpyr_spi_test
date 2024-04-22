#ifndef ICM42688_REGISTERS_H_
#define ICM42688_REGISTERS_H_

#include <stdint.h>

// Accesible from all user banks
#define REG_BANK_SEL 0x76

// User Bank 0
#define UB0_REG_DEVICE_CONFIG 0x11
#define UB0_REG_DRIVE_CONFIG 0x13
#define UB0_REG_INT_CONFIG 0x14
#define UB0_REG_FIFO_CONFIG 0x16
#define UB0_REG_TEMP_DATA1 0x1D
#define UB0_REG_TEMP_DATA0 0x1E
#define UB0_REG_ACCEL_DATA_X1 0x1F
#define UB0_REG_ACCEL_DATA_X0 0x20
#define UB0_REG_ACCEL_DATA_Y1 0x21
#define UB0_REG_ACCEL_DATA_Y0 0x22
#define UB0_REG_ACCEL_DATA_Z1 0x23
#define UB0_REG_ACCEL_DATA_Z0 0x24
#define UB0_REG_GYRO_DATA_X1 0x25
#define UB0_REG_GYRO_DATA_X0 0x26
#define UB0_REG_GYRO_DATA_Y1 0x27
#define UB0_REG_GYRO_DATA_Y0 0x28
#define UB0_REG_GYRO_DATA_Z1 0x29
#define UB0_REG_GYRO_DATA_Z0 0x2A
#define UB0_REG_TMST_FSYNCH 0x2B
#define UB0_REG_TMST_FSYNCL 0x2C
#define UB0_REG_INT_STATUS 0x2D
#define UB0_REG_FIFO_COUNTH 0x2E
#define UB0_REG_FIFO_COUNTL 0x2F
#define UB0_REG_FIFO_DATA 0x30
#define UB0_REG_APEX_DATA0 0x31
#define UB0_REG_APEX_DATA1 0x32
#define UB0_REG_APEX_DATA2 0x33
#define UB0_REG_APEX_DATA3 0x34
#define UB0_REG_APEX_DATA4 0x35
#define UB0_REG_APEX_DATA5 0x36
#define UB0_REG_INT_STATUS2 0x37
#define UB0_REG_INT_STATUS3 0x38
#define UB0_REG_SIGNAL_PATH_RESET 0x4B
#define UB0_REG_INTF_CONFIG0 0x4C
#define UB0_REG_INTF_CONFIG1 0x4D
#define UB0_REG_PWR_MGMT0 0x4E
#define UB0_REG_GYRO_CONFIG0 0x4F
#define UB0_REG_ACCEL_CONFIG0 0x50
#define UB0_REG_GYRO_CONFIG1 0x51
#define UB0_REG_GYRO_ACCEL_CONFIG0 0x52
#define UB0_REG_ACCEFL_CONFIG1 0x53
#define UB0_REG_TMST_CONFIG 0x54
#define UB0_REG_APEX_CONFIG0 0x56
#define UB0_REG_SMD_CONFIG 0x57
#define UB0_REG_FIFO_CONFIG1 0x5F
#define UB0_REG_FIFO_CONFIG2 0x60
#define UB0_REG_FIFO_CONFIG3 0x61
#define UB0_REG_FSYNC_CONFIG 0x62
#define UB0_REG_INT_CONFIG0 0x63
#define UB0_REG_INT_CONFIG1 0x64
#define UB0_REG_INT_SOURCE0 0x65
#define UB0_REG_INT_SOURCE1 0x66
#define UB0_REG_INT_SOURCE3 0x68
#define UB0_REG_INT_SOURCE4 0x69
#define UB0_REG_FIFO_LOST_PKT0 0x6C
#define UB0_REG_FIFO_LOST_PKT1 0x6D
#define UB0_REG_SELF_TEST_CONFIG 0x70
#define UB0_REG_WHO_AM_I 0x75

// Additional definitions for other banks can be added below using the same pattern

#endif // ICM42688_REGISTERS_H_