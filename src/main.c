/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <assert.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/lbs.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>

#include "ICM42688_Registers.h"

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define RUN_LED_BLINK_INTERVAL 1000

#define USER_LED DK_LED3

#define USER_BUTTON DK_BTN1_MSK

// #define SPI_OP_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE)
// static const struct spi_dt_spec spiDev = SPI_DT_SPEC_GET(DT_NODELABEL(icm20689), SPI_OP_MODE, 0);

#define SPIOP (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE)
static const struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);
#define WHO_AM_I_REG 0x75
#define RESET_REG 0x6b
#define WHO_AM_I_VAL 0x47
#define ICM_READ_BIT_MASK (0x80)
#define ICM_WRITE_BIT_MASK (0x7F)
static struct inv_icm207xx_serif icm207xx_serif;

extern const struct device *spi_dev;
const struct spi_config spi_cfg = {
	.frequency = 1000000, // 1 MHz
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.slave = 0, // CS 线路
	.cs = NULL, // 如果需要，这里可以指定一个 spi_cs_control 结构体来控制片选线
};
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB

static bool app_button_state;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};
struct inv_icm207xx_serif
{
	void *context;
	int (*read_reg)(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
	int (*write_reg)(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);
	uint32_t max_read;
	uint32_t max_write;
	int is_spi;
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	dk_set_led_off(CON_STATUS_LED);
}

#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		printk("Security changed: %s level %u\n", addr, level);
	}
	else
	{
		printk("Security failed: %s level %u err %d\n", addr, level,
			   err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_LBS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void app_led_cb(bool led_state)
{
	dk_set_led(USER_LED, led_state);
}

static bool app_button_cb(void)
{
	return app_button_state;
}

static struct bt_lbs_cb lbs_callbacs = {
	.led_cb = app_led_cb,
	.button_cb = app_button_cb,
};

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & USER_BUTTON)
	{
		uint32_t user_button_state = button_state & USER_BUTTON;

		bt_lbs_send_button_state(user_button_state);
		app_button_state = user_button_state ? true : false;
	}
}

static int init_button(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err)
	{
		printk("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}
int spi_master_transfer(uint8_t reg, const uint8_t *tx, uint32_t tx_len, uint8_t *rx, uint32_t rx_len)
{

	static uint8_t _send[32];
	static uint8_t _recv[64];
	memset(_send, 0, sizeof(_send));
	_send[0] = reg;

	if (tx_len && tx_len < sizeof(_send) - 1) // 确保不会超出 _send 数组的界限
	{
		memcpy(&_send[1], tx, tx_len);
	}
#ifdef DEBUG
	printk("reg 0x%02x \n", reg);
	// print tx
	printk("tx_len %d \n", tx_len);
	for (int i = 0; i < tx_len; i++)
	{
		printk("0x%02x ", tx[i]);
	}
	// print _send
	printk("\n");
	for (int i = 0; i < tx_len + 1; i++)
	{
		printk("0x%02x ", _send[i]);
	}

	printk("\n");
#endif
	struct spi_buf tx_spi_buf = {.buf = (void *)_send, .len = sizeof(reg) + tx_len};
	struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_buf = {.buf = (void *)_recv, .len = tx_spi_buf.len + rx_len};
	struct spi_buf_set rx_set = {.buffers = &rx_spi_buf, .count = 1};
	spi_transceive_dt(&spispec, &tx_set, &rx_set);

	if (rx_len)
	{
		// printk("_recv[tx_spi_buf.len] 0x%02x \n", _recv[tx_spi_buf.len]);
		memcpy(rx, &_recv[tx_spi_buf.len], rx_len);
	}

	return 0;
}
static int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	(void)context;
	return spi_master_transfer(reg | ICM_READ_BIT_MASK, NULL, 0, rbuffer, rlen);
}

static int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
	(void)context;
	return spi_master_transfer(reg & ICM_WRITE_BIT_MASK, wbuffer, wlen, NULL, 0);
}

void read_spi_data_from_register(void)
{
	uint8_t tx_buffer[] = {WHO_AM_I_REG | 0x80}; // 设定读取命令，这里假设需要设置最高位为1
	uint8_t rx_buffer[10] = {0};				 // 假设我们需要读取 10 个字节的数据

	struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)};
	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer)};

	struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1};
	struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1};

	/* 调用 spi_transceive 来发送读取命令并接收数据 */
	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
	if (ret == 0)
	{
		printk("Data read from register successfully.\n");
		// 这里可以添加代码来处理接收到的数据
	}
	else
	{
		printk("Failed to read data from register: %d\n", ret);
	}
}
void main(void)
{
	int blink_status = 0;
	int err;

	printk("Starting Bluetooth Peripheral LBS example\n");

	err = dk_leds_init();
	if (err)
	{
		printk("LEDs init failed (err %d)\n", err);
		return;
	}

	err = init_button();
	if (err)
	{
		printk("Button init failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_BT_LBS_SECURITY_ENABLED))
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err)
		{
			printk("Failed to register authorization callbacks.\n");
			return;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err)
		{
			printk("Failed to register authorization info callbacks.\n");
			return;
		}
	}

	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_lbs_init(&lbs_callbacs);
	if (err)
	{
		printk("Failed to init LBS (err:%d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
						  sd, ARRAY_SIZE(sd));
	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");

	if (!spi_is_ready(&spispec))
	{
		printk("Device spi not ready, aborting test");
		return -ENXIO;
	}
	else
	{

		printk("Device spi ready\n");
	}

	uint32_t max_read = 2;
	uint32_t max_write = 32;
	// uint8_t rxbuff[max_read];
	// uint8_t txbuff[max_write];
	uint8_t data = 0xAA;
	uint8_t tx_data[] = {0x00}; // 创建包含单个字节的数组
	uint8_t rx_data[] = {0x00}; // 创建包含单个字节的数组
	for (;;)
	{
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(1));
		spi_master_transfer(UB0_REG_DEVICE_CONFIG & ICM_WRITE_BIT_MASK, (uint8_t[]){0x00}, 1, NULL, 0);
		spi_master_transfer(WHO_AM_I_REG | ICM_READ_BIT_MASK, NULL, 0, rx_data, 1);
		printf("WHOAMI 0x%02x \n", rx_data[0]);
		// MGMT0 CONFIG
		spi_master_transfer(UB0_REG_PWR_MGMT0 & ICM_WRITE_BIT_MASK, (uint8_t[]){0x1F}, 1, NULL, 0); // UB0_REG_PWR_MGMT0 W 0x0F
		spi_master_transfer(UB0_REG_PWR_MGMT0 | ICM_READ_BIT_MASK, NULL, 0, rx_data, 1);
		printf("MGMT0 0x%02x \n", rx_data[0]);
		// ACC CONFIG
		spi_master_transfer(UB0_REG_ACCEL_CONFIG0 & ICM_WRITE_BIT_MASK, (uint8_t[]){0x00}, 1, NULL, 0); // UB0_REG_PWR_MGMT0 W 0x0F
		spi_master_transfer(UB0_REG_ACCEL_CONFIG0 | ICM_READ_BIT_MASK, NULL, 0, rx_data, 1);
		printf("ACLLEL_CONFIG 0x%02x \n", rx_data[0]);
		// GYRO CONFIG
		spi_master_transfer(UB0_REG_GYRO_CONFIG0 & ICM_WRITE_BIT_MASK, (uint8_t[]){0x00}, 1, NULL, 0); // UB0_REG_PWR_MGMT0 W 0x0F
		spi_master_transfer(UB0_REG_GYRO_CONFIG0 | ICM_READ_BIT_MASK, NULL, 0, rx_data, 1);
		printf("ACLLEL_CONFIG 0x%02x \n", rx_data[0]);
		k_msleep(1000);
		while (1)
		{
			spi_master_transfer(UB0_REG_TEMP_DATA1 | ICM_READ_BIT_MASK, NULL, 0, rx_data, 14);
			int16_t temp_raw = (int16_t)((rx_data[0] << 8) | rx_data[1]); // 将两个字节合并为一个 16 位整数
			double temp_c = temp_raw / 132.48 + 25;						  // 计算温度值，保持为浮点数以保持精度
			printf("TEMP= %.2f \n", temp_c);							  // 以浮点格式打印温度值			printf("ACCEL_DATA_X1 0x%02x \n", rx_data[2]);

			int16_t acc_x = (int16_t)((rx_data[2] << 8) | rx_data[3]);
			int16_t acc_y = (int16_t)((rx_data[4] << 8) | rx_data[5]);
			int16_t acc_z = (int16_t)((rx_data[6] << 8) | rx_data[7]);

			int16_t gyro_x = (int16_t)((rx_data[8] << 8) | rx_data[9]);
			int16_t gyro_y = (int16_t)((rx_data[10] << 8) | rx_data[11]);
			int16_t gyro_z = (int16_t)((rx_data[12] << 8) | rx_data[13]);
			printf("ACC_X= %d \n", acc_x);
			printf("ACC_Y= %d \n", acc_y);
			printf("ACC_Z= %d \n", acc_z);
			printf("GYRO_X= %d \n", gyro_x);
			printf("GYRO_Y= %d \n", gyro_y);
			printf("GYRO_Z= %d \n", gyro_z);
			k_msleep(500);
			/*
			int16_t acc_x = 0;
			int16_t acc_x_test = 0;
			// UB0_REG_ACCEL_DATA_X1 ACC HIGH
			spi_master_transfer(UB0_REG_ACCEL_DATA_X1 | ICM_READ_BIT_MASK, NULL, 0, rx_data, 1);
			printk("0x%02x \n", rx_data[0]);
			//  UB0_REG_ACCEL_DATA_X1 ACC LOW
			acc_x = rx_data[0] << 8;
			spi_master_transfer(UB0_REG_ACCEL_DATA_X0 | ICM_READ_BIT_MASK, NULL, 0, rx_data, 1);
			printk("0x%02x \n", rx_data[0]);
			acc_x |= rx_data[0];
			// caul acc
			printk("acc_x\t\t %d \n", acc_x);
			printf("====================================\n");
			spi_master_transfer(UB0_REG_ACCEL_DATA_X1 | ICM_READ_BIT_MASK, NULL, 0, rx_data, 2);
			printk("0x%02x \n", rx_data[0]);
			printk("0x%02x \n", rx_data[1]);
			printk("acc_x temp\t %d \n", (int16_t)((rx_data[0] << 8) | rx_data[1]));
			k_msleep(500);
			// spi_master_transfer(RESET_REG & ICM_WRITE_BIT_MASK, 0x01, 1, NULL, 0);
			// k_msleep(1000);
			*/
		}
		// spi_write_dt(&spispec, &tx_set);

		printk("\n");
	}
}
