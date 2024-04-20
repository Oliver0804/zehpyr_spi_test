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
#include <string.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/lbs.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define RUN_LED_BLINK_INTERVAL 1000

#define USER_LED DK_LED3

#define USER_BUTTON DK_BTN1_MSK

#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
static const struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);
#define WHO_AM_I_REG 0x75
#define WHO_AM_I_VAL 0x47
#define ICM_READ_BIT_MASK (0x80)
#define ICM_WRITE_BIT_MASK (0x7F)
static struct inv_icm207xx_serif icm207xx_serif;

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

	_send[0] = reg;
	if (tx_len)
	{
		memcpy(&_send[1], tx, tx_len);
	}

	struct spi_buf tx_spi_buf = {.buf = (void *)_send, .len = sizeof(reg) + tx_len};
	struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_buf = {.buf = (void *)_recv, .len = tx_spi_buf.len + rx_len};
	struct spi_buf_set rx_set = {.buffers = &rx_spi_buf, .count = 1};
	spi_transceive_dt(&spispec, &tx_set, &rx_set);

	if (rx_len)
	{
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
	icm207xx_serif.context = 0; /* no need */
	icm207xx_serif.read_reg = idd_io_hal_read_reg;
	icm207xx_serif.write_reg = idd_io_hal_write_reg;
	icm207xx_serif.max_read = 32;  /* maximum number of bytes allowed per serial read */
	icm207xx_serif.max_write = 32; /* maximum number of bytes allowed per serial write */
	icm207xx_serif.is_spi = 1;	   /* enable spi if */
	if (!device_is_ready(spispec.bus))
	{
		printk("Device spi not ready, aborting test");
		return -ENXIO;
	}
	else
	{
		printk("Device spi ready");
	}
	uint32_t max_read = 32;
	uint32_t max_write = 32;
	uint8_t rxbuff[max_read];
	for (;;)
	{
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(1));
		spi_master_transfer(WHO_AM_I_REG | ICM_READ_BIT_MASK, NULL, 0, rxbuff, 2);
		for (int i = 0; i < max_read; i++)
		{
			printk("0x%02x ", rxbuff[i]);
		}
		printk("\n");
	}
}
