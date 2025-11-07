/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic Mesh light sample
 */
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(chat, CONFIG_LOG_DEFAULT_LEVEL);

/* LED blink work for provisioning indication */
static struct k_work_delayable led_blink_work;
static int blink_count_remaining = 0;
static bool led_state = false;

static void led_blink_handler(struct k_work *work)
{
	if (blink_count_remaining > 0) {
		led_state = !led_state;
		dk_set_led(DK_LED1, led_state);
		
		if (!led_state) {
			blink_count_remaining--;
		}
		
		if (blink_count_remaining > 0) {
			k_work_reschedule(&led_blink_work, K_MSEC(200));
		}
	}
}

void mesh_led_blink(int count)
{
	blink_count_remaining = count;
	led_state = false;
	k_work_reschedule(&led_blink_work, K_MSEC(100));
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	printk("Provisioned! Address: 0x%04x\n", addr);
	mesh_led_blink(3);  // Blink 3 times when provisioned
}

static void prov_reset(void)
{
	printk("Provisioning reset\n");
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16];

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.complete = prov_complete,
	.reset = prov_reset,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	// Generate unique UUID from device address
	bt_addr_le_t addr;
	size_t count = 1;
	bt_id_get(&addr, &count);
	memcpy(dev_uuid, addr.a.val, 6);
	memcpy(dev_uuid + 6, "NordicChat", 10);

	err = dk_leds_init();
	if (err) {
		printk("Initializing LEDs failed (err %d)\n", err);
		return;
	}

	err = dk_buttons_init(NULL);
	if (err) {
		printk("Initializing buttons failed (err %d)\n", err);
		return;
	}

	k_work_init_delayable(&led_blink_work, led_blink_handler);

	err = bt_mesh_init(&prov, model_handler_init());
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

int main(void)
{
	int err;

	printk("Initializing...\n");

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	return 0;
}
