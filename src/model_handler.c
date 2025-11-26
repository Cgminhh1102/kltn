/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include "chat_cli.h"
#include "model_handler.h"
#include <stdlib.h>
#include <zephyr/logging/log.h>
// Extern DSDV routing table from chat_cli.c
extern struct dsdv_route_entry g_dsdv_routes[];
#define DSDV_ROUTE_TABLE_SIZE 64
LOG_MODULE_DECLARE(chat);

static const struct shell *chat_shell;

/******************************************************************************/
/*************************** LED Blink Function *******************************/
/******************************************************************************/
static struct k_work_delayable led_blink_work;
static int blink_count_remaining = 0;

static void led_blink_work_handler(struct k_work *work)
{
	if (blink_count_remaining > 0) {
		// Toggle LED
		static bool led_state = false;
		if (!led_state) {
			dk_set_led_on(DK_LED1);
			led_state = true;
			k_work_reschedule(&led_blink_work, K_MSEC(1000));
		} else {
			dk_set_led_off(DK_LED1);
			led_state = false;
			blink_count_remaining--;
			if (blink_count_remaining > 0) {
				k_work_reschedule(&led_blink_work, K_MSEC(1000));
			}
		}
	}
}

void mesh_led_blink(int count)
{
	if (count <= 0) {
		return;
	}
	blink_count_remaining = count;
	k_work_reschedule(&led_blink_work, K_MSEC(1000));
}

/******************************************************************************/
/*************************** Health server setup ******************************/
/******************************************************************************/
static struct bt_mesh_health_srv health_srv = {
	.cb = NULL,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/******************************************************************************/
/***************************** Chat model setup *******************************/
/******************************************************************************/
struct metrics_cache_entry
{
	uint16_t src_addr;
	uint16_t about_addr;
	struct bt_mesh_network_metrics last_metrics;
	uint32_t measurement_count;
	uint32_t last_latency_us;  // Store calculated latency
};
static struct metrics_cache_entry metrics_cache[16];  // 4x4 nodes max

// Relay metrics cache - stores metrics from intermediate nodes
struct relay_metrics_cache_entry
{
	uint16_t relay_addr;           // Address of relay node
	uint16_t original_src;         // Original packet source
	uint32_t original_seq;         // Original packet sequence
	struct bt_mesh_network_metrics relay_metrics;  // Full relay node metrics
	uint32_t timestamp;            // When received
	uint8_t measurement_count;     // How many times seen
};
static struct relay_metrics_cache_entry relay_cache[16];  // Store up to 16 relay metrics

static void store_relay_metrics_to_cache(uint16_t relay_addr, 
                                         uint16_t original_src,
                                         uint32_t original_seq,
                                         const struct bt_mesh_network_metrics *metrics)
{
    // Find existing entry for this relay node
    for (int i = 0; i < ARRAY_SIZE(relay_cache); i++) {
        if (relay_cache[i].relay_addr == relay_addr && 
            relay_cache[i].original_src == original_src) {
            // Update existing entry
            relay_cache[i].original_seq = original_seq;
            relay_cache[i].relay_metrics = *metrics;
            relay_cache[i].timestamp = k_uptime_get_32();
            relay_cache[i].measurement_count++;
            return;
        }
    }
    
    // Find empty slot
    for (int i = 0; i < ARRAY_SIZE(relay_cache); i++) {
        if (relay_cache[i].relay_addr == 0) {
            relay_cache[i].relay_addr = relay_addr;
            relay_cache[i].original_src = original_src;
            relay_cache[i].original_seq = original_seq;
            relay_cache[i].relay_metrics = *metrics;
            relay_cache[i].timestamp = k_uptime_get_32();
            relay_cache[i].measurement_count = 1;
            return;
        }
    }
    
    // Cache full - use LRU replacement (find oldest)
    int oldest_idx = 0;
    uint32_t oldest_time = relay_cache[0].timestamp;
    for (int i = 1; i < ARRAY_SIZE(relay_cache); i++) {
        if (relay_cache[i].timestamp < oldest_time) {
            oldest_idx = i;
            oldest_time = relay_cache[i].timestamp;
        }
    }
    
    // Replace oldest entry
    relay_cache[oldest_idx].relay_addr = relay_addr;
    relay_cache[oldest_idx].original_src = original_src;
    relay_cache[oldest_idx].original_seq = original_seq;
    relay_cache[oldest_idx].relay_metrics = *metrics;
    relay_cache[oldest_idx].timestamp = k_uptime_get_32();
    relay_cache[oldest_idx].measurement_count = 1;
}

static void store_metrics_to_cache(const struct bt_mesh_network_metrics *metrics)
{
    // Tìm hoặc tạo entry trong cache
    for (int i = 0; i < ARRAY_SIZE(metrics_cache); i++) {
        if (metrics_cache[i].src_addr == metrics->src_addr && 
            metrics_cache[i].about_addr == metrics->about_addr) {
            // Update existing entry
            metrics_cache[i].last_metrics = *metrics;
            metrics_cache[i].measurement_count++;
            return;
        }
    }
    
    // Tìm empty slot
    for (int i = 0; i < ARRAY_SIZE(metrics_cache); i++) {
        if (metrics_cache[i].src_addr == 0) {
            metrics_cache[i].src_addr = metrics->src_addr;
            metrics_cache[i].about_addr = metrics->about_addr;
            metrics_cache[i].last_metrics = *metrics;
            metrics_cache[i].measurement_count = 1;
            return;
        }
    }
}

static void update_latency_in_cache(uint16_t target_addr, uint32_t latency_us)
{
    for (int i = 0; i < ARRAY_SIZE(metrics_cache); i++) {
        if (metrics_cache[i].about_addr == target_addr) {
            metrics_cache[i].last_latency_us = latency_us;
            return;
        }
    }
}

static void print_client_status(void);

static void handle_chat_start(struct bt_mesh_chat_cli *chat)
{
    /* keep silent on start */
}
static void handle_network_metrics(struct bt_mesh_chat_cli *chat,
					struct bt_mesh_msg_ctx *ctx,
					const struct bt_mesh_network_metrics *metrics)
{
	// Calculate TTL-based hops for comparison
	uint8_t ttl_final = (ctx->recv_ttl <= metrics->initial_ttl) ? ctx->recv_ttl : metrics->initial_ttl;
	uint8_t ttl_hops = metrics->initial_ttl - ttl_final;
	
	// Get latency from cache if available
	uint32_t latency_ms = 0;
	for (int i = 0; i < ARRAY_SIZE(metrics_cache); i++) {
		if (metrics_cache[i].src_addr == metrics->src_addr) {
			latency_ms = metrics_cache[i].last_latency_us / 1000;
			break;
		}
	}
	
	// Print concise ASCII-only metrics
	shell_print(chat_shell, "NETWORK METRICS");
	shell_print(chat_shell, "From: 0x%04X  About: 0x%04X", metrics->src_addr, metrics->about_addr);
	shell_print(chat_shell, "TTL: %u/%u (TTL-hops: %u) | Congestion: %u | Req-ACK: %u",
				metrics->initial_ttl, ttl_final, ttl_hops, metrics->congestion, metrics->request_ack);
	shell_print(chat_shell, "Timestamp: %u ms | Method: %s",
				metrics->timestamp, (metrics->hop_count == ttl_hops) ? "Consistent" : "Incremental");
	
	// Display neighbor RSSI if available
	if (metrics->neighbor_count > 0) {
		shell_print(chat_shell, "Neighbors (%u):", metrics->neighbor_count);
		for (int i = 0; i < metrics->neighbor_count; i++) {
			shell_print(chat_shell, "  0x%04X: %d dBm", 
			            metrics->neighbor_rssi[i].addr,
			            metrics->neighbor_rssi[i].rssi);
		}
	}
	
	store_metrics_to_cache(metrics);
}					
static void handle_metrics_ack(struct bt_mesh_chat_cli *chat,
				  struct bt_mesh_msg_ctx *ctx,
				  const struct bt_mesh_metrics_ack *ack)
{
	// Find in cache
	uint32_t current_time = k_uptime_get_32();
	uint32_t rtt = current_time - ack->original_timestamp;
	uint32_t latency = rtt/2;
	shell_print(chat_shell,
			"[LATENCY] To 0x%04X: RTT %d ms, Estimated latency %d ms",
			ack->src_addr, rtt, latency);
	update_latency_in_cache(ack->src_addr, latency);
};

// Public function to store relay metrics (called from chat_cli.c)
void store_relay_metrics(uint16_t relay_addr, 
                        uint16_t original_src,
                        uint32_t original_seq,
                        const struct bt_mesh_network_metrics *metrics)
{
	store_relay_metrics_to_cache(relay_addr, original_src, original_seq, metrics);
	LOG_INF("Stored relay metrics from 0x%04x (src=0x%04x, seq=%u)", 
	        relay_addr, original_src, (unsigned)original_seq);
}

static const struct bt_mesh_chat_cli_handlers chat_handlers = {
	.start = handle_chat_start,
	.network_metrics = handle_network_metrics,
	.metrics_ack = handle_metrics_ack,
};

/* .. include_startingpoint_model_handler_rst_1 */
static struct bt_mesh_chat_cli chat = {
	.handlers = &chat_handlers,
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(
		1,
		BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub)),
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_CHAT_CLI(&chat))),
};
/* .. include_endpoint_model_handler_rst_1 */

static void print_client_status(void)
{
	if (!bt_mesh_is_provisioned()) {
		shell_print(chat_shell,
			    "The mesh node is not provisioned. Please provision the mesh node before using the chat.");
	} else {
		shell_print(chat_shell,
			    "The mesh node is provisioned. The client address is 0x%04x.",
			    bt_mesh_model_elem(chat.model)->rt->addr);
	}
}

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/******************************************************************************/
/******************************** Chat shell **********************************/
/******************************************************************************/
static int cmd_status(const struct shell *shell, size_t argc, char *argv[])
{
	print_client_status();

	return 0;
}

static int cmd_metrics_to(const struct shell *shell, size_t argc, char *argv[])
{
    uint16_t target_addr;
    int err;

    if (argc < 2) {
        shell_error(shell, "Usage: metrics_to <addr>");
        return -EINVAL;
    }

    target_addr = strtol(argv[1], NULL, 0);

    if (target_addr == 0 || target_addr > 0x7FFF) {
        shell_error(shell, "Invalid target address: 0x%04X", target_addr);
        return -EINVAL;
    }

    err = bt_mesh_chat_cli_metrics_send(&chat, target_addr);
    if (err) {
        shell_error(shell, "Failed to send metrics to 0x%04X: %d", target_addr, err);
        return err;
    }

    /* success: silent */
    return 0;
}

static int cmd_structure_to(const struct shell *shell, size_t argc, char *argv[])
{
    uint16_t target_addr;
    int err;

    if (argc < 2) {
        shell_error(shell, "Usage: structure_to <addr>");
        return -EINVAL;
    }

    target_addr = strtol(argv[1], NULL, 0);

    if (target_addr == 0 || target_addr > 0x7FFF) {
        shell_error(shell, "Invalid target address: 0x%04X", target_addr);
        return -EINVAL;
    }

    err = bt_mesh_chat_cli_structure_request(&chat, target_addr);
    if (err) {
        shell_error(shell, "Failed to request structure from 0x%04X: %d", target_addr, err);
        return err;
    }

    shell_print(shell, "Structure request sent to 0x%04X", target_addr);
    return 0;
}

static int cmd_show_routes(const struct shell *shell, size_t argc, char *argv[])
{
    shell_print(shell, "DSDV Routing Table:");
    shell_print(shell, "Dest   Next   Hops  Seq      Age(s)");
    shell_print(shell, "------------------------------------");
    
    uint32_t now = k_uptime_get_32();
    int count = 0;
    
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
        if (g_dsdv_routes[i].dest != 0) {
            uint32_t age_sec = (now - g_dsdv_routes[i].last_update_time) / 1000;
            shell_print(shell, "0x%04x 0x%04x %4u  %-8u %u",
                g_dsdv_routes[i].dest,
                g_dsdv_routes[i].next_hop,
                g_dsdv_routes[i].hop_count,
                (unsigned)g_dsdv_routes[i].seq_num,
                age_sec);
            count++;
        }
    }
    
    if (count == 0) {
        shell_print(shell, "(No routes available)");
    } else {
        shell_print(shell, "Total: %d routes", count);
    }
    return 0;
}

static int cmd_verify_route(const struct shell *shell, size_t argc, char *argv[])
{
    if (argc < 2) {
        shell_error(shell, "Usage: verify_route <dest_addr>");
        return -EINVAL;
    }
    
    uint16_t dest = strtol(argv[1], NULL, 0);
    
    // Find route
    struct dsdv_route_entry *route = NULL;
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
        if (g_dsdv_routes[i].dest == dest) {
            route = &g_dsdv_routes[i];
            break;
        }
    }
    
    if (!route) {
        shell_error(shell, "No route to 0x%04x", dest);
        return -ENOENT;
    }
    
    shell_print(shell, "Route to 0x%04x:", dest);
    shell_print(shell, "  Next hop: 0x%04x", route->next_hop);
    shell_print(shell, "  Hop count: %u", route->hop_count);
    shell_print(shell, "  Sequence: %u", (unsigned)route->seq_num);
    shell_print(shell, "  Age: %u seconds", 
                (k_uptime_get_32() - route->last_update_time) / 1000);
    shell_print(shell, "");
    shell_print(shell, "Now sending metrics to verify actual path...");
    
    // Send metrics để verify
    int err = bt_mesh_chat_cli_metrics_send(&chat, dest);
    if (err) {
        shell_error(shell, "Failed to send verification metrics: %d", err);
        return err;
    }
    
    shell_print(shell, "Check destination log for path vector to verify");
    shell_print(shell, "Expected first hop: 0x%04x", route->next_hop);
    
    return 0;
}

static int cmd_neighbor_rssi(const struct shell *shell, size_t argc, char **argv)
{
    uint16_t addrs[16];
    int8_t rssi[16];
    
    int count = bt_mesh_chat_cli_get_neighbor_rssi(addrs, rssi, 16);
    
    if (count == 0) {
        shell_print(shell, "No neighbor RSSI data available");
        return 0;
    }
    
    shell_print(shell, "Neighbor RSSI (last 60s):");
    shell_print(shell, "Address  | RSSI (dBm)");
    shell_print(shell, "---------|------------");
    
    for (int i = 0; i < count; i++) {
        shell_print(shell, "0x%04x   | %d", addrs[i], rssi[i]);
    }
    
    return 0;
}

static int cmd_clear_history(const struct shell *shell, size_t argc, char **argv)
{
    bt_mesh_chat_cli_clear_route_history();
    shell_print(shell, "Route metrics history cleared");
    return 0;
}

static int cmd_show_history(const struct shell *shell, size_t argc, char **argv)
{
    bt_mesh_chat_cli_show_route_history();
    return 0;
}

static int cmd_show_relay_metrics(const struct shell *shell, size_t argc, char **argv)
{
    uint8_t count = 0;
    
    // Count valid entries
    for (int i = 0; i < ARRAY_SIZE(relay_cache); i++) {
        if (relay_cache[i].relay_addr != 0) {
            count++;
        }
    }
    
    if (count == 0) {
        shell_print(shell, "No relay metrics available");
        return 0;
    }
    
    shell_print(shell, "\n=== RELAY METRICS CACHE (%u entries) ===", count);
    shell_print(shell, "");
    
    for (int i = 0; i < ARRAY_SIZE(relay_cache); i++) {
        if (relay_cache[i].relay_addr == 0) {
            continue;  // Skip empty slot
        }
        
        struct relay_metrics_cache_entry *entry = &relay_cache[i];
        struct bt_mesh_network_metrics *m = &entry->relay_metrics;
        
        // Calculate weak node percentage
        uint8_t weak_node_pct = 0;
        if (m->neighbor_count > 0) {
            uint8_t weak_count = 0;
            for (int j = 0; j < m->neighbor_count; j++) {
                if (m->neighbor_rssi[j].rssi < -80) {
                    weak_count++;
                }
            }
            weak_node_pct = (weak_count * 100) / m->neighbor_count;
        }
        
        // Calculate age
        uint32_t age_s = (k_uptime_get_32() - entry->timestamp) / 1000;
        
        shell_print(shell, "Relay Node: 0x%04x (src=0x%04x, seq=%u, count=%u, age=%us)",
                    entry->relay_addr, entry->original_src, 
                    (unsigned)entry->original_seq, entry->measurement_count, age_s);
        shell_print(shell, " Neighbors: %u | Weak: %u%% | Congestion: %u‰", m->neighbor_count, weak_node_pct, m->congestion);
        
        if (m->neighbor_count > 0) {
            shell_print(shell, "  Neighbors RSSI:");
            for (int j = 0; j < m->neighbor_count; j++) {
                shell_print(shell, "    0x%04x: %d dBm%s",
                            m->neighbor_rssi[j].addr,
                            m->neighbor_rssi[j].rssi,
                            (m->neighbor_rssi[j].rssi < -80) ? " (WEAK)" : "");
            }
        }
        shell_print(shell, "");
    }
    
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(chat_cmds,
	SHELL_CMD_ARG(status, NULL, "Print client status", cmd_status, 1, 0),
	SHELL_CMD_ARG(metrics_to, NULL, "Send metrics to specific node <addr> via DSDV",
		      cmd_metrics_to, 2, 0),
	SHELL_CMD_ARG(structure_to, NULL, "Request network structure from node <addr>",
		      cmd_structure_to, 2, 0),
	SHELL_CMD_ARG(routes, NULL,"Show DSDV routing table",cmd_show_routes, 1, 0),
	SHELL_CMD_ARG(verify_route, NULL, "Verify route to <addr> by sending metrics",
		      cmd_verify_route, 2, 0),
	SHELL_CMD_ARG(neighbors, NULL, "Show per-neighbor RSSI",
		      cmd_neighbor_rssi, 1, 0),
	SHELL_CMD_ARG(clear_history, NULL, "Clear route metrics history",
		      cmd_clear_history, 1, 0),
	SHELL_CMD_ARG(show_history, NULL, "Show route metrics comparison table",
		      cmd_show_history, 1, 0),
	SHELL_CMD_ARG(relay_metrics, NULL, "Show relay node metrics cache",
		      cmd_show_relay_metrics, 1, 0),
	SHELL_SUBCMD_SET_END
);

static int cmd_chat(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_help(shell);
		/* shell returns 1 when help is printed */
		return 1;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

SHELL_CMD_ARG_REGISTER(chat, &chat_cmds, "Bluetooth Mesh Chat Client commands",
		       cmd_chat, 1, 1);

/******************************************************************************/
/******************************** Public API **********************************/
/******************************************************************************/
const struct bt_mesh_comp *model_handler_init(void)
{
	chat_shell = shell_backend_uart_get_ptr();
	
	// Initialize LED blink work queue
	k_work_init_delayable(&led_blink_work, led_blink_work_handler);

	return &comp;
}
