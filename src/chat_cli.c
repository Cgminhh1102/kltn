/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/mesh.h>
#include "chat_cli.h"
#include "battery_adc.h"
#include "model_handler.h"
#include "mesh/net.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/random/random.h>
#define DSDV_ROUTE_TABLE_SIZE 64
#define DSDV_DUP_CACHE_SIZE 8
#define DSDV_ROUTE_TIMEOUT_MS 45000  // 45 seconds (3x UPDATE interval)
LOG_MODULE_DECLARE(chat);
static void log_tx_power_config(void);
static uint32_t total_reconfigurations = 0;
static uint32_t max_convergence_time_ms = 0;
static uint32_t min_convergence_time_ms = UINT32_MAX;
static int handle_network_metrics(const struct bt_mesh_model *model, 
                                 struct bt_mesh_msg_ctx *ctx,
                                 struct net_buf_simple *buf);

static int handle_dsdv_hello(const struct bt_mesh_model *model,
							 struct bt_mesh_msg_ctx *ctx,
							 struct net_buf_simple *buf);
static void dsdv_send_hello(struct k_work *work);
static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num, uint8_t flags);
static int handle_dsdv_update(const struct bt_mesh_model *model,
							  struct bt_mesh_msg_ctx *ctx,
							  struct net_buf_simple *buf);
static int handle_dsdv_data(const struct bt_mesh_model *model,
							struct bt_mesh_msg_ctx *ctx,
							struct net_buf_simple *buf);
struct dsdv_dup_cache{
	uint16_t src;
	uint32_t last_seq;
};
static struct dsdv_dup_cache g_dup_cache[DSDV_DUP_CACHE_SIZE];

static bool seen_duplicate(uint16_t src, uint32_t seq) {
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == src) {
            if (seq <= g_dup_cache[i].last_seq) return true;
            g_dup_cache[i].last_seq = seq;
            return false;
        }
    }
    // insert into first empty or rotate 0
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == 0) {
            g_dup_cache[i].src = src;
            g_dup_cache[i].last_seq = seq;
            return false;
        }
    }
    g_dup_cache[0].src = src;
    g_dup_cache[0].last_seq = seq;
    return false;
}
// Global variables để track metrics
static int8_t last_received_rssi = -40;  // Mạnh hơn cho nodes gần nhau
static uint16_t current_target_node = 0x0000;
struct dsdv_route_entry g_dsdv_routes[DSDV_ROUTE_TABLE_SIZE];
static uint32_t g_dsdv_my_seq = 1;
static struct k_work_delayable dsdv_hello_work;
static struct k_work_delayable dsdv_update_work;
static bool dsdv_route_changed = false; // flag để trigger update
static uint32_t last_route_change_time = 0;
static uint32_t first_route_change_time = 0;
static bool dsdv_network_stable = true;
static uint32_t convergence_time = 0;
static struct k_work_delayable stability_check_work;
// Per-neighbor RSSI tracking (more accurate than global)
#define MAX_RSSI_NEIGHBORS 16
static struct {
    uint16_t addr;
    int8_t rssi;
    uint32_t last_update;
} neighbor_rssi[MAX_RSSI_NEIGHBORS];

// Congestion tracking with sliding window (reset every 60s)
#define CONGESTION_WINDOW_MS 60000
static struct {
    uint16_t total_sends;
    uint16_t failed_sends;
    uint32_t window_start;
} congestion_stats = {0};

// Packet delivery tracking
static struct {
    uint32_t packets_sent;
    uint32_t packets_acked;
    uint32_t window_start;
} delivery_stats = {0};

// Route metrics history for comparison (stores last 5 routes)
#define MAX_ROUTE_HISTORY 5
static struct {
    uint16_t src_addr;
    uint32_t timestamp;
    uint32_t latency_ms;
    uint8_t hop_count;
    uint8_t battery_pct;
    uint8_t weak_node_pct;
    uint16_t congestion;
} route_history[MAX_ROUTE_HISTORY];
static uint8_t route_history_count = 0;

// Forward declarations
static inline int8_t rssi_ewma(int8_t prev, int8_t now);
static uint8_t calculate_ttl(struct dsdv_route_entry *route);
static struct dsdv_route_entry* find_best_route(uint16_t dest);

// Find best route with RSSI-first priority
// Strategy: Prioritize strong signal (RSSI) to minimize retransmissions,
//           then use hop count as tiebreaker for equivalent RSSI
static struct dsdv_route_entry* find_best_route(uint16_t dest)
{
    struct dsdv_route_entry *best = NULL;
    float best_score = 1000.0f;  // Initialize high (lower is better)
    uint32_t now = k_uptime_get_32();
    
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
        struct dsdv_route_entry *route = &g_dsdv_routes[i];
        
        // Skip invalid or expired routes
        if (route->dest != dest || route->dest == 0) {
            continue;
        }
        if ((now - route->last_update_time) > DSDV_ROUTE_TIMEOUT_MS) {
            continue;
        }
        
        // === RSSI-FIRST SCORING (lower score = better) ===
        
        // 1. RSSI Score (PRIMARY - 70% weight)
        //    Strong signal reduces retransmissions and errors
        //    -100 dBm (weak) → score 1.0 (bad)
        //    -30 dBm (strong) → score 0.0 (good)
        float rssi_norm = (route->rssi + 100.0f) / 70.0f;
        if (rssi_norm < 0.0f) rssi_norm = 0.0f;
        if (rssi_norm > 1.0f) rssi_norm = 1.0f;
        float rssi_score = 1.0f - rssi_norm;  // Invert: strong = low score
        
        // 2. Hop Count Score (SECONDARY - 30% weight)
        //    Used as tiebreaker when RSSI is similar
        //    Fewer hops reduce latency and resource consumption
        //    0 hops → score 0.0 (best)
        //    15 hops → score 1.0 (worst)
        float hop_score = route->hop_count / 15.0f;
        
        // 3. Weighted Total Score: 70% RSSI + 30% Hop Count
        //    This ensures RSSI dominates, but hop count matters for close calls
        float total_score = (rssi_score * 0.7f) + (hop_score * 0.3f);
        
        // Pick route with LOWEST (best) score
        if (total_score < best_score) {
            best = route;
            best_score = total_score;
        }
    }
    
    return best;
}

// Calculate optimal TTL based on route hop count
static uint8_t calculate_ttl(struct dsdv_route_entry *route)
{
    if (!route) {
        // No route: use network diameter estimate
        uint8_t max_hops = 0;
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
            if (g_dsdv_routes[i].dest != 0 && 
                g_dsdv_routes[i].hop_count > max_hops) {
                max_hops = g_dsdv_routes[i].hop_count;
            }
        }
        uint8_t ttl = max_hops + 2;  // Diameter + margin
        return (ttl < 5) ? 5 : (ttl > 15 ? 15 : ttl);
    }
    
    // Known route: hop_count + safety margin
    uint8_t ttl = route->hop_count + 2;
    
    // Clamp to valid range [3, 15]
    if (ttl < 3) ttl = 3;
    if (ttl > 15) ttl = 15;
    
    return ttl;
}

static struct dsdv_route_entry* find_route(uint16_t dest){
	for(int i = 0; i<DSDV_ROUTE_TABLE_SIZE; ++i){
		if(g_dsdv_routes[i].dest == dest){
			return &g_dsdv_routes[i];
		}
	}
	return NULL;
}

static void dsdv_cleanup_expired_routes(void) {
	uint32_t now = k_uptime_get_32();
	int removed = 0;
	
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
		if (g_dsdv_routes[i].dest != 0) {
			uint32_t age = now - g_dsdv_routes[i].last_update_time;
			if (age > DSDV_ROUTE_TIMEOUT_MS) {
				g_dsdv_routes[i].dest = 0;  // Mark as empty
				removed++;
			}
		}
	}

    if (removed > 0) {
        LOG_INF("DSDV: Removed %d expired routes", removed);
        if (dsdv_network_stable)
        {
            first_route_change_time = now;
            dsdv_network_stable = false;
        }
        last_route_change_time = now;
    }
}

// Update per-neighbor RSSI (more accurate than global RSSI)
static void update_neighbor_rssi(uint16_t addr, int8_t rssi)
{
    uint32_t now = k_uptime_get_32();
    int8_t old_rssi = rssi;
    
    // Find existing entry or oldest slot
    int oldest_idx = 0;
    uint32_t oldest_time = neighbor_rssi[0].last_update;
    
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            // Apply EWMA to existing entry
            old_rssi = neighbor_rssi[i].rssi;
            neighbor_rssi[i].rssi = rssi_ewma(old_rssi, rssi);
            neighbor_rssi[i].last_update = now;
            return;
        }
        if (neighbor_rssi[i].last_update < oldest_time) {
            oldest_idx = i;
            oldest_time = neighbor_rssi[i].last_update;
        }
    }
    
    // New neighbor - use oldest slot
    neighbor_rssi[oldest_idx].addr = addr;
    neighbor_rssi[oldest_idx].rssi = rssi;
    neighbor_rssi[oldest_idx].last_update = now;
}

// Get RSSI for specific neighbor (0 if not found)
static int8_t get_neighbor_rssi(uint16_t addr)
{
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            return neighbor_rssi[i].rssi;
        }
    }
    return -127; // No data
}

// Reset congestion stats if window expired
static void check_congestion_window(void)
{
    uint32_t now = k_uptime_get_32();
    if (congestion_stats.window_start == 0) {
        congestion_stats.window_start = now;
    } else if (now - congestion_stats.window_start > CONGESTION_WINDOW_MS) {
        // Reset window
        congestion_stats.total_sends = 0;
        congestion_stats.failed_sends = 0;
        congestion_stats.window_start = now;
    }
}

// Reset delivery stats if window expired
static void check_delivery_window(void)
{
    uint32_t now = k_uptime_get_32();
    if (delivery_stats.window_start == 0) {
        delivery_stats.window_start = now;
    } else if (now - delivery_stats.window_start > CONGESTION_WINDOW_MS) {
        delivery_stats.packets_sent = 0;
        delivery_stats.packets_acked = 0;
        delivery_stats.window_start = now;
    }
}

// Get battery percentage from ADC
static uint8_t get_battery_percentage(void)
{
#ifdef CONFIG_ADC
    // Use real ADC measurement
    return battery_get_percentage();
#else
    // Fallback: Simulate battery drain based on uptime
    uint32_t uptime_min = k_uptime_get_32() / 60000;
    uint8_t battery = 100;
    
    // Simulate 1% drain per 10 minutes
    if (uptime_min < 1000) {
        battery = 100 - (uptime_min / 10);
    } else {
        battery = 0;
    }
    
    return battery > 100 ? 85 : battery;
#endif
}

static inline int8_t rssi_ewma(int8_t prev, int8_t now) {
    // alpha = 1/8 (EWMA smoothing: 7/8 old + 1/8 new)
    int32_t acc = (7 * (int32_t)prev + (int32_t)now) / 8;
    if (acc > 127) acc = 127;
    if (acc < -128) acc = -128;
    return (int8_t)acc;
}

static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num, uint8_t flags) {
    struct dsdv_route_entry *e = find_route(dest);
    uint32_t now = k_uptime_get_32();
    int8_t rssi = get_neighbor_rssi(next_hop);  // Get RSSI for route quality
    bool route_changed = false;

    if(!e) {
        route_changed = true;
        LOG_DBG("Route change: New route to 0x%04x via 0x%04x", dest, next_hop);
    } else if(next_hop != e->next_hop || hop_count != e->hop_count) {
        route_changed = true;
        LOG_DBG("Route change: 0x%04x changed next_hop 0x%04x->0x%04x hops %u->%u", 
                dest, e->next_hop, next_hop, e->hop_count, hop_count);
    }
    if (!e) {
        // tìm slot trống
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
            if (g_dsdv_routes[i].dest == 0) {
                g_dsdv_routes[i] = (struct dsdv_route_entry){
                    .dest = dest, 
                    .next_hop = next_hop, 
                    .hop_count = hop_count, 
                    .flags = flags, 
                    .seq_num = seq_num,
                    .last_update_time = now,
                    .rssi = rssi
                };
                return;
            }
        }
        // nếu đầy, ghi đè slot 0 (đơn giản; có thể dùng LRU sau)
        g_dsdv_routes[0] = (struct dsdv_route_entry){
            .dest = dest, 
            .next_hop = next_hop, 
            .hop_count = hop_count, 
            .flags = flags, 
            .seq_num = seq_num,
            .last_update_time = now,
            .rssi = rssi
        };
        return;
    } else if (seq_num > e->seq_num || (seq_num == e->seq_num && hop_count < e->hop_count)) {
        e->next_hop = next_hop;
        e->hop_count = hop_count;
        e->flags = flags;
        e->seq_num = seq_num;
        e->last_update_time = now;
        e->rssi = rssi;  // Update RSSI
    }
    
    if(route_changed) {
        if(dsdv_network_stable) {
            first_route_change_time = now;
            dsdv_network_stable = false;
        }
        last_route_change_time = now;
    }
}

void check_network_stability(void){
    if(dsdv_network_stable) {
        return;
    }

    uint32_t current_time = k_uptime_get_32();
    uint32_t time_since_last_change = current_time - last_route_change_time;
    if(time_since_last_change > 30000){
        convergence_time = last_route_change_time - first_route_change_time;
        dsdv_network_stable = true;
        total_reconfigurations++;
        if (convergence_time > max_convergence_time_ms) {
            max_convergence_time_ms = convergence_time;
        }
        if (convergence_time < min_convergence_time_ms) {
            min_convergence_time_ms = convergence_time;
        }
        
        LOG_INF("=== NETWORK CONVERGED ===");
        LOG_INF("Convergence time: %u ms", convergence_time);
        LOG_INF("Total reconfigurations: %u", total_reconfigurations);
        LOG_INF("Min/Max: %u / %u ms", min_convergence_time_ms, max_convergence_time_ms);
    }
}

static void stability_check_handler(struct k_work *work) {
    uint32_t now = k_uptime_get_32();
    uint32_t idle_time = now - last_route_change_time;
    
    LOG_DBG("Stability check: idle=%u ms, stable=%d", idle_time, dsdv_network_stable);
    
    check_network_stability();
    
    // Reschedule mỗi 5 giây
    k_work_reschedule(&stability_check_work, K_MSEC(5000));
}
// Thêm work queue cho metrics
static struct k_work_delayable metrics_send_work;
static struct bt_mesh_chat_cli *metrics_chat_instance = NULL;
static struct bt_mesh_chat_cli *g_chat_cli_instance = NULL;
static void collect_current_metrics(struct bt_mesh_chat_cli *chat, 
								   struct bt_mesh_network_metrics *metrics);

static void collect_current_metrics(struct bt_mesh_chat_cli *chat, struct bt_mesh_network_metrics *metrics)
{
	uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
	metrics->src_addr = my_addr;
	metrics->about_addr = current_target_node;
	metrics->timestamp = k_uptime_get_32();
	
	// Calculate TTL dynamically based on BEST route
	struct dsdv_route_entry *route = find_best_route(current_target_node);
	metrics->initial_ttl = calculate_ttl(route);
	metrics->request_ack = 1;

	// Get real battery percentage (or simulated)
	metrics->battery_pct = get_battery_percentage();
	
	// Get per-neighbor RSSI (more accurate than global)
	int8_t rssi = get_neighbor_rssi(current_target_node);
	metrics->rssi_dbm = (rssi == -127) ? last_received_rssi : rssi;
	
	if (!chat->model || !chat->model->rt) {
		return;
	}
	
	// Calculate congestion with sliding window
	check_congestion_window();
	if (congestion_stats.total_sends > 0) {
		metrics->congestion = (congestion_stats.failed_sends * 1000) / congestion_stats.total_sends;
	} else {
		metrics->congestion = 0;
	}
	
	metrics->hop_count = 1;  // Start at 1 (source to first hop)
	
	// Collect all neighbor RSSI values
	uint32_t now = k_uptime_get_32();
	metrics->neighbor_count = 0;
	
	for (int i = 0; i < MAX_RSSI_NEIGHBORS && metrics->neighbor_count < MAX_NEIGHBORS_IN_METRICS; i++) {
		// Only include recent neighbors (< 60s old)
		if (neighbor_rssi[i].addr != 0 && 
		    (now - neighbor_rssi[i].last_update) < 60000) {
			metrics->neighbor_rssi[metrics->neighbor_count].addr = neighbor_rssi[i].addr;
			metrics->neighbor_rssi[metrics->neighbor_count].rssi = neighbor_rssi[i].rssi;
			metrics->neighbor_count++;
		}
	}
	
	LOG_DBG("Metrics: battery=%u%%, RSSI=%ddBm (node 0x%04x), congestion=%u‰, neighbors=%u", 
	        metrics->battery_pct, metrics->rssi_dbm, current_target_node, 
	        metrics->congestion, metrics->neighbor_count);
}

static void metrics_work_handler(struct k_work *work)
{
    if (!metrics_chat_instance) {
        LOG_WRN("No metrics_chat_instance available");
        return;
    }
    
    struct bt_mesh_network_metrics metrics;
    collect_current_metrics(metrics_chat_instance, &metrics);
    uint16_t my_addr = bt_mesh_model_elem(metrics_chat_instance->model)->rt->addr;
	uint16_t dest = current_target_node;
	if(dest == 0 || dest == my_addr){
		LOG_WRN("No valid target node for metrics (0x%04X)", dest);
		return;
	}
	
	// Use BEST route selection based on multi-criteria
	struct dsdv_route_entry *route = find_best_route(dest);
	if (!route) {
		LOG_WRN("No route to target node 0x%04X for metrics", dest);
		return;
	}

	struct dsdv_data_packet pkt = {
		.src = my_addr,
		.dest = dest,
		.seq_num = k_uptime_get_32(), // dùng timestamp làm seq
		.hop_count = 1,  // Start at 1 for direct send
		.path_len = 1,
		.metrics = metrics,  // Source metrics
		.collect_relay_metrics = 1,  // Request relay metrics
	};
	pkt.path_nodes[0] = my_addr; // path bắt đầu từ chính nó

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
							 BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);
	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
	net_buf_simple_add_mem(&msg, &pkt, sizeof(pkt));

	struct bt_mesh_msg_ctx ctx = {
		.addr = route->next_hop,
		.app_idx = metrics_chat_instance->model->keys[0],
		.send_ttl = calculate_ttl(route),
		.send_rel = true,
	};

	// Track packet delivery
	check_delivery_window();
	delivery_stats.packets_sent++;
	
	// Track congestion
	check_congestion_window();
	congestion_stats.total_sends++;

	int ret = bt_mesh_model_send(metrics_chat_instance->model, &ctx, &msg, NULL, NULL);
	if (ret != 0) {
		congestion_stats.failed_sends++;
		LOG_WRN("Failed to send DSDV data to 0x%04x via 0x%04x, ret=%d", 
		        dest, route->next_hop, ret);
	}
	// LOG_DBG("dsdv data to 0x%04x via 0x%04x, ret=%d", dest, route->next_hop, ret);
	
}

static void dsdv_send_hello(struct k_work *work){
	if (!g_chat_cli_instance||!g_chat_cli_instance->model||!g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_hello_work, K_MSEC(1000));
		return;
	}

	// Cleanup expired routes periodically
	dsdv_cleanup_expired_routes();

	g_dsdv_my_seq += 2;
	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;

	struct dsdv_hello hello = {
		.src = my_addr,
		.seq_num = g_dsdv_my_seq,
		.flags = 0
	};

	struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;
	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_HELLO);
	net_buf_simple_add_mem(pub, &hello, sizeof(hello));
	(void)bt_mesh_model_publish(g_chat_cli_instance->model);

	// LOG_DBG("DSDV: Sent HELLO seq=%u", (unsigned)g_dsdv_my_seq);

	uint32_t jitter = sys_rand32_get() % 500;
	k_work_reschedule(&dsdv_hello_work, K_MSEC(5000 + jitter));

}

static void dsdv_send_update(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model || !g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_update_work, K_MSEC(2000));
		return;
	}

	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;
	uint8_t num_entries = 0;
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE && num_entries < 16; ++i)
	{
		if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].dest != my_addr) {
			num_entries++;
		}
	}
	if (num_entries == 0)
	{
		k_work_reschedule(&dsdv_update_work, K_MSEC(10000));
		return;
	}
	
	struct dsdv_update_header hdr = {
		.src = my_addr,
		.num_entries = num_entries,
		.flags = 0
	};

	struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;
	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_UPDATE);
	net_buf_simple_add_mem(pub, &hdr, sizeof(hdr));

	uint8_t added = 0;
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE && added < num_entries; ++i) {
		if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].dest != my_addr) {
			const struct dsdv_route_entry *entry = &g_dsdv_routes[i];
			struct dsdv_route_entry tmp = *entry; // copy
			if (tmp.hop_count < UINT8_MAX) {
				tmp.hop_count++; // chỉ tăng trong PAYLOAD
			}
			net_buf_simple_add_mem(pub, &tmp, sizeof(tmp));
			added++;
		}
	}

	(void)bt_mesh_model_publish(g_chat_cli_instance->model);
	// LOG_DBG("dsdv: sent update with %u entries", num_entries);
	dsdv_route_changed = false; // reset flag
	uint32_t jitter = sys_rand32_get() % 1000;
	k_work_reschedule(&dsdv_update_work, K_MSEC(15000 + jitter));
}

/** @brief Log current TX power configuration for debugging */
static void log_tx_power_config(void)
{
#ifdef CONFIG_BT_CTLR_TX_PWR_MINUS_20
    LOG_INF("TX Power: -20 dBm (minimum range ~1-2m for linear topology)");
#elif defined(CONFIG_BT_CTLR_TX_PWR_MINUS_16)
    LOG_INF("TX Power: -16 dBm (very low range ~2-3m)");
#elif defined(CONFIG_BT_CTLR_TX_PWR_MINUS_12)
    LOG_INF("TX Power: -12 dBm (low range ~3-5m)");
#elif defined(CONFIG_BT_CTLR_TX_PWR_MINUS_8)
    LOG_INF("TX Power: -8 dBm (medium range ~5-8m)");
#elif defined(CONFIG_BT_CTLR_TX_PWR_0)
    LOG_INF("TX Power: 0 dBm (default range ~10-15m)");
#elif defined(CONFIG_BT_CTLR_TX_PWR_PLUS_4)
    LOG_INF("TX Power: +4 dBm (maximum range ~15-20m)");
#else
    LOG_INF("TX Power: Unknown configuration");
#endif
}

static void send_metrics_ack(struct bt_mesh_chat_cli *chat,
                            struct bt_mesh_msg_ctx *ctx,
                            const struct bt_mesh_network_metrics *original_metrics)
{
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK,
                             BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK);
    bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK);
    
    struct bt_mesh_metrics_ack ack = {
        .src_addr = original_metrics->src_addr,
        .original_timestamp = original_metrics->timestamp,
        .padding = 0
    };
    
    net_buf_simple_add_mem(&msg, &ack, sizeof(ack));
    
    (void)bt_mesh_model_send(chat->model, ctx, &msg, NULL, NULL);
}

static int handle_metrics_ack(const struct bt_mesh_model *model, 
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;
    struct bt_mesh_metrics_ack ack;
    
    memcpy(&ack, net_buf_simple_pull_mem(buf, sizeof(ack)), sizeof(ack));
    
    if (chat->handlers->metrics_ack) {
        chat->handlers->metrics_ack(chat, ctx, &ack);
    }
    
    return 0;
}

static int handle_dsdv_hello(const struct bt_mesh_model *model,
							 struct bt_mesh_msg_ctx *ctx,
							 struct net_buf_simple *buf)
{
	struct dsdv_hello hello;
	if (buf->len < sizeof(hello)){
		return -EINVAL;
	}
	memcpy(&hello, net_buf_simple_pull_mem(buf, sizeof(hello)), sizeof(hello));
	
	uint16_t neighbor = ctx->addr;
	uint16_t dest = hello.src;
	uint32_t seq = hello.seq_num;
	
	// Update neighbor RSSI from HELLO packet
	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}
	
	dsdv_upsert(dest, neighbor, 1, seq, 0);
	// LOG_DBG("dsdv hello from 0x%04x seq=%u->route to 0x%04x via 0x%04x hop=1",
	//         neighbor, (unsigned)seq, dest, neighbor);
	return 0;
}

static int handle_dsdv_update(const struct bt_mesh_model *model,
							  struct bt_mesh_msg_ctx *ctx,
							  struct net_buf_simple *buf)
{
	
	struct dsdv_update_header hdr;
	if (buf->len < sizeof(hdr)){
		return -EINVAL;
	}
	memcpy(&hdr, net_buf_simple_pull_mem(buf, sizeof(hdr)), sizeof(hdr));
	uint16_t neighbor = ctx->addr;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
	
	// Update neighbor RSSI from UPDATE packet
	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}
	
	// LOG_DBG("dsdv update from 0x%04x with %u entries", neighbor, hdr.num_entries);
	for (int i = 0; i < hdr.num_entries; ++i)
	{
		if (buf->len < sizeof(struct dsdv_route_entry))
		{
			break;
		}
		struct dsdv_route_entry entry;
		memcpy(&entry, net_buf_simple_pull_mem(buf, sizeof(entry)), sizeof(entry));
		if (entry.dest == my_addr)
		{
			continue; // ignore route to self
		}
		dsdv_upsert(entry.dest, neighbor, entry.hop_count, entry.seq_num, entry.flags);
	}
	return 0;
}

static int handle_dsdv_data(const struct bt_mesh_model *model,
                           struct bt_mesh_msg_ctx *ctx,
                           struct net_buf_simple *buf)
{
    mesh_led_blink(1);  // Blink LED on packet receive
    
    struct bt_mesh_chat_cli *chat = model->rt->user_data;
    uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

    if (buf->len < sizeof(struct dsdv_data_packet)) {
        LOG_ERR("DSDV DATA too short");
        return -EINVAL;
    }

    struct dsdv_data_packet pkt;
    memcpy(&pkt, net_buf_simple_pull_mem(buf, sizeof(pkt)), sizeof(pkt));
	
	// Kiểm tra xem mình có phải đích không
    if (pkt.dest == my_addr) {
        // DESTINATION: Accept all packets to see all routes (no duplicate check)
        uint32_t current_time = k_uptime_get_32();
        
        LOG_INF("=== RECEIVED METRICS AT DESTINATION ===");
        LOG_INF("Source: 0x%04x, Relay count: %u", pkt.src, pkt.hop_count);
        if (pkt.collect_relay_metrics) {
            LOG_INF("(Relay metrics will arrive separately)");
        }
        
        LOG_INF("Path vector:");
        for (int i = 0; i < pkt.path_len && i < MAX_PATH_NODES; ++i) {
            LOG_INF("  [%d] 0x%04x%s", i, pkt.path_nodes[i],
                    (i == 0) ? " (source)" : 
                    (i == pkt.path_len-1) ? " (destination)" : " (relay)");
        }
        
        // *** Display source metrics ***
        LOG_INF("");
        LOG_INF("=== SOURCE NODE METRICS ===");
        
        struct bt_mesh_network_metrics *m = &pkt.metrics;
        
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
        
        // Header
        LOG_INF("+--------+---------+----------+-------+-----------+");
        LOG_INF("| Node 0x%04x: Battery=%3u%%, Neighbors=%2u, Weak=%3u%%, Congestion=%4u%% |",
                m->src_addr,
                m->battery_pct,
                m->neighbor_count,
                weak_node_pct,
                m->congestion);
        LOG_INF("+--------+---------+----------+-------+-----------+");
        
        // Show neighbor RSSI
        if (m->neighbor_count > 0) {
            LOG_INF("| Neighbors RSSI:                                    |");
            for (int j = 0; j < m->neighbor_count; j++) {
                LOG_INF("|   → 0x%04x: %4d dBm %-28s|",
                        m->neighbor_rssi[j].addr,
                        m->neighbor_rssi[j].rssi,
                        (m->neighbor_rssi[j].rssi < -80) ? "(WEAK)" : "");
            }
        } else {
            LOG_INF("| No neighbors detected                              |");
        }
        LOG_INF("+----------------------------------------------------+");
        
        // Calculate latency
        uint32_t latency_ms = (current_time > m->timestamp) ? 
                             (current_time - m->timestamp) : 0;
        LOG_INF("");
        LOG_INF("End-to-end latency: %u ms", latency_ms);
        
        // Store in route history
        if (route_history_count < MAX_ROUTE_HISTORY) {
            route_history[route_history_count].src_addr = pkt.src;
            route_history[route_history_count].timestamp = current_time;
            route_history[route_history_count].latency_ms = latency_ms;
            route_history[route_history_count].hop_count = pkt.hop_count;
            route_history[route_history_count].battery_pct = m->battery_pct;
            route_history[route_history_count].weak_node_pct = weak_node_pct;
            route_history[route_history_count].congestion = m->congestion;
            route_history_count++;
        } else {
            // Shift history and add new entry
            for (int i = 0; i < MAX_ROUTE_HISTORY - 1; i++) {
                route_history[i] = route_history[i + 1];
            }
            route_history[MAX_ROUTE_HISTORY - 1].src_addr = pkt.src;
            route_history[MAX_ROUTE_HISTORY - 1].timestamp = current_time;
            route_history[MAX_ROUTE_HISTORY - 1].latency_ms = latency_ms;
            route_history[MAX_ROUTE_HISTORY - 1].hop_count = pkt.hop_count;
            route_history[MAX_ROUTE_HISTORY - 1].battery_pct = m->battery_pct;
            route_history[MAX_ROUTE_HISTORY - 1].weak_node_pct = weak_node_pct;
            route_history[MAX_ROUTE_HISTORY - 1].congestion = m->congestion;
        }

        // Gọi handler ứng dụng
        if (chat->handlers->network_metrics) {
            chat->handlers->network_metrics(chat, ctx, &pkt.metrics);
        }

        // CHỈ NODE ĐÍCH GỬI ACK
        if (pkt.metrics.request_ack == 1) {
            send_metrics_ack(chat, ctx, &pkt.metrics);
            LOG_INF("Sent ACK as destination");
        }

        return 0;
    }

    // INTERMEDIATE NODE (relay): Check duplicate to prevent loops
    if (seen_duplicate(pkt.src, pkt.seq_num)) {
        LOG_DBG("Drop duplicate relay DATA from 0x%04x seq=%u", pkt.src, (unsigned)pkt.seq_num);
        return 0;
    }

    // *** NEW: If collect_relay_metrics flag is set, send relay metrics separately ***
    if (pkt.collect_relay_metrics == 1) {
        struct bt_mesh_network_metrics my_metrics;
        current_target_node = pkt.dest;
        collect_current_metrics(chat, &my_metrics);
        
        // Find DSDV route to destination
        struct dsdv_route_entry *relay_route = find_best_route(pkt.dest);
        if (!relay_route) {
            LOG_WRN("Relay 0x%04x: No route to dest 0x%04x for metrics", my_addr, pkt.dest);
        } else {
            // Send relay metrics packet via DSDV route
            struct relay_metrics_packet relay_pkt = {
                .original_src = pkt.src,
                .original_seq = pkt.seq_num,
                .relay_addr = my_addr,
                .relay_metrics = my_metrics
            };
            
            BT_MESH_MODEL_BUF_DEFINE(relay_msg, BT_MESH_CHAT_CLI_OP_RELAY_METRICS,
                                     BT_MESH_CHAT_CLI_MSG_LEN_RELAY_METRICS);
            bt_mesh_model_msg_init(&relay_msg, BT_MESH_CHAT_CLI_OP_RELAY_METRICS);
            net_buf_simple_add_mem(&relay_msg, &relay_pkt, sizeof(relay_pkt));
            
            struct bt_mesh_msg_ctx relay_ctx = {
                .addr = relay_route->next_hop,  // Gửi qua DSDV next_hop (không trực tiếp)
                .app_idx = chat->model->keys[0],
                .send_ttl = calculate_ttl(relay_route),  // TTL dựa trên route
                .send_rel = false,  // Best effort delivery
            };
            
            int ret = bt_mesh_model_send(chat->model, &relay_ctx, &relay_msg, NULL, NULL);
            if (ret == 0) {
                LOG_DBG("Relay 0x%04x sent metrics via 0x%04x (hops=%u) to dest 0x%04x", 
                        my_addr, relay_route->next_hop, relay_route->hop_count, pkt.dest);
            } else {
                LOG_WRN("Failed to send relay metrics: %d", ret);
            }
        }
    }

    // Không phải đích: forward đến next_hop qua BEST route
    struct dsdv_route_entry *route = find_best_route(pkt.dest);
    if (!route) {
        LOG_WRN("No route to forward packet from 0x%04x to 0x%04x", 
                pkt.src, pkt.dest);
        return 0;
    }

    // Tăng hop_count và thêm mình vào path
    pkt.hop_count++;
    if (pkt.path_len < MAX_PATH_NODES) {
        pkt.path_nodes[pkt.path_len++] = my_addr;
    }
    
    BT_MESH_MODEL_BUF_DEFINE(fwd, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
                             BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);
    bt_mesh_model_msg_init(&fwd, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
    net_buf_simple_add_mem(&fwd, &pkt, sizeof(pkt));

    struct bt_mesh_msg_ctx fwd_ctx = {
        .addr = route->next_hop,
        .app_idx = chat->model->keys[0],
        .send_ttl = calculate_ttl(route),
        .send_rel = true,
    };

    (void)bt_mesh_model_send(chat->model, &fwd_ctx, &fwd, NULL, NULL);
    // LOG_DBG("Forwarded DATA to 0x%04x via 0x%04x", pkt.dest, route->next_hop);
    return 0;
}
/** @brief Relay metrics with incremented hop count */
static int relay_network_metrics(const struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct net_buf_simple *buf)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;
    struct bt_mesh_network_metrics metrics;
    uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
    
    // Extract metrics from received message
    memcpy(&metrics, buf->data, sizeof(metrics));
    
    // Check if this message is about us (we are the target)
    bool is_about_me = (metrics.about_addr == my_addr) || 
                       (metrics.about_addr == 0x0000); // Broadcast
    
    if (is_about_me) {
        // We are the target - process as final destination
        return handle_network_metrics(model, ctx, buf);
    }
    
    // We are a relay - increment hop count and forward
    metrics.hop_count++;
    // LOG_DBG("Relaying metrics: hop_count incremented to %d (from 0x%04x to 0x%04x)", 
    //     metrics.hop_count, metrics.src_addr, metrics.about_addr);
    
    // Re-publish the modified metrics
    BT_MESH_MODEL_BUF_DEFINE(relay_buf, BT_MESH_CHAT_CLI_OP_NETWORK_METRICS,
                             BT_MESH_CHAT_CLI_MSG_LEN_NETWORK_METRICS);
    bt_mesh_model_msg_init(&relay_buf, BT_MESH_CHAT_CLI_OP_NETWORK_METRICS);
    net_buf_simple_add_mem(&relay_buf, &metrics, sizeof(metrics));
    
    // Forward using publish
    struct net_buf_simple *pub = chat->model->pub->msg;
	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_NETWORK_METRICS);
	net_buf_simple_add_mem(pub, &metrics, sizeof(metrics));
	int ret = bt_mesh_model_publish(chat->model);
	if (ret != 0) {
		LOG_WRN("Failed to relay metrics, ret=%d", ret);
	}
	// LOG_DBG("Metrics relayed, ret=%d", ret);
    return 0;
}
static int handle_network_metrics(const struct bt_mesh_model *model, 
                                 struct bt_mesh_msg_ctx *ctx,
                                 struct net_buf_simple *buf)
{
    mesh_led_blink(1);  // Blink LED on packet receive
    
    struct bt_mesh_chat_cli *chat = model->rt->user_data;
    struct bt_mesh_network_metrics metrics;
    
    // Extract metrics
    memcpy(&metrics, net_buf_simple_pull_mem(buf, sizeof(metrics)), sizeof(metrics));
    
    // Update per-neighbor RSSI with EWMA
    if (ctx->recv_rssi != 0) {
        update_neighbor_rssi(ctx->addr, ctx->recv_rssi);
        last_received_rssi = rssi_ewma(last_received_rssi, ctx->recv_rssi);
        // LOG_DBG("RSSI updated(EWMA) to %d dBm from metrics message", last_received_rssi);
    }
    
    // Calculate hop count tại receiver
    if (metrics.hop_count > 10) {
        // high hop count (silent)
    }
    
    // Call application handler
    if (chat->handlers->network_metrics) {
        chat->handlers->network_metrics(chat, ctx, &metrics);
    }
    
    // Send ACK if requested
    if (metrics.request_ack == 1) {
		uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
		if (metrics.about_addr == my_addr) {
			send_metrics_ack(chat, ctx, &metrics);
			// LOG_DBG("sent ack as destination node");
		}
    }
    
    return 0;
}

/** @brief Handle relay metrics packet at destination */
static int handle_relay_metrics(const struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct net_buf_simple *buf)
{
    struct relay_metrics_packet relay_pkt;
    
    if (buf->len < sizeof(relay_pkt)) {
        LOG_ERR("RELAY_METRICS too short");
        return -EINVAL;
    }
    
    memcpy(&relay_pkt, net_buf_simple_pull_mem(buf, sizeof(relay_pkt)), sizeof(relay_pkt));
    
    LOG_INF("");
    LOG_INF("=== RELAY NODE METRICS ===");
    LOG_INF("From relay 0x%04x (for packet src=0x%04x, seq=%u)",
            relay_pkt.relay_addr, relay_pkt.original_src, (unsigned)relay_pkt.original_seq);
    
    struct bt_mesh_network_metrics *m = &relay_pkt.relay_metrics;
    
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
    
    // Header
    LOG_INF("+--------+---------+----------+-------+-----------+");
    LOG_INF("| Node 0x%04x: Battery=%3u%%, Neighbors=%2u, Weak=%3u%%, Congestion=%4u‰ |",
            m->src_addr,
            m->battery_pct,
            m->neighbor_count,
            weak_node_pct,
            m->congestion);
    LOG_INF("+--------+---------+----------+-------+-----------+");
    
    // Show neighbor RSSI
    if (m->neighbor_count > 0) {
        LOG_INF("| Neighbors RSSI:                                    |");
        for (int j = 0; j < m->neighbor_count; j++) {
            LOG_INF("|   → 0x%04x: %4d dBm %-28s|",
                    m->neighbor_rssi[j].addr,
                    m->neighbor_rssi[j].rssi,
                    (m->neighbor_rssi[j].rssi < -80) ? "(WEAK)" : "");
        }
    } else {
        LOG_INF("| No neighbors detected                              |");
    }
    LOG_INF("+----------------------------------------------------+");
    
    return 0;
}

/** @brief Handle network structure request at destination - Print routing table locally */
static int handle_structure_request(const struct bt_mesh_model *model,
                                    struct bt_mesh_msg_ctx *ctx,
                                    struct net_buf_simple *buf)
{
    struct structure_request_packet req;
    
    if (buf->len < sizeof(req)) {
        return -EINVAL;
    }
    
    memcpy(&req, net_buf_simple_pull_mem(buf, sizeof(req)), sizeof(req));
    uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
    
    // Count valid routes
    uint8_t total_nodes = 0;
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
        if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].dest != my_addr) {
            total_nodes++;
        }
    }
    
    // Print simple table
    LOG_INF("\n=== NETWORK STRUCTURE (Node 0x%04X) ===", my_addr);
    LOG_INF("Request from: 0x%04X | Total nodes: %u", req.requester_addr, total_nodes);
    
    if (total_nodes == 0) {
        LOG_INF("No routes available");
        return 0;
    }
    
    LOG_INF("--------------------------------------------------");
    LOG_INF(" Node    Next-Hop  Hops  RSSI(dBm)  Quality");
    LOG_INF("--------------------------------------------------");
    
    // Print all routes
    uint8_t printed = 0;
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE && printed < total_nodes; i++) {
        if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].dest != my_addr) {
            const char *quality;
            int8_t rssi = g_dsdv_routes[i].rssi;
            
            if (rssi >= -50) {
                quality = "Excellent";
            } else if (rssi >= -65) {
                quality = "Good";
            } else if (rssi >= -80) {
                quality = "Fair";
            } else {
                quality = "Weak";
            }
            
            LOG_INF(" 0x%04X  0x%04X    %2u    %4d       %s",
                    g_dsdv_routes[i].dest,
                    g_dsdv_routes[i].next_hop,
                    g_dsdv_routes[i].hop_count,
                    rssi,
                    quality);
            printed++;
        }
    }
    
    LOG_INF("--------------------------------------------------\n");
    
    return 0;
}

/** @brief Handle convergence statistics request - Print convergence stats locally */
static int handle_convergence_request(const struct bt_mesh_model *model,
                                      struct bt_mesh_msg_ctx *ctx,
                                      struct net_buf_simple *buf)
{
    struct convergence_request_packet req;
    
    if (buf->len < sizeof(req)) {
        return -EINVAL;
    }
    
    memcpy(&req, net_buf_simple_pull_mem(buf, sizeof(req)), sizeof(req));
    uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
    uint32_t current_time = k_uptime_get_32();
    
    LOG_INF("\n=== CONVERGENCE STATISTICS (Node 0x%04X) ===", my_addr);
    LOG_INF("Request from: 0x%04X", req.requester_addr);
    LOG_INF("--------------------------------------------------");
    
    // Current network state
    LOG_INF("Network State: %s", dsdv_network_stable ? "STABLE" : "CONVERGING");
    
    if (!dsdv_network_stable) {
        uint32_t elapsed = current_time - first_route_change_time;
        LOG_INF("Currently converging for: %u ms", elapsed);
    }
    
    // Statistics
    LOG_INF("--------------------------------------------------");
    LOG_INF("Total Reconfigurations: %u", total_reconfigurations);
    LOG_INF("Last Convergence Time:  %u ms", convergence_time);
    
    if (total_reconfigurations > 0) {
        LOG_INF("Min Convergence Time:   %u ms", min_convergence_time_ms);
        LOG_INF("Max Convergence Time:   %u ms", max_convergence_time_ms);
        
        // Calculate average if we have data
        if (total_reconfigurations > 0) {
            // Note: For accurate average, would need to store all convergence times
            LOG_INF("(Average calculation requires storing all convergence times)");
        }
    } else {
        LOG_INF("No reconfigurations recorded yet");
    }
    
    LOG_INF("--------------------------------------------------");
    
    // Routing table size
    uint8_t total_routes = 0;
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
        if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].dest != my_addr) {
            total_routes++;
        }
    }
    LOG_INF("Current Routing Table Size: %u nodes", total_routes);
    LOG_INF("--------------------------------------------------\n");
    
    return 0;
}

/* .. include_startingpoint_chat_cli_rst_2 */
const struct bt_mesh_model_op _bt_mesh_chat_cli_op[] = {
	{
		BT_MESH_CHAT_CLI_OP_METRICS_ACK,
		BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK),
		handle_metrics_ack
	},
	{
		BT_MESH_CHAT_CLI_OP_NETWORK_METRICS,
		BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_NETWORK_METRICS),
		relay_network_metrics
	},
	{
		BT_MESH_CHAT_CLI_OP_DSDV_HELLO,
		BT_MESH_LEN_EXACT(sizeof(struct dsdv_hello)),
		handle_dsdv_hello
	},
	{
		BT_MESH_CHAT_CLI_OP_DSDV_UPDATE,
		BT_MESH_LEN_MIN(BT_MESH_CHAT_CLI_MSG_LEN_DSDV_UPDATE_MIN),
		handle_dsdv_update
	},
	{
		BT_MESH_CHAT_CLI_OP_DSDV_DATA,
		BT_MESH_LEN_MIN(sizeof(struct dsdv_data_packet)),
		handle_dsdv_data
	},
	{
		BT_MESH_CHAT_CLI_OP_RELAY_METRICS,
		BT_MESH_LEN_MIN(sizeof(struct relay_metrics_packet)),
		handle_relay_metrics
	},
	{
		BT_MESH_CHAT_CLI_OP_STRUCTURE_REQUEST,
		BT_MESH_LEN_MIN(sizeof(struct structure_request_packet)),
		handle_structure_request
	},
	{
		BT_MESH_CHAT_CLI_OP_CONVERGENCE_REQUEST,
		BT_MESH_LEN_MIN(sizeof(struct convergence_request_packet)),
		handle_convergence_request
	},
	BT_MESH_MODEL_OP_END,
};
/* .. include_endpoint_chat_cli_rst_2 */

/* .. include_startingpoint_chat_cli_rst_3 */
#ifdef CONFIG_BT_SETTINGS
static int bt_mesh_chat_cli_settings_set(const struct bt_mesh_model *model,
					 const char *name,
					 size_t len_rd,
					 settings_read_cb read_cb,
					 void *cb_arg)
{
	/* No settings to store */
	if (name) {
		return -ENOENT;
	}

	return 0;
}
#endif
/* .. include_endpoint_chat_cli_rst_3 */

/* .. include_startingpoint_chat_cli_rst_4 */
static int bt_mesh_chat_cli_init(const struct bt_mesh_model *model)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;

    // LOG_DBG("Chat CLI model initializing...");
    chat->model = model;

    net_buf_simple_init_with_data(&chat->pub_msg, chat->buf,
                      sizeof(chat->buf));
    chat->pub.msg = &chat->pub_msg;
    chat->pub.update = NULL;  // No periodic updates
    k_work_init_delayable(&metrics_send_work, metrics_work_handler);
    k_work_init_delayable(&dsdv_hello_work, dsdv_send_hello);
	k_work_init_delayable(&dsdv_update_work, dsdv_send_update);
    
    // Initialize battery ADC
#ifdef CONFIG_ADC
    int err = battery_adc_init();
    if (err) {
        LOG_WRN("Battery ADC init failed: %d, will use simulated values", err);
    } else {
        LOG_INF("Battery ADC initialized successfully");
    }
#endif
    
    log_tx_power_config();
    // LOG_DBG("Chat CLI model initialized successfully");
    return 0;
}
/* .. include_endpoint_chat_cli_rst_4 */

/* .. include_startingpoint_chat_cli_rst_5 */
static int bt_mesh_chat_cli_start(const struct bt_mesh_model *model)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;

    LOG_INF("Chat model started");
    
    // Check app key binding
    if (model->keys[0] == BT_MESH_KEY_UNUSED) {
        LOG_WRN("No App Key bound to chat model!");
    }

    if (chat->handlers->start) {
        chat->handlers->start(chat);
    }
	g_chat_cli_instance = chat;
	k_work_schedule(&dsdv_hello_work, K_MSEC(1500+ (sys_rand32_get() % 500)));
	k_work_schedule(&dsdv_update_work, K_MSEC(3000 + (sys_rand32_get() % 1000)));
    k_work_init_delayable(&stability_check_work, stability_check_handler);
    k_work_schedule(&stability_check_work, K_MSEC(5000));
    return 0;
}
/* .. include_endpoint_chat_cli_rst_5 */

/* .. include_startingpoint_chat_cli_rst_7 */
const struct bt_mesh_model_cb _bt_mesh_chat_cli_cb = {
	.init = bt_mesh_chat_cli_init,
	.start = bt_mesh_chat_cli_start,
#ifdef CONFIG_BT_SETTINGS
	.settings_set = bt_mesh_chat_cli_settings_set,
#endif
};
/* .. include_endpoint_chat_cli_rst_7 */

/* .. include_startingpoint_chat_cli_rst_8 */
void bt_mesh_chat_cli_set_metrics_target(uint16_t target_addr)
{
	current_target_node = target_addr;
}

int bt_mesh_chat_cli_get_neighbor_rssi(uint16_t *addrs, int8_t *rssi, int max_count)
{
    int count = 0;
    uint32_t now = k_uptime_get_32();
    
    for (int i = 0; i < MAX_RSSI_NEIGHBORS && count < max_count; i++) {
        if (neighbor_rssi[i].addr != 0 && 
            (now - neighbor_rssi[i].last_update) < 60000) { // Only recent (< 60s)
            addrs[count] = neighbor_rssi[i].addr;
            rssi[count] = neighbor_rssi[i].rssi;
            count++;
        }
    }
    
    return count;
}

int bt_mesh_chat_cli_metrics_send(struct bt_mesh_chat_cli *chat, uint16_t dest)
{
    if (!chat || !chat->model || !chat->model->rt) {
        return -EINVAL;
    }

    uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
    if (dest == 0 || dest == my_addr) {
        return -EINVAL;
    }

    // Select BEST route using multi-criteria (hop_count + RSSI)
    struct dsdv_route_entry *route = find_best_route(dest);
    if (!route) {
        // Follow DSDV strictly: do not fallback publish
        return -ENOENT;
    }

    // Log chi tiết route được chọn khi gửi metrics
    LOG_INF("=== SENDING METRICS ===");
    LOG_INF("Source: 0x%04x → Destination: 0x%04x", my_addr, dest);
    LOG_INF("Selected Route: next_hop=0x%04x | route_hops=%u | RSSI=%ddBm | TTL=%u (RSSI-first)",
            route->next_hop, route->hop_count, route->rssi, calculate_ttl(route));
    LOG_INF("Packet will start with hop_count=0 (incremented at each relay)");

    // Use collect_current_metrics to avoid code duplication
    current_target_node = dest;
    struct bt_mesh_network_metrics metrics;
    collect_current_metrics(chat, &metrics);

    struct dsdv_data_packet pkt = {
        .src = my_addr,
        .dest = dest,
        .seq_num = k_uptime_get_32(),
        .hop_count = 1,
        .path_len = 1,
        .metrics = metrics,  // Source metrics only
        .collect_relay_metrics = 1,  // Request relay metrics
    };
    pkt.path_nodes[0] = my_addr;

    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
                             BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);
    bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
    net_buf_simple_add_mem(&msg, &pkt, sizeof(pkt));

    struct bt_mesh_msg_ctx ctx = {
        .addr = route->next_hop,
        .app_idx = chat->model->keys[0],
        .send_ttl = calculate_ttl(route),
        .send_rel = true,
    };

    // Track delivery and congestion stats
    check_delivery_window();
    delivery_stats.packets_sent++;
    
    check_congestion_window();
    congestion_stats.total_sends++;
    
    int ret = bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
    if (ret != 0) {
        congestion_stats.failed_sends++;
    }
    
    return ret;
}

void bt_mesh_chat_cli_clear_route_history(void)
{
    route_history_count = 0;
    memset(route_history, 0, sizeof(route_history));
    LOG_INF("Route metrics history cleared");
}

void bt_mesh_chat_cli_show_route_history(void)
{
    if (route_history_count == 0) {
        LOG_INF("No route metrics history available");
        return;
    }
    
    LOG_INF("=== ROUTE METRICS TABLE ===");
    LOG_INF("+---------+----------+-----+------------+------------+------------+");
    LOG_INF("|         | Latency  |     |   Avg.     |   Weak     |            |");
    LOG_INF("|         |   (ms)   | Hop | Energy (%%) |  Node (%%) | Congestion |");
    LOG_INF("+---------+----------+-----+------------+------------+------------+");
    
    for (int i = 0; i < route_history_count; i++) {
        LOG_INF("| Route %d |     %4u |  %2u |        %3u |        %3u |       %4u |",
                i + 1,
                route_history[i].latency_ms,
                route_history[i].hop_count,
                route_history[i].battery_pct,
                route_history[i].weak_node_pct,
                route_history[i].congestion);
    }
    
    LOG_INF("+---------+----------+-----+------------+------------+------------+");
}

int bt_mesh_chat_cli_structure_request(struct bt_mesh_chat_cli *chat, uint16_t dest)
{
    if (!chat || !chat->model || !chat->model->rt) {
        return -EINVAL;
    }

    uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
    if (dest == 0 || dest == my_addr) {
        return -EINVAL;
    }

    struct dsdv_route_entry *route = find_best_route(dest);
    if (!route) {
        return -ENOENT;
    }

    struct structure_request_packet req = {
        .requester_addr = my_addr,
        .request_seq = k_uptime_get_32(),
    };

    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_STRUCTURE_REQUEST,
                             sizeof(struct structure_request_packet));
    bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_STRUCTURE_REQUEST);
    net_buf_simple_add_mem(&msg, &req, sizeof(req));

    struct bt_mesh_msg_ctx ctx = {
        .addr = route->next_hop,
        .app_idx = chat->model->keys[0],
        .send_ttl = calculate_ttl(route),
        .send_rel = true,
    };

    return bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
}

int bt_mesh_chat_cli_convergence_request(struct bt_mesh_chat_cli *chat, uint16_t dest)
{
    if (!chat || !chat->model || !chat->model->rt) {
        return -EINVAL;
    }

    uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
    if (dest == 0 || dest == my_addr) {
        return -EINVAL;
    }

    struct dsdv_route_entry *route = find_best_route(dest);
    if (!route) {
        return -ENOENT;
    }

    struct convergence_request_packet req = {
        .requester_addr = my_addr,
        .request_seq = k_uptime_get_32(),
    };

    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_CONVERGENCE_REQUEST,
                             sizeof(struct convergence_request_packet));
    bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_CONVERGENCE_REQUEST);
    net_buf_simple_add_mem(&msg, &req, sizeof(req));

    struct bt_mesh_msg_ctx ctx = {
        .addr = route->next_hop,
        .app_idx = chat->model->keys[0],
        .send_ttl = calculate_ttl(route),
        .send_rel = true,
    };

    return bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
}
