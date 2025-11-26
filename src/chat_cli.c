/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file chat_cli.c
 * @brief DSDV (Destination-Sequenced Distance-Vector) Routing Implementation for BLE Mesh
 * Protocol Overview:
 * 1. HELLO packets (5s interval): Neighbor discovery and direct link quality
 * 2. UPDATE packets (15s interval): Route table exchange and propagation
 * 3. DATA packets: End-to-end data delivery with path tracking and relay metrics
 * 4. Route timeout: 45s (3x UPDATE interval) to handle node mobility
 */

#include <zephyr/bluetooth/mesh.h>
#include "chat_cli.h"
#include "model_handler.h"
#include "mesh/net.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/random/random.h>


#define DSDV_ROUTE_TABLE_SIZE 64

/** Maximum number of route entries per UPDATE packet (balance between completeness and packet size) */
#define MAX_UPDATE_ENTRIES 32

/** Size of duplicate packet detection cache to prevent routing loops.
 * Increased to handle more simultaneous sources in 20-node test network.
 */
#define DSDV_DUP_CACHE_SIZE 24

/** Route expiration timeout - routes older than this are removed (30 seconds = 2x UPDATE interval) */
#define DSDV_ROUTE_TIMEOUT_MS 30000

/** Maximum number of neighbors to track RSSI for */
#define MAX_RSSI_NEIGHBORS 16

/** Neighbor RSSI validity window (milliseconds) - how long a neighbor
 *  is considered "recent" for UPDATE acceptance and metrics.
 *  30000ms ≈ 6 HELLO cycles at 5s interval.
 */
#define NEIGHBOR_RSSI_VALID_WINDOW_MS 30000

/** Congestion measurement window duration (60 seconds) */
#define CONGESTION_WINDOW_MS 60000

/** Maximum number of historical routes to store */
#define MAX_ROUTE_HISTORY 5

LOG_MODULE_DECLARE(chat);

/* =========================================================================
 * PROTOCOL HANDLER DECLARATIONS
 * ============================================================================ */

/** Handle HELLO packet - used for neighbor discovery and RSSI tracking */
static int handle_dsdv_hello(const struct bt_mesh_model *model,  /** Con trỏ đến model Mesh hiện tại. **/
							 struct bt_mesh_msg_ctx *ctx,   /** Ngữ cảnh tin nhắn Mesh. **/
							 struct net_buf_simple *buf);   /** Dữ liệu gói tin. **/

/** Periodic work handler - sends HELLO packets every 5s to neighbors */
static void dsdv_send_hello(struct k_work *work);

/** Update or insert a route in the routing table (core DSDV logic) */
static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num);

/** Handle UPDATE packet - receives routing table from neighbors */
static int handle_dsdv_update(const struct bt_mesh_model *model,
							  struct bt_mesh_msg_ctx *ctx,
							  struct net_buf_simple *buf);

/** Handle DATA packet - forwards or processes end-to-end data */
static int handle_dsdv_data(const struct bt_mesh_model *model,
							struct bt_mesh_msg_ctx *ctx,
							struct net_buf_simple *buf);

/* ============================================================================
 * DUPLICATE DETECTION - Prevent routing loops
 * ============================================================================ */

/** Cache entry for tracking seen packets from each source */
struct dsdv_dup_cache{
	uint16_t src;        /**< Source address of the packet */
	uint32_t last_seq;   /**< Highest sequence number seen from this source */
};

/** Global duplicate detection cache */
static struct dsdv_dup_cache g_dup_cache[DSDV_DUP_CACHE_SIZE];

/**
 * @brief Check if a packet is a duplicate (already seen)
 * 
 * Uses sequence number comparison to detect duplicates. Packets with
 * sequence numbers <= last seen are considered duplicates.
 * 
 * @param src Source address of the packet
 * @param seq Sequence number of the packet
 * @return true if packet is duplicate, false if new
 */
static bool seen_duplicate(uint16_t src, uint32_t seq) {
    // Check existing entries for this source
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == src) {
            if (seq <= g_dup_cache[i].last_seq) return true;  // Duplicate detected
            g_dup_cache[i].last_seq = seq;  // Update to newer sequence
            return false;
        }
    }
    
    // New source - find empty slot or reuse oldest (slot 0)
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == 0) {
            g_dup_cache[i].src = src;
            g_dup_cache[i].last_seq = seq;
            return false;
        }
    }
    
    // Cache full - overwrite slot 0 (simple FIFO replacement)
    g_dup_cache[0].src = src;
    g_dup_cache[0].last_seq = seq;
    return false;
}

/* ============================================================================
 * ROUTING TABLE AND DSDV STATE
 * ============================================================================ */

/** Current target node for metrics collection */
static uint16_t current_target_node = 0x0000;

/** Main DSDV routing table - stores all known routes in the network */
struct dsdv_route_entry g_dsdv_routes[DSDV_ROUTE_TABLE_SIZE];

/** This node's sequence number - incremented by 2 for each HELLO (even numbers only) */
static uint32_t g_dsdv_my_seq = 1;

/** Flag indicating whether any route changed (triggers UPDATE) */
static bool dsdv_route_changed = false;



/* ============================================================================
 * PER-NEIGHBOR RSSI TRACKING - More accurate than global RSSI
 * ============================================================================ */

/** Per-neighbor RSSI tracking structure */
static struct {
    uint16_t addr;         /**< Neighbor node address */
    int8_t rssi;           /**< Smoothed RSSI value (dBm) using EWMA */
    uint32_t last_update;  /**< Last update timestamp (k_uptime_get_32) */
} neighbor_rssi[MAX_RSSI_NEIGHBORS];

/* ============================================================================
 * CONGESTION TRACKING - Sliding window statistics
 * ============================================================================ */

/** Congestion statistics - tracks send success/failure rate */
static struct {
    uint16_t total_sends;   /**< Total packets sent in current window */
    uint16_t failed_sends;  /**< Failed packet sends in current window */
    uint32_t window_start;  /**< Window start timestamp */
} congestion_stats = {0};

/* ============================================================================
 * PACKET DELIVERY TRACKING - Monitor end-to-end delivery
 * ============================================================================ */

/** Packet delivery statistics */
static struct {
    uint32_t packets_sent;  /**< Total packets sent */
    uint32_t packets_acked; /**< Packets acknowledged by destination */
    uint32_t window_start;  /**< Window start timestamp */
} delivery_stats = {0};

/* ============================================================================
 * ROUTE HISTORY - Store metrics from recent routes for comparison
 * ============================================================================ */

/** Route metrics history for performance analysis */
static struct {
    uint16_t src_addr;        /**< Source address of the route */
    uint32_t timestamp;       /**< When metrics were collected */
    uint32_t latency_ms;      /**< End-to-end latency */
    uint8_t hop_count;        /**< Number of hops */
    uint8_t weak_node_pct;    /**< Percentage of weak neighbors (<-80dBm) */
    uint16_t congestion;      /**< Congestion level (‰) */
} route_history[MAX_ROUTE_HISTORY];

/** Current number of entries in route history */
static uint8_t route_history_count = 0;

/* ============================================================================
 * WORK QUEUES - Delayed work handlers
 * ============================================================================ */

/** Delayed work for periodic HELLO transmission (every ~5s) */
static struct k_work_delayable dsdv_hello_work;

/** Delayed work for periodic UPDATE transmission (every ~15s) */
static struct k_work_delayable dsdv_update_work;

/** Work queue for periodic metrics transmission */
static struct k_work_delayable metrics_send_work;

/* ============================================================================
 * MODEL INSTANCES - Chat CLI model pointers
 * ============================================================================ */

/** Global chat CLI instance (for HELLO/UPDATE workers) */
static struct bt_mesh_chat_cli *g_chat_cli_instance = NULL;

/** Chat instance for metrics operations */
static struct bt_mesh_chat_cli *metrics_chat_instance = NULL;

/* ============================================================================
 * UTILITY FUNCTION DECLARATIONS
 * ============================================================================ */

/** Apply Exponential Weighted Moving Average to RSSI values for smoothing */
static inline int8_t rssi_ewma(int8_t prev, int8_t now);

/** Calculate optimal TTL (Time-To-Live) based on route hop count */
static uint8_t calculate_ttl(struct dsdv_route_entry *route);

/** Get RSSI value for specific neighbor */
static int8_t get_neighbor_rssi(uint16_t addr);

/* ============================================================================
 * TTL CALCULATION - Optimize packet lifetime
 * ============================================================================ */

/**
 * @brief Calculate optimal TTL (Time-To-Live) for a packet
 * 
 * TTL determines how many hops a packet can travel before being discarded.
 * Too low = packet doesn't reach destination
 * Too high = wastes network resources on failed deliveries
 * 
 * Strategy:
 * - If route known: hop_count + 2 (safety margin for minor topology changes)
 * - If no route: estimate from network diameter (max hops seen) + margin
 * - Clamp to valid range [3, 15] to prevent extremes
 * 
 * @param route Pointer to route entry (NULL if no route known)
 * @return Optimal TTL value (3-15)
 */
static uint8_t calculate_ttl(struct dsdv_route_entry *route)
{
    if (!route) {
        uint8_t max_hops = 0;
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
            if (g_dsdv_routes[i].dest != 0 && 
                g_dsdv_routes[i].hop_count > max_hops) {
                max_hops = g_dsdv_routes[i].hop_count;
            }
        }
        uint8_t ttl = max_hops + 2;  // Network diameter + safety margin
        return (ttl < 5) ? 5 : (ttl > 15 ? 15 : ttl);
    }
    
    // Route known: use hop_count + safety margin
    uint8_t ttl = route->hop_count + 2;
    
    // Clamp to valid BLE Mesh range [3, 15]
    if (ttl < 3) ttl = 3;
    if (ttl > 15) ttl = 15;
    
    return ttl;
}

/* ============================================================================
 * ROUTE TABLE MANAGEMENT
 * ============================================================================ */

/**
 * @brief Find valid (non-expired) route to destination in routing table
 * 
 * Searches routing table for a route to the destination, automatically
 * filtering out expired routes (older than 45 seconds).
 * 
 * This unified function serves two purposes:
 * 1. Route existence check in dsdv_upsert() - expired routes treated as non-existent
 * 2. Route lookup for sending packets - ensures only valid routes are used
 * 
 * @param dest Destination address to find
 * @return Pointer to valid route entry, or NULL if not found or expired
 */
static struct dsdv_route_entry* find_route(uint16_t dest){
	uint32_t now = k_uptime_get_32();
	
	for(int i = 0; i<DSDV_ROUTE_TABLE_SIZE; ++i){
		if(g_dsdv_routes[i].dest == dest){
			// Check if route has expired (> 45 seconds old)
			if ((now - g_dsdv_routes[i].last_update_time) > DSDV_ROUTE_TIMEOUT_MS) {
				return NULL;  // Expired route = no valid route
			}
			return &g_dsdv_routes[i];
		}
	}
	return NULL;
}

/**
 * @brief Clean up expired routes from routing table
 * 
 * DSDV requires automatic route timeout to handle topology changes:
 * - Node leaving network (no more UPDATEs received)
 * - Link break (next_hop becomes unreachable)
 * 
 * Routes expire after DSDV_ROUTE_TIMEOUT_MS (45 seconds = 3x UPDATE interval)
 * This allows network to adapt when nodes join/leave or move.
 */
static void dsdv_cleanup_expired_routes(void) {
	uint32_t now = k_uptime_get_32();
	
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
		if (g_dsdv_routes[i].dest != 0) {
			uint32_t age_ms = now - g_dsdv_routes[i].last_update_time;
			
			// Only remove expired routes (> 30 seconds old)
            if (age_ms > DSDV_ROUTE_TIMEOUT_MS) {
                /* Instead of silent removal, first propagate invalidation (odd seq + hop_count=0xFF)
                 * so neighbors can drop stale routes faster. Next periodic cleanup will clear if still unused. */
                if (g_dsdv_routes[i].hop_count != 0xFF) {
                    g_dsdv_routes[i].hop_count = 0xFF;
                    if ((g_dsdv_routes[i].seq_num & 1) == 0) {
                        g_dsdv_routes[i].seq_num += 1; // make sequence odd (invalidation)
                    }
                    dsdv_route_changed = true; // trigger fast UPDATE to advertise invalidation
                    g_dsdv_routes[i].last_update_time = now; // refresh timestamp to defer removal
                } else {
                    // Already invalidated previously, now clear fully
                    g_dsdv_routes[i].dest = 0;
                    g_dsdv_routes[i].next_hop = 0;
                    g_dsdv_routes[i].hop_count = 0;
                    g_dsdv_routes[i].seq_num = 0;
                    g_dsdv_routes[i].last_update_time = 0;
                }
            }
		}
	}
}

/* ============================================================================
 * RSSI TRACKING - Per-neighbor signal strength monitoring
 * ============================================================================ */

/**
 * @brief Update RSSI value for a specific neighbor with exponential smoothing
 * 
 * Maintains per-neighbor RSSI history with EWMA filtering to reduce noise.
 * More accurate than using a single global RSSI value.
 * 
 * Uses LRU (Least Recently Used) replacement when table is full.
 * 
 * @param addr Neighbor node address
 * @param rssi New RSSI measurement (dBm)
 */
static void update_neighbor_rssi(uint16_t addr, int8_t rssi)
{
    uint32_t now = k_uptime_get_32();
    int8_t old_rssi = rssi;
    
    // Find existing entry or track oldest slot for LRU replacement
    int oldest_idx = 0;
    uint32_t oldest_time = neighbor_rssi[0].last_update;
    
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            // Existing entry - apply EWMA smoothing
            old_rssi = neighbor_rssi[i].rssi;
            neighbor_rssi[i].rssi = rssi_ewma(old_rssi, rssi);
            neighbor_rssi[i].last_update = now;
            return;
        }
        // Track oldest entry for potential replacement
        if (neighbor_rssi[i].last_update < oldest_time) {
            oldest_idx = i;
            oldest_time = neighbor_rssi[i].last_update;
        }
    }
    
    // New neighbor - use oldest slot (LRU replacement)
    neighbor_rssi[oldest_idx].addr = addr;
    neighbor_rssi[oldest_idx].rssi = rssi;
    neighbor_rssi[oldest_idx].last_update = now;
}

/**
 * @brief Get RSSI value for specific neighbor
 * 
 * @param addr Neighbor address to query
 * @return RSSI value in dBm, or -127 if neighbor not found
 */
static int8_t get_neighbor_rssi(uint16_t addr)
{
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            return neighbor_rssi[i].rssi;
        }
    }
    return -127; // No data available
}

/* ============================================================================
 * STATISTICS WINDOW MANAGEMENT - Sliding window for metrics
 * ============================================================================ */

/**
 * @brief Reset congestion statistics if measurement window has expired
 * 
 * Implements a 60-second sliding window for congestion tracking.
 * Window resets automatically after CONGESTION_WINDOW_MS.
 */
static void check_congestion_window(void)
{
    uint32_t now = k_uptime_get_32();
    if (congestion_stats.window_start == 0) {
        congestion_stats.window_start = now;  // Initialize window
    } else if (now - congestion_stats.window_start > CONGESTION_WINDOW_MS) {
        // Window expired - reset statistics
        congestion_stats.total_sends = 0;
        congestion_stats.failed_sends = 0;
        congestion_stats.window_start = now;
    }
}

/**
 * @brief Reset delivery statistics if measurement window has expired
 * 
 * Implements a 60-second sliding window for packet delivery tracking.
 */
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

/* ============================================================================
 * RSSI SMOOTHING
 * ============================================================================ */

/**
 * @brief Apply Exponential Weighted Moving Average (EWMA) to RSSI values
 * 
 * EWMA formula: new_value = (7/8) * old_value + (1/8) * new_measurement
 * This smooths out rapid RSSI fluctuations while still responding to real changes.
 * 
 * @param prev Previous RSSI value
 * @param now New RSSI measurement
 * @return Smoothed RSSI value
 */
static inline int8_t rssi_ewma(int8_t prev, int8_t now) {
    // alpha = 1/8 (EWMA smoothing: 7/8 old + 1/8 new)
    int32_t acc = (7 * (int32_t)prev + (int32_t)now) / 8;
    if (acc > 127) acc = 127;
    if (acc < -128) acc = -128;
    return (int8_t)acc;
}

/* ============================================================================
 * CORE DSDV ROUTING - Route table update logic
 * ============================================================================ */

/**
 * @brief Update or insert a route in the routing table (CORE DSDV LOGIC)
 * 
 * This is the heart of the DSDV protocol. Implements the Bellman-Ford
 * distance-vector algorithm with sequence numbers to prevent routing loops.
 * 
 * DSDV Update Rules (priority order):
 * 1. Fresher sequence number → always accept (newer information)
 * 2. Same sequence, shorter path → accept (better quality)
 * 3. Same sequence, same/longer path → refresh timestamp only (keep-alive)
 * 4. Older sequence → reject (stale information)
 * 
 * @param dest Destination node address
 * @param next_hop Next hop to reach destination
 * @param hop_count Number of hops to destination
 * @param seq_num Sequence number from destination (freshness indicator)
 */
static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num) {
    struct dsdv_route_entry *e = find_route(dest);
    uint32_t now = k_uptime_get_32();
    const uint32_t SETTLING_TIME_MS = 5000; // 5s settling for hop-count changes on same seq

    if (!e) {
        // NEW ROUTE: Find empty slot or replace oldest entry (LRU)
        int slot = -1;
        uint32_t oldest_time = UINT32_MAX;
        
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
            if (g_dsdv_routes[i].dest == 0) {
                slot = i;  // Found empty slot
                break;
            }
            // Track oldest entry for LRU replacement
            if (g_dsdv_routes[i].last_update_time < oldest_time) {
                oldest_time = g_dsdv_routes[i].last_update_time;
                slot = i;
            }
        }
        
        // Insert new route
        g_dsdv_routes[slot] = (struct dsdv_route_entry){
            .dest = dest, 
            .next_hop = next_hop, 
            .hop_count = hop_count, 
            .seq_num = seq_num,
            .last_update_time = now,
            .tentative = 0
        };
        dsdv_route_changed = true;
        return;
    }
    
    // EXISTING ROUTE: Apply DSDV update rules with priority
    
    if (seq_num > e->seq_num) {
        // Rule 1: Fresher sequence → accept update
        e->next_hop = next_hop;
        e->hop_count = hop_count;
        e->seq_num = seq_num;
        e->last_update_time = now;
        e->tentative = 0; // new sequence considered stable immediately
        dsdv_route_changed = true;
    }
    else if (seq_num == e->seq_num) {
        if (hop_count < e->hop_count) {
            // Rule 2: Same seq, shorter path → apply settling time to avoid oscillation
            e->next_hop = next_hop;
            e->hop_count = hop_count;

            if (e->tentative == 0) {
                // First time we see an improvement on this seq: mark tentative
                e->tentative = 1;
                e->last_update_time = now; // start settling timer
            } else {
                // Already tentative: if enough time passed, consider stable and trigger update
                if (now - e->last_update_time >= SETTLING_TIME_MS) {
                    e->tentative = 0;
                    e->last_update_time = now;
                    dsdv_route_changed = true;
                } else {
                    // Still within settling window: update timestamp but do not trigger update yet
                    e->last_update_time = now;
                }
            }
        }
        else if (hop_count == e->hop_count) {
            /* Same seq and same hop_count: use RSSI as tie-breaker.
             * Prefer route via neighbor with significantly better link quality.
             */
            int8_t new_rssi = get_neighbor_rssi(next_hop);
            int8_t old_rssi = get_neighbor_rssi(e->next_hop);

            if (new_rssi != -127 && old_rssi != -127 &&
                new_rssi >= old_rssi + 6) {
                // New route has clearly better RSSI (>= 6 dB improvement)
                e->next_hop = next_hop;
                e->hop_count = hop_count;
                e->tentative = 0;
                e->last_update_time = now;
                dsdv_route_changed = true;
            } else {
                // RSSI not significantly better (or missing) → keep existing route, just refresh
                e->tentative = 0;
                e->last_update_time = now;
            }
        }
        else {
            // Same seq, longer path → keep-alive only, do not downgrade route
            e->tentative = 0;
            e->last_update_time = now;
        }
    }
    else {
        // Rule 4: Older sequence → reject silently (normal operation)
        // No need to log warnings for every stale packet
    }
}

/* ============================================================================
 * METRICS COLLECTION - Network performance monitoring
 * ============================================================================ */

/**
 * @brief Collect comprehensive network metrics from this node
 * 
 * Gathers:
 * - RSSI to target node
 * - Congestion statistics (send failure rate)
 * - All neighbor RSSI values (link quality map)
 * - Optimal TTL based on valid route (with expired check)
 * 
 * @param chat Chat model instance
 * @param metrics Output structure to fill with collected metrics
 */
static void collect_current_metrics(struct bt_mesh_chat_cli *chat, struct bt_mesh_network_metrics *metrics)
{
	uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
	metrics->src_addr = my_addr;
	metrics->about_addr = current_target_node;
	metrics->timestamp = k_uptime_get_32();
	
	// Calculate TTL dynamically based on route
	struct dsdv_route_entry *route = find_route(current_target_node);
	metrics->initial_ttl = calculate_ttl(route);
	metrics->request_ack = 1;
	
	// Get per-neighbor RSSI (more accurate than global)
	int8_t rssi = get_neighbor_rssi(current_target_node);
	// Fallback: If no RSSI data, use -80 dBm (realistic weak signal assumption)
	metrics->rssi_dbm = (rssi == -127) ? -80 : rssi;
	
	if (!chat->model || !chat->model->rt) {
		return;
	}
	
	// Calculate congestion with sliding window (failures per 1000 sends)
	check_congestion_window();
	if (congestion_stats.total_sends > 0) {
		metrics->congestion = (congestion_stats.failed_sends * 1000) / congestion_stats.total_sends;
	} else {
		metrics->congestion = 0;
	}
	
	metrics->hop_count = 1;  // Start at 1 (source to first hop)
	
	// Collect all neighbor RSSI values for link quality map
	uint32_t now = k_uptime_get_32();
	metrics->neighbor_count = 0;
	
	for (int i = 0; i < MAX_RSSI_NEIGHBORS && metrics->neighbor_count < MAX_NEIGHBORS_IN_METRICS; i++) {
        if (neighbor_rssi[i].addr != 0 && 
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			metrics->neighbor_rssi[metrics->neighbor_count].addr = neighbor_rssi[i].addr;
			metrics->neighbor_rssi[metrics->neighbor_count].rssi = neighbor_rssi[i].rssi;
			metrics->neighbor_count++;
		}
	}
	
	LOG_DBG("RSSI=%ddBm (node 0x%04x), congestion=%u‰, neighbors=%u", 
	        metrics->rssi_dbm, current_target_node, 
	        metrics->congestion, metrics->neighbor_count);
}

/**
 * @brief Work handler for sending metrics packets
 * 
 * Scheduled by user command to send metrics to target node.
 * Uses DSDV routing to find best path.
 * 
 * @param work Pointer to work structure
 */
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
	
	// Find valid route to destination
	struct dsdv_route_entry *route = find_route(dest);
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
        /* Link failure heuristic: consecutive send failures invalidate route early */
        route->fail_streak++;
        if (route->fail_streak >= 3) {
            if ((route->seq_num & 1) == 0) {
                route->seq_num += 1; // mark odd sequence for invalidation
            }
            route->hop_count = 0xFF; // unreachable
            route->last_update_time = k_uptime_get_32();
            dsdv_route_changed = true; // trigger quick UPDATE
            route->fail_streak = 0; // reset counter
            LOG_WRN("Route to 0x%04x invalidated early (send failures)", dest);
        }
	}
	// LOG_DBG("dsdv data to 0x%04x via 0x%04x, ret=%d", dest, route->next_hop, ret);
	
}

/* ============================================================================
 * DSDV PROTOCOL WORKERS - Periodic packet transmission
 * ============================================================================ */

/**
 * @brief Periodic HELLO packet sender (runs every ~5 seconds)
 * 
 * HELLO Packet Purpose:
 * - Neighbor discovery (who can I hear directly?)
 * - RSSI measurement (how strong is the link?)
 * - 1-hop route creation (direct neighbors)
 * 
 * Timing: 5000ms + random jitter (0-500ms) to prevent synchronization
 * 
 * @param work Pointer to work structure
 */
static void dsdv_send_hello(struct k_work *work){
	if (!g_chat_cli_instance||!g_chat_cli_instance->model||!g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_hello_work, K_MSEC(1000));
		return;
	}

	// Cleanup expired routes before sending HELLO
	dsdv_cleanup_expired_routes();

	// Increment sequence number (even numbers only for HELLO)
	g_dsdv_my_seq += 2;
	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;

	struct dsdv_hello hello = {
		.src = my_addr,
		.seq_num = g_dsdv_my_seq,
		.flags = 0
	};

	// Publish HELLO packet to all neighbors
	struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;
	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_HELLO);
	net_buf_simple_add_mem(pub, &hello, sizeof(hello));
	
	// NOTE: Not forcing TTL=1 to allow multi-hop HELLO propagation like original code
	// (TTL will use model's publish config, typically 3-7)
	// g_chat_cli_instance->model->pub->ttl = 1;
	(void)bt_mesh_model_publish(g_chat_cli_instance->model);
	
	// Log periodically (every 6th HELLO ≈ 30s) to reduce spam
	//static uint8_t hello_count = 0;
	//if (++hello_count % 6 == 0) {
		LOG_INF("[HELLO TX] 0x%04x seq=%u", my_addr, g_dsdv_my_seq);
	//}
        LOG_INF("[HELLO PUB] pub.addr=0x%04x, ttl=%u",
        g_chat_cli_instance->model->pub->addr,
        g_chat_cli_instance->model->pub->ttl);
	// Reschedule with jitter to prevent network synchronization
	uint32_t jitter = sys_rand32_get() % 500;
	k_work_reschedule(&dsdv_hello_work, K_MSEC(5000 + jitter));

}

/**
 * @brief Periodic UPDATE packet sender (runs every ~15 seconds)
 * 
 * UPDATE Packet Purpose:
 * - Share routing table with neighbors
 * - Propagate routes throughout network
 * - Enable multi-hop routing
 * 
 * Contains up to 16 route entries (excludes self and empty slots)
 * Hop counts are incremented by 1 when sent (distance-vector algorithm)
 * 
 * Timing: 15000ms + random jitter (0-1000ms)
 * 
 * @param work Pointer to work structure
 */
static void dsdv_send_update(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model || !g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_update_work, K_MSEC(2000));
		return;
	}

    /* Rotation index to distribute which routes get advertised first across cycles.
     * This avoids starvation of later table entries when buffer/clamp limits apply.
     */
    static uint16_t update_rotation_index = 0;

    uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;
    uint32_t now = k_uptime_get_32();
    uint8_t num_entries = 0;
	
    /* First pass: count valid routes starting from rotation index */
    for (int k = 0; k < DSDV_ROUTE_TABLE_SIZE && num_entries < MAX_UPDATE_ENTRIES; ++k) {
        int i = (update_rotation_index + k) % DSDV_ROUTE_TABLE_SIZE;
        if (g_dsdv_routes[i].dest != 0 &&
            g_dsdv_routes[i].dest != my_addr &&
            (now - g_dsdv_routes[i].last_update_time) <= DSDV_ROUTE_TIMEOUT_MS) {
            num_entries++;
        }
    }
	
	if (num_entries == 0) {
		k_work_reschedule(&dsdv_update_work, K_MSEC(10000));
		return;
	}
	
    struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;
    net_buf_simple_reset(pub);
    bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_UPDATE);

    /* Buffer safety: clamp num_entries to actual buffer capacity */
    size_t entry_size = sizeof(struct dsdv_update_entry);
    size_t header_size = sizeof(struct dsdv_update_header);
    size_t capacity = pub->size; /* after reset len == 0 */
    size_t max_entries_buf = 0;
    if (capacity > header_size) {
        max_entries_buf = (capacity - header_size) / entry_size;
    }
    if (num_entries > max_entries_buf) {
        num_entries = (uint8_t)max_entries_buf;
    }

    struct dsdv_update_header hdr = {
        .src = my_addr,
        .num_entries = num_entries,
        .flags = 0
    };

    net_buf_simple_add_mem(pub, &hdr, sizeof(hdr));

	uint8_t added = 0;
	
    for (int k = 0; k < DSDV_ROUTE_TABLE_SIZE && added < num_entries; ++k) {
        int i = (update_rotation_index + k) % DSDV_ROUTE_TABLE_SIZE;
        if (g_dsdv_routes[i].dest != 0 &&
            g_dsdv_routes[i].dest != my_addr &&
            (now - g_dsdv_routes[i].last_update_time) <= DSDV_ROUTE_TIMEOUT_MS) {

            /* Ensure tailroom before adding */
            if ((pub->size - pub->len) < sizeof(struct dsdv_update_entry)) {
                break;
            }
            const struct dsdv_route_entry *entry = &g_dsdv_routes[i];
            struct dsdv_update_entry update_entry = {
                .dest = entry->dest,
                .hop_count = entry->hop_count,
                .seq_num = entry->seq_num,
                .padding = 0
            };
            net_buf_simple_add_mem(pub, &update_entry, sizeof(update_entry));
            added++;
        }
    }

    /* Advance rotation index for next cycle (added may be < num_entries if clamped) */
    update_rotation_index = (update_rotation_index + added) % DSDV_ROUTE_TABLE_SIZE;

	(void)bt_mesh_model_publish(g_chat_cli_instance->model);
    bool changed = dsdv_route_changed;
    dsdv_route_changed = false;
    uint32_t base_delay = changed ? 800 : 15000;
    uint32_t jitter = sys_rand32_get() % (changed ? 400 : 1000);
    k_work_reschedule(&dsdv_update_work, K_MSEC(base_delay + jitter));
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
    
	// Validate packet size
	struct dsdv_hello hello;
	if (buf->len < sizeof(hello)){
		return -EINVAL;
	}
	memcpy(&hello, net_buf_simple_pull_mem(buf, sizeof(hello)), sizeof(hello));
	
	uint16_t neighbor = ctx->addr;  // Who sent this HELLO
	uint16_t dest = hello.src;      // Original source (should be same as neighbor)
	uint32_t seq = hello.seq_num;   // Sequence number from source
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

	// Log EVERY HELLO that reaches access layer (before any filters)
	LOG_INF("[HELLO RX] from src=0x%04x (ctx->addr=0x%04x, my_addr=0x%04x, rssi=%d, ttl=%u)",
			hello.src, ctx->addr, my_addr, ctx->recv_rssi, ctx->recv_ttl);

    /* SECURITY / CONSISTENCY CHECK:
     * Reject HELLO if lower-layer sender address (ctx->addr) differs from declared source (hello.src).
     * This prevents creating a 1-hop route to a spoofed address if the packet header was modified or relayed improperly.
     */
    if (neighbor != dest) {
        LOG_WRN("[HELLO REJECT] Spoofed: ctx->addr=0x%04x != hello.src=0x%04x", neighbor, dest);
        return 0; // Silently ignore inconsistent HELLO
    }
	
	// Filter 1: Ignore self-originated packets (prevent routing loop)
	if (dest == my_addr) {
		LOG_DBG("[HELLO SELF] Ignoring own HELLO");
		return 0;
	}
	// Filter 2: Ignore very weak signals (< -90dBm = unreliable link)
	if (ctx->recv_rssi != 0 && ctx->recv_rssi < -90) {
		return 0;
	}

	// Update neighbor RSSI tracking (used for link quality metrics)
	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}
	
	// Create/update 1-hop route to neighbor
	dsdv_upsert(dest, neighbor, 1, seq);
	
	// Trigger UPDATE to propagate new neighbor to network (with jitter)
	k_work_reschedule(&dsdv_update_work, K_MSEC(2000 + (sys_rand32_get() % 500)));
	
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
	uint16_t neighbor = ctx->addr; /*neighbor address*/
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
	
	// CRITICAL: Clean up expired routes before processing new updates
	// This ensures routing table adapts to topology changes (node leaving, link break)
	dsdv_cleanup_expired_routes();
	
	// CRITICAL: Ignore packets from self to prevent routing loop
	if (hdr.src == my_addr) {
		return 0;
	}

	// CRITICAL: BLE Mesh broadcast leak protection
	// Problem: BLE Mesh can broadcast UPDATE packets beyond 1-hop range
	// Solution: Only accept UPDATE from nodes in neighbor_rssi[] (populated by HELLO)
	// This ensures UPDATE sender is truly a direct neighbor (1-hop away)
	bool is_direct_neighbor = false;
	uint32_t now = k_uptime_get_32();
	
	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        // Check if neighbor exists and was heard recently
        if (neighbor_rssi[i].addr == neighbor && 
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			is_direct_neighbor = true;
			break;
		}
	}
	
	// Reject UPDATE from non-neighbors to prevent incorrect routing table
	if (!is_direct_neighbor) {
		return 0;  // Silent rejection (normal in BLE Mesh broadcast environment)
	}

	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

	for (int i = 0; i < hdr.num_entries; ++i)
	{
		if (buf->len < sizeof(struct dsdv_update_entry))
		{
			break;
		}
		struct dsdv_update_entry entry;
		memcpy(&entry, net_buf_simple_pull_mem(buf, sizeof(entry)), sizeof(entry));
		if (entry.dest == my_addr || entry.dest == neighbor)
		{
			continue; // ignore route to self
		}
		
		// Distance-vector algorithm: increment hop count
		// Neighbor says "X hops to dest" → We are X+1 hops away (through neighbor)
		uint8_t actual_hops = entry.hop_count + 1;
		
		// Prevent overflow (rare: network diameter would need to be 255+ hops)
		if (entry.hop_count >= UINT8_MAX) {
			continue;  // Skip this route (unreachable)
		}
		
		// Update routing table with route through this neighbor
		dsdv_upsert(entry.dest, neighbor, actual_hops, entry.seq_num);
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
	
     if (ctx->recv_rssi != 0) {
        update_neighbor_rssi(ctx->addr, ctx->recv_rssi);
    }
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
        LOG_INF("| Node 0x%04x: Neighbors=%2u, Weak=%3u%%, Congestion=%4u%% |",
                m->src_addr,
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
        // LOG_DBG("Drop duplicate relay DATA from 0x%04x seq=%u", pkt.src, (unsigned)pkt.seq_num);
        return 0;
    }
    struct dsdv_route_entry *route = find_route(pkt.dest);
    if (!route) {
        LOG_WRN("No route to forward packet from 0x%04x to 0x%04x", 
                pkt.src, pkt.dest);
        return 0;
    }
    // If collect_relay_metrics flag is set, send relay metrics separately ***
    if (pkt.collect_relay_metrics == 1) {
        struct bt_mesh_network_metrics my_metrics;
        current_target_node = pkt.dest;
        collect_current_metrics(chat, &my_metrics);
        
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
                .addr = route->next_hop,  // Gửi qua DSDV next_hop (không trực tiếp)
                .app_idx = chat->model->keys[0],
                .send_ttl = calculate_ttl(route),  // TTL dựa trên route
                .send_rel = false,  // Best effort delivery
            };
            
            int ret = bt_mesh_model_send(chat->model, &relay_ctx, &relay_msg, NULL, NULL);
            if (ret == 0) {
                // LOG_DBG("Relay 0x%04x sent metrics via 0x%04x (hops=%u) to dest 0x%04x", 
                //         my_addr, route->next_hop, route->hop_count, pkt.dest);
            } else {
                LOG_WRN("Failed to send relay metrics: %d", ret);
            }
        }

    // Tăng hop_count và thêm mình vào path
    pkt.hop_count++;
    if (pkt.path_len < MAX_PATH_NODES) {
        pkt.path_nodes[pkt.path_len++] = my_addr;
    }else {
        LOG_WRN("Path length exceeded max nodes, cannot add 0x%04x", my_addr);
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

    int ret = bt_mesh_model_send(chat->model, &fwd_ctx, &fwd, NULL, NULL);
    if (ret != 0) {
        LOG_ERR("Failed to forward DSDV data to 0x%04x via 0x%04x, ret=%d", 
                pkt.dest, route->next_hop, ret);
        /* Early invalidation heuristic for relay failures */
        route->fail_streak++;
        if (route->fail_streak >= 3) {
            if ((route->seq_num & 1) == 0) {
                route->seq_num += 1; // odd sequence indicates invalidation
            }
            route->hop_count = 0xFF;
            route->last_update_time = k_uptime_get_32();
            dsdv_route_changed = true;
            route->fail_streak = 0;
            LOG_WRN("Relay invalidated route to 0x%04x (forward failures)", pkt.dest);
        }
        return ret;
    }

    // LOG_DBG("Forwarded DSDV data to 0x%04x via 0x%04x", 
    //     pkt.dest, route->next_hop);
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
    LOG_INF("| Node 0x%04x:Neighbors=%2u, Weak=%3u%%, Congestion=%4u‰ |",
            m->src_addr,
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
    
    // *** NEW: Store relay metrics to cache for later analysis ***
    store_relay_metrics(relay_pkt.relay_addr, 
                       relay_pkt.original_src,
                       relay_pkt.original_seq,
                       &relay_pkt.relay_metrics);
    
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
            int8_t rssi = get_neighbor_rssi(g_dsdv_routes[i].next_hop);
            
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

/* .. include_startingpoint_chat_cli_rst_2 */
const struct bt_mesh_model_op _bt_mesh_chat_cli_op[] = {
    {
        BT_MESH_CHAT_CLI_OP_METRICS_ACK,
        BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK),
        handle_metrics_ack
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
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) { // Only recent
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
    
    LOG_INF("========================================");
    LOG_INF("[METRICS SEND] From 0x%04x to 0x%04x", my_addr, dest);
    LOG_INF("========================================");
    
    // Show full routing table before lookup
    LOG_INF("Current routing table:");
    int route_count = 0;
    uint32_t now = k_uptime_get_32();
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
        if (g_dsdv_routes[i].dest != 0) {
            uint32_t age_sec = (now - g_dsdv_routes[i].last_update_time) / 1000;
            LOG_INF("  [%d] dest=0x%04x via 0x%04x, hops=%u, seq=%u, age=%us%s",
                    i, g_dsdv_routes[i].dest, g_dsdv_routes[i].next_hop,
                    g_dsdv_routes[i].hop_count, g_dsdv_routes[i].seq_num, age_sec,
                    (age_sec > 45) ? " (EXPIRED)" : "");
            route_count++;
        }
    }
    if (route_count == 0) {
        LOG_WRN("  (Routing table is EMPTY)");
    }
    LOG_INF("----------------------------------------");

    // Find valid route to destination
    struct dsdv_route_entry *route = find_route(dest);
    if (!route) {
        LOG_WRN("No valid route to 0x%04x", dest);
        return -ENOENT;
    }

    // Log route details when sending metrics
    int8_t next_hop_rssi = get_neighbor_rssi(route->next_hop);
    LOG_INF("=== SENDING METRICS ===");
    LOG_INF("Source: 0x%04x → Destination: 0x%04x", my_addr, dest);
    LOG_INF("Selected Route: next_hop=0x%04x | route_hops=%u | RSSI=%ddBm | TTL=%u",
            route->next_hop, route->hop_count, next_hop_rssi, calculate_ttl(route));

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
        LOG_INF("| Route %d |     %4u |  %2u |     %3u |       %4u |",
                i + 1,
                route_history[i].latency_ms,
                route_history[i].hop_count,
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

    struct dsdv_route_entry *route = find_route(dest);
    if (!route) {
        LOG_WRN("No valid route to 0x%04x", dest);
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
