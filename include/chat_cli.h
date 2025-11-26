/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file chat_cli.h
 * @defgroup bt_mesh_chat_cli Bluetooth Mesh Chat Client Model API
 * @{
 * @brief API for the Bluetooth Mesh Chat Client model with DSDV routing
 * 
 * This model implements:
 * - DSDV (Destination-Sequenced Distance-Vector) proactive routing protocol
 * - Network metrics collection (battery, RSSI, congestion, neighbor quality)
 * - Automatic expired route filtering (45s timeout)
 * - Remote structure querying
 */

#ifndef BT_MESH_CHAT_CLI_H__
#define BT_MESH_CHAT_CLI_H__

#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/model_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * MODEL IDENTIFICATION
 * ============================================================================ */

/** Company ID of the Bluetooth Mesh Chat Client model. */
#define BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID    CONFIG_BT_COMPANY_ID_NORDIC

/** Model ID of the Bluetooth Mesh Chat Client model. */
#define BT_MESH_CHAT_CLI_VENDOR_MODEL_ID      0x000A

/* ============================================================================
 * OPCODES - Mesh message operation codes
 * ============================================================================ */

/** Opcode: Metrics acknowledgment packet */
#define BT_MESH_CHAT_CLI_OP_METRICS_ACK BT_MESH_MODEL_OP_3(0x0F, \
                                       BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID)
/** Opcode: DSDV HELLO packet (neighbor discovery + RSSI) */
#define BT_MESH_CHAT_CLI_OP_DSDV_HELLO BT_MESH_MODEL_OP_3(0x12, \
									   BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID)

/** Opcode: DSDV UPDATE packet (routing table exchange) */
#define BT_MESH_CHAT_CLI_OP_DSDV_UPDATE BT_MESH_MODEL_OP_3(0x13, \
									   BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID)

/** Opcode: DSDV DATA packet (end-to-end data with metrics) */
#define BT_MESH_CHAT_CLI_OP_DSDV_DATA BT_MESH_MODEL_OP_3(0x14, \
									   BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID)

/** Opcode: Relay metrics packet (metrics from intermediate nodes) */
#define BT_MESH_CHAT_CLI_OP_RELAY_METRICS BT_MESH_MODEL_OP_3(0x15, \
									   BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID)

/** Opcode: Structure request packet (query routing table) */
#define BT_MESH_CHAT_CLI_OP_STRUCTURE_REQUEST BT_MESH_MODEL_OP_3(0x16, \
									   BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID)

/* ============================================================================
 * MESSAGE LENGTH CONSTANTS
 * ============================================================================ */
									   
#define BT_MESH_CHAT_CLI_MSG_LEN_DSDV_HELLO 8                /**< HELLO packet size */
#define BT_MESH_CHAT_CLI_MSG_LEN_DSDV_UPDATE_MIN 8           /**< Minimum UPDATE packet size */
// #define BT_MESH_CHAT_CLI_MSG_LEN_NETWORK_METRICS 15       /**< DEPRECATED: Legacy standalone metrics */
#define BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK 8               /**< Metrics ACK packet size */
#define BT_MESH_CHAT_CLI_MSG_LEN_RELAY_METRICS sizeof(struct relay_metrics_packet)  /**< Relay metrics size */
#define BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX 128           /**< Maximum DATA packet size */

/* ============================================================================
 * DATA STRUCTURES - Packet formats and routing table entries
 * ============================================================================ */

/**
 * @brief Per-neighbor RSSI tracking entry
 * 
 * Stores RSSI measurement for each neighbor to enable link quality
 * assessment and route selection.
 */
struct neighbor_rssi_entry {
	uint16_t addr;  /**< Neighbor node address */
	int8_t rssi;    /**< RSSI value in dBm */
} __packed;

/** Maximum number of neighbors to include in metrics packet */
#define MAX_NEIGHBORS_IN_METRICS 8

/**
 * @brief Network metrics packet structure
 * 
 * Contains comprehensive network performance metrics from a node:
 * - Link quality (RSSI to target and all neighbors)
 * - Network load (congestion percentage)
 * - Packet delivery info (TTL, hop count)
 */
struct bt_mesh_network_metrics
{
	uint16_t src_addr;      /**< Source node address */
	uint16_t about_addr;    /**< Target node address (for directed metrics) */
	uint32_t timestamp;     /**< Metric collection timestamp (ms) */
	int8_t rssi_dbm;        /**< RSSI to about_addr in dBm */
	uint8_t _padding;       /**< Padding for alignment */
	uint16_t congestion;    /**< Congestion level (failures per 1000 sends) */
	uint8_t initial_ttl;    /**< TTL used for packet transmission */
	uint8_t hop_count;      /**< Number of hops from source */
	uint8_t request_ack;    /**< 1 = request acknowledgment, 0 = no ACK needed */
	uint8_t neighbor_count; /**< Number of valid entries in neighbor_rssi[] */
	struct neighbor_rssi_entry neighbor_rssi[MAX_NEIGHBORS_IN_METRICS]; /**< All neighbor link qualities */
}__packed;

/**
 * @brief Metrics acknowledgment packet
 * 
 * Sent by destination to confirm metrics packet delivery.
 * Enables RTT (Round Trip Time) calculation.
 */
struct bt_mesh_metrics_ack{
	uint16_t src_addr;           /**< Original sender address */
	uint32_t original_timestamp; /**< Echo of original timestamp for RTT calculation */
	uint16_t padding;            /**< Padding to reach 8 bytes */
} __packed;

/**
 * @brief DSDV routing table entry
 * 
 * Stores routing information for one destination in the network.
 * Updated when receiving HELLO or UPDATE packets.
 */
struct dsdv_route_entry {
	uint16_t dest;               /**< Destination node address */
	uint16_t next_hop;           /**< Next hop node address to reach dest */
	uint8_t hop_count;           /**< Number of hops to destination (0xFF = invalidation) */
	uint32_t seq_num;            /**< Sequence number from destination (freshness / odd=invalid) */
	uint32_t last_update_time;   /**< Last successful update time (ms) */
	uint8_t tentative;           /**< 1 = route learned without confirmed HELLO yet */
	uint8_t fail_streak;         /**< Consecutive send failures via this route */
	uint8_t dirty;               /**< 1 = needs to be advertised in next incremental UPDATE */
	uint8_t _pad;                /**< Padding for alignment */
}__packed;

/**
 * @brief DSDV HELLO packet structure
 * 
 * Broadcast every ~5 seconds for neighbor discovery.
 * Receivers create 1-hop routes and measure RSSI.
 */
struct dsdv_hello {
    uint16_t src;       /**< Source node address */
    uint32_t seq_num;   /**< Source's sequence number (even numbers only) */
    uint16_t flags;     /**< Reserved for future use */
} __packed;

/**
 * @brief DSDV UPDATE packet header
 * 
 * Followed by num_entries route entries.
 * Shares routing table with neighbors for multi-hop routing.
 */
struct dsdv_update_header {
	uint16_t src;         /**< Source node address */
	uint8_t num_entries;  /**< Number of route entries following this header */
	uint8_t flags;        /**< Reserved flags */
} __packed;

/**
 * @brief DSDV UPDATE route entry payload
 * 
 * Sent in UPDATE packets. Does NOT include next_hop because:
 * - Receiver will set next_hop = sender_address
 * - Including next_hop would cause routing errors (transitive next_hop)
 */
struct dsdv_update_entry {
	uint16_t dest;        /**< Destination node address */
	uint8_t hop_count;    /**< Number of hops from sender to dest */
	uint32_t seq_num;     /**< Sequence number from destination */
	uint8_t padding;      /**< Padding for alignment */
} __packed;

/** Maximum number of nodes in path vector for DATA packets */
#define MAX_PATH_NODES 8

/**
 * @brief DSDV DATA packet structure
 * 
 * End-to-end data delivery with embedded metrics and path tracking.
 * Supports relay metrics collection for network analysis.
 */
struct dsdv_data_packet {
	uint16_t src;
	uint16_t dest;
	uint32_t seq_num;
	uint8_t hop_count;
	uint8_t path_len;
	uint16_t path_nodes[MAX_PATH_NODES];
	
	// Sequential mode: Only source metrics in main packet
	struct bt_mesh_network_metrics metrics;
	uint8_t collect_relay_metrics;  // Flag: 1 = relays send metrics separately
} __packed;

// Relay metrics packet (sent separately by each relay)
struct relay_metrics_packet {
	uint16_t original_src;
	uint32_t original_seq;
	uint16_t relay_addr;
	struct bt_mesh_network_metrics relay_metrics;
} __packed;

// Network structure request packet (destination will print its routing table)
struct structure_request_packet {
	uint16_t requester_addr;
	uint32_t request_seq;
} __packed;

/* Forward declaration of the Bluetooth Mesh Chat Client model context. */
struct bt_mesh_chat_cli;

/* .. include_startingpoint_chat_cli_rst_2 */
/** @def BT_MESH_MODEL_CHAT_CLI
 *
 * @brief Bluetooth Mesh Chat Client model composition data entry.
 *
 * @param[in] _chat Pointer to a @ref bt_mesh_chat_cli instance.
 */
#define BT_MESH_MODEL_CHAT_CLI(_chat)                                          \
		BT_MESH_MODEL_VND_CB(BT_MESH_CHAT_CLI_VENDOR_COMPANY_ID,       \
			BT_MESH_CHAT_CLI_VENDOR_MODEL_ID,                      \
			_bt_mesh_chat_cli_op, &(_chat)->pub,                   \
			BT_MESH_MODEL_USER_DATA(struct bt_mesh_chat_cli,       \
						_chat),                        \
			&_bt_mesh_chat_cli_cb)
/* .. include_endpoint_chat_cli_rst_2 */

/** Bluetooth Mesh Chat Client model handlers. */
struct bt_mesh_chat_cli_handlers {
	/** @brief Called after the node has been provisioned, or after all
	 * mesh data has been loaded from persistent storage.
	 *
	 * @param[in] cli Chat Client instance that has been started.
	 */
	void (*const start)(struct bt_mesh_chat_cli *chat);

	/** @brief Handler for a network metrics message.
	 *
	 * @param[in] cli Chat client instance that received the message.
	 * @param[in] ctx Context of the incoming message.
	 * @param[in] metrics Pointer to the network metrics data.
	 */
	void (*const network_metrics)(struct bt_mesh_chat_cli *chat,
				       struct bt_mesh_msg_ctx *ctx,
				       const struct bt_mesh_network_metrics *metrics);
	
	/** @brief Handler for metrics ACK message.
	 * @param[in] chat Chat client instance.
	 * @param[in] ctx Context of the incoming message.
	 * @param[in] ack Metrics ACK data.
 */
	void (*const metrics_ack)(struct bt_mesh_chat_cli *chat,
				  struct bt_mesh_msg_ctx *ctx,
				  const struct bt_mesh_metrics_ack *ack);
};

/* .. include_startingpoint_chat_cli_rst_3 */
/**
 * Bluetooth Mesh Chat Client model context.
 */
struct bt_mesh_chat_cli {
	/** Access model pointer. */
	const struct bt_mesh_model *model;
	/** Publish parameters. */
	struct bt_mesh_model_pub pub;
	/** Publication message. */
	struct net_buf_simple pub_msg;
	/** Publication message buffer. */
	uint8_t buf[BT_MESH_MODEL_BUF_LEN(BT_MESH_CHAT_CLI_OP_DSDV_UPDATE,
					  BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX)];
	/** Handler function structure. */
	const struct bt_mesh_chat_cli_handlers *handlers;
};
/* .. include_endpoint_chat_cli_rst_3 */

/** @brief Send network metrics to specified destination.
 *
 * @param[in] chat Chat Client model instance to send the message.
 * @param[in] addr Address of the destination node.
 * @param[in] metrics Pointer to network metrics data to send.
 *
 * @retval 0 Successfully sent the message.
 * @retval -EINVAL The model is not bound to an application key.
 * @retval -EAGAIN The device has not been provisioned.
 */
/** @brief set target node for metrics collection.
 *
 * @param[in] target_addr Address of target node for metrics.
 */
void bt_mesh_chat_cli_set_metrics_target(uint16_t target_addr);

/** @brief Get neighbor RSSI statistics.
 *
 * Returns the number of neighbors with RSSI data.
 * Fills the provided arrays with neighbor addresses and RSSI values.
 *
 * @param[out] addrs Array to store neighbor addresses (must have space for 16).
 * @param[out] rssi Array to store RSSI values (must have space for 16).
 * @param[in] max_count Maximum number of entries to return.
 *
 * @return Number of neighbors returned.
 */
int bt_mesh_chat_cli_get_neighbor_rssi(uint16_t *addrs, int8_t *rssi, int max_count);

/** @brief Send network metrics immediately via DSDV.
 *
 * Attempts to find a DSDV route and unicast a metrics packet to @p addr.
 * If no route is found, returns an error and does not publish/broadcast.
 *
 * @param[in] chat Chat Client model instance.
 * @param[in] addr Destination unicast address.
 *
 * @retval 0 on success.
 * @retval -EINVAL Invalid args or addr.
 * @retval -ENOENT No route to destination.
 */
int bt_mesh_chat_cli_metrics_send(struct bt_mesh_chat_cli *chat, uint16_t addr);

/**
 * @brief Clear route metrics history
 *
 * Clears the stored route metrics history table at destination node.
 */
void bt_mesh_chat_cli_clear_route_history(void);

/**
 * @brief Display route metrics history
 *
 * Displays the stored route metrics comparison table.
 */
void bt_mesh_chat_cli_show_route_history(void);

/**
 * @brief Request network structure from a node
 *
 * Sends a structure request to the specified node. The target node will
 * reply with its complete DSDV routing table, showing all known nodes,
 * their next_hop addresses, hop counts, and RSSI values.
 *
 * @param[in] chat Chat Client model instance.
 * @param[in] dest Destination unicast address to request structure from.
 *
 * @retval 0 on success.
 * @retval -EINVAL Invalid arguments or destination address.
 * @retval -ENOENT No route to destination.
 */
int bt_mesh_chat_cli_structure_request(struct bt_mesh_chat_cli *chat, uint16_t dest);

/** @cond INTERNAL_HIDDEN */
extern const struct bt_mesh_model_op _bt_mesh_chat_cli_op[];
extern const struct bt_mesh_model_cb _bt_mesh_chat_cli_cb;
/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* BT_MESH_CHAT_CLI_H__ */

/** @} */
