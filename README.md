# Bluetooth Mesh DSDV Routing Protocol

A Bluetooth Mesh implementation with DSDV (Destination-Sequenced Distance Vector) routing protocol and comprehensive network metrics collection.

## Features

### 🛣️ DSDV Routing
- **HELLO packets** (5s interval): Neighbor discovery and RSSI tracking
- **UPDATE packets** (15s interval): Routing table propagation
- **Multi-hop routing** with path vector tracking
- **Sequence number** based loop prevention
- **RSSI-first route selection** (70% RSSI + 30% hop count)

### 📊 Network Metrics
- **Battery percentage** monitoring (ADC-based or simulated)
- **RSSI per-neighbor** tracking with EWMA smoothing
- **Congestion detection** (failed sends ratio)
- **Hop count** measurement (incremental and TTL-based)
- **End-to-end latency** calculation (RTT/2)
- **Relay metrics collection** (each relay sends its metrics separately)

### ⏱️ Convergence Monitoring
- **Convergence time** measurement (network stabilization time)
- **Route change detection** with 30-second stability threshold
- **Reconfigurations counting** (total, min, max times)
- **Network state tracking** (STABLE vs CONVERGING)

### 💡 LED Indicators
- **Provisioning**: LED blinks 3 times when node is provisioned
- **Packet receive**: LED blinks once on packet reception (HELLO, UPDATE, DATA)

## Project Structure

```
chat_2/
├── CMakeLists.txt              # Build configuration
├── prj.conf                    # Project configuration
├── Kconfig                     # Kconfig options
├── include/
│   ├── chat_cli.h              # Chat client model API
│   ├── model_handler.h         # Model handler API
│   └── battery_adc.h           # Battery ADC interface
├── src/
│   ├── main.c                  # Main application entry
│   ├── chat_cli.c              # DSDV routing implementation
│   ├── model_handler.c         # Shell commands and handlers
│   └── battery_adc.c           # Battery voltage reading
└── boards/
    └── *.conf                  # Board-specific configurations
```

## Shell Commands

### Network Monitoring
```bash
# Show DSDV routing table
chat routes

# Show per-neighbor RSSI statistics
chat neighbors

# Show battery status
chat battery

# Show convergence statistics
chat status
```

### Metrics Operations
```bash
# Send metrics to specific node
chat metrics_to <addr>

# Request network structure from node
chat structure_to <addr>

# Request convergence statistics from node
chat convergence_to <addr>
```

### Route Analysis
```bash
# Verify route to destination
chat verify_route <dest_addr>

# Show route metrics history
chat show_history

# Clear route history
chat clear_history
```

## Packet Types

| Packet | Opcode | Size | Frequency | Purpose |
|--------|--------|------|-----------|---------|
| HELLO | 0x12 | 8 bytes | 5s | Neighbor discovery + RSSI |
| UPDATE | 0x13 | 4+(18×N) | 15s | Routing table sharing |
| DATA | 0x14 | ~50-70 | On-demand | Send metrics + data |
| RELAY_METRICS | 0x15 | ~24-48 | On-relay | Relay node metrics |
| METRICS_ACK | 0x0F | 8 bytes | On-receive | ACK for latency |
| STRUCTURE_REQ | 0x16 | 6 bytes | On-demand | Request routing table |
| CONVERGENCE_REQ | 0x17 | 6 bytes | On-demand | Request stats |

## Route Selection Algorithm

**RSSI-First Strategy (70% RSSI + 30% Hop Count)**

```c
rssi_score = 1.0 - ((rssi + 100) / 70.0)  // Strong signal = low score
hop_score = hop_count / 15.0              // Few hops = low score
total_score = (rssi_score * 0.7) + (hop_score * 0.3)
→ Select route with LOWEST score
```

**Rationale:**
- Prioritize strong signal (RSSI) to minimize retransmissions
- Use hop count as tiebreaker for similar RSSI values
- Reduce packet loss and improve network reliability

## Building and Flashing

### Prerequisites
- nRF Connect SDK v2.5.0+
- nRF52 DK or compatible board
- West build tool

### Build
```bash
west build -b nrf52dk_nrf52832
```

### Flash
```bash
west flash
```

## Configuration

### TX Power Adjustment
Edit `prj.conf`:
```ini
# For linear topology testing (short range)
CONFIG_BT_CTLR_TX_PWR_MINUS_20=y

# For standard range
CONFIG_BT_CTLR_TX_PWR_0=y
```

### DSDV Parameters
```c
#define DSDV_HELLO_INTERVAL_MS   5000   // HELLO broadcast interval
#define DSDV_UPDATE_INTERVAL_MS  15000  // UPDATE broadcast interval
#define DSDV_ROUTE_TIMEOUT_MS    45000  // Route expiration (3× UPDATE)
#define CONVERGENCE_THRESHOLD_MS 30000  // Stability detection threshold
```

## Example Output

### Metrics Reception
```
=== RECEIVED METRICS AT DESTINATION ===
Source: 0x0001, Relay count: 2
Path vector:
  [0] 0x0001 (source)
  [1] 0x0002 (relay)
  [2] 0x0003 (destination)

=== SOURCE NODE METRICS ===
+--------+---------+----------+-------+-----------+
| Node 0x0001: Battery= 85%, Neighbors= 3, Weak= 33%, Congestion=   5‰ |
+--------+---------+----------+-------+-----------+
| Neighbors RSSI:                                    |
|   → 0x0002:  -45 dBm                               |
|   → 0x0004:  -50 dBm                               |
|   → 0x0005:  -82 dBm (WEAK)                        |
+----------------------------------------------------+
End-to-end latency: 125 ms
```

### Convergence Detection
```
=== NETWORK CONVERGED ===
Convergence time: 8520 ms
Total reconfigurations: 3
Min/Max: 5120 / 12340 ms
```

## License

SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

## Author

Developed as part of a thesis project on Bluetooth Mesh routing protocols.

## Acknowledgments

- Based on Nordic Semiconductor's Bluetooth Mesh Chat sample
- DSDV protocol implementation inspired by RFC 3561
