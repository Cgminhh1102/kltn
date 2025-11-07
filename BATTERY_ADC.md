# Battery ADC Configuration

## ⚠️ IMPORTANT: Pin Selection

On nRF54L15DK, some GPIO pins are already used by buttons and LEDs.

### Pin Usage Table

| Pin    | ADC Channel | Arduino | nRF54L15DK Usage | Battery ADC? |
|--------|-------------|---------|------------------|--------------|
| P0.04  | AIN0        | A0      | **Button 3** ❌  | ❌ DO NOT USE |
| **P0.05** | **AIN1** | **A1** | **Available** ✅ | **✅ RECOMMENDED** |
| P0.06  | AIN2        | A2      | Available ✅     | ✅ Alternative |
| P0.07  | AIN3        | A3      | Available ✅     | ✅ Alternative |
| P0.08  | -           | -       | Button 4         | ❌ |
| P0.09  | -           | -       | LED 0            | ❌ |

**Default configuration uses P0.05 (AIN1) on Arduino A1 header.**

---

## Overview
This module provides battery voltage measurement using the nRF54L15 SAADC (Successive Approximation ADC).

## Hardware Setup

### Option 1: Internal VDD Measurement (Default for testing)
No external connections required. Measures the supply voltage directly.

**Pros:**
- No external components
- Easy testing

**Cons:**
- Measures regulated VDD, not actual battery
- Less accurate for battery percentage

### Option 2: External Voltage Divider (Recommended for production)
Connect battery through voltage divider to **AIN1 (P0.05)** - Arduino A1 connector.

**⚠️ IMPORTANT:** Do NOT use P0.04 (AIN0) - it's Button 3 on nRF54L15DK!

```
VBAT ---[R1: 100kΩ]---+---[R2: 100kΩ]--- GND
                      |
                    AIN1 (P0.05) - Arduino A1 header
```

**Pin Location on nRF54L15DK:**
- P0.05 is available on Arduino header pin A1
- Locate the Arduino analog headers on the side of the board
- A0, A1, A2, A3 are marked clearly

**Calculation:**
- V_AIN0 = VBAT / 2
- With gain 1/6: ADC can measure up to 3.6V
- Max battery voltage: 7.2V (sufficient for single Li-Ion/LiPo)

**Why 100kΩ resistors?**
- Low current drain: I = VBAT / 200kΩ ≈ 20µA @ 4V
- High impedance won't affect battery
- Standard resistor values

## Configuration

### Enable ADC in prj.conf
```properties
CONFIG_ADC=y
CONFIG_ADC_NRFX_SAADC=y
```

### Device Tree Overlay
File: `boards/nrf54l15dk_nrf54l15_cpuapp.overlay`

For **internal VDD measurement**:
```dts
&adc {
    channel@0 {
        zephyr,input-positive = <NRF_SAADC_VDD>;
    };
};
```

For **external battery (P0.05)**:
```dts
&adc {
    channel@0 {
        zephyr,input-positive = <NRF_SAADC_AIN1>;  /* P0.05 - Arduino A1 */
    };
};
```

**Alternative pins if P0.05 is busy:**
- AIN2 (P0.06) - Arduino A2
- AIN3 (P0.07) - Arduino A3

**⚠️ DO NOT USE:**
- AIN0 (P0.04) - Used for Button 3!

## Usage

### Shell Commands
```bash
# Check battery status
uart:~$ chat battery
Battery Status:
  Voltage: 3847 mV
  Percentage: 92%
  Status: GOOD [████████░░]

# Battery info appears in metrics automatically
uart:~$ chat metrics_to 0x0005
# Metrics will include real battery percentage
```

### API Usage in Code
```c
#include "battery_adc.h"

// Initialize (done automatically in model init)
int err = battery_adc_init();

// Read voltage
uint16_t voltage_mv;
battery_adc_read_mv(&voltage_mv);

// Get percentage directly
uint8_t percent = battery_get_percentage();
```

## Voltage-to-Percentage Mapping

Based on typical Li-Ion/LiPo discharge curve:

| Voltage | Percentage | Status |
|---------|------------|--------|
| 4.2V    | 100%       | Full   |
| 3.7V    | 80%        | Good   |
| 3.3V    | 20%        | Low    |
| 3.0V    | 5%         | Critical |
| <2.7V   | 0%         | Dead   |

## Calibration

If readings are inaccurate, adjust the voltage calculation in `battery_adc.c`:

```c
// Current formula for gain 1/6, internal 0.6V reference:
val_mv = (sample_buffer * 600 * 6) / 4095;

// If using voltage divider with R1=R2 (divide by 2):
val_mv = (sample_buffer * 600 * 6 * 2) / 4095;
```

## Testing Without Hardware

If ADC init fails, the code automatically falls back to simulated battery:
- Starts at 100%
- Drains 1% per 10 minutes of uptime
- Good for testing metrics functionality

## Troubleshooting

### ADC init failed
```
Battery ADC init failed: -19, will use simulated values
```

**Solutions:**
1. Check `CONFIG_ADC=y` in prj.conf
2. Verify overlay file exists for your board
3. Check P0.04 is not used by other peripherals
4. Try internal VDD measurement first

### Voltage reads 0 or very high
1. Check voltage divider connections on **P0.05 (Arduino A1)**
2. Verify correct AIN pin in overlay (should be AIN1, not AIN0)
3. Test with multimeter on P0.05 pin (Arduino A1 header)
4. Should read VBAT/2 (around 1.5-2.1V for 3-4.2V battery)

### Button 3 not working
If you accidentally used P0.04:
1. Change overlay to use AIN1 (P0.05)
2. Rebuild and reflash
3. Button 3 should work again

### Percentage stuck at 85%
ADC not initialized, using fallback value. Check logs for init errors.

## Power Consumption

- ADC active: ~200µA during measurement
- Voltage divider (100kΩ): ~20µA continuous
- Total impact: Minimal (<1% of BLE Mesh power)

ADC is only active during measurement (< 1ms), so average power is negligible.
