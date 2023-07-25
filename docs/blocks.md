# Blocks

- 4ms to stabilize
- Termination required for Tx and Rx

## Transmitter

- Output high is maximum of 100mV above receiving chiplet's receiver frontend circuit power supply rail

### Electrical Parameters

| Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- |
| Data Lane TX Swing | 0.4 | | | |
| Fwd Clock Tx Swing (single ended) | 0.4 | | | |
| Incoming Clock Slew Rate | 0.1 | 0.22 | 0.25 | UI |
| Incoming Differential Clock Overlap | | | 30 | mUI |
| Incoming Data Slew Rate | | 0.35 | | UI |
| Driver Pull-up/down Impedance | 27 | 30 | 33 | Ohms |
| Impedance Step Size | | | 0.5 | Ohms |
| Pull-up/down Delay Mismatch | | | 20 | mUI |
| 1-UI Total Jitter | | | 96/113 | mUI pk-pk |
| 1-UI Deterministic Jitter (Dual Dirac) | | | 48 | mUI pk-pk |
| Duty Cycle Error  | -0.02 | | 0.02 | UI |
| Lane-to-Lane Skew Correction Range (up to 16 GT/s) | -0.14 | | 0.14 | UI |
| Lane-to-Lane Skew | -0.02 | | 0.02 | UI |
| Clock to Mean Data Training Accuracy | -0.07 | |  0.07 | UI |
| Phase Adjustment Step | | | 16 | mUI |
| TX Pad Capacitance (8 GT/s capable design) | | | 300 | fF |
| TX Pad Capacitance (16 GT/s capable design) | | | 200 | fF |

### Equalization

Recommended for 16 GT/s. Should only use de-emphasis (?). Equalization coefficient is subject to maximum unity swing constraint. Support setting with and without de-emphasis.

### Sub-blocks

#### FIFO

#### Serialize

#### TX Driver

#### De-Skew

- Need per lane deskew for higher data rates

#### PI + DCC

#### DLL

#### PLL

## Receiver

### Electrical Parameters

| Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- |
| Rx Input Impedance | 45 | 50 | 55 | Ohms |
| Impedance Step Size | | | 1 | Ohms |
| Data/Clock Total Differential Jitter | | | 0.03 | UI |
| Lane-to-Lane Skew (up to 16 GT/s) | -0.07 | | 0.07 | UI |
| Lane-to-Lane Skew (> 16 GT/s) | -0.12 | | 0.12 | UI |
| Phase Error | -0.04 | | 0.04 | UI |
| Per Lane De-Skew Adjustment Step | | | 16 | mUI |
| Output Rise Time | | | 0.1 | UI |
| Output Fall Time | | | 0.1 | UI |
| Rx Pad Capacitance (up to 8 GT/s) | | | 300 | fF |
| Rx Pad Capacitance (up to 16 GT/s) | | | 200 | fF |
| Rx Voltage Sensitivity | | | 40 | mV |


### Equalization

Optional for 24 GT/s and 32 GT/s data rates, expected capability is equivalent of 1st order CTLE.

### Sub-blocks

#### Flop

#### De-Skew

#### FIFO

#### Phase Gen

#### Track

## Clocking

- Forwarded clock must have two phases (90 and 270) at a frequency of half the data rate
- Requires eye height of 40 mV and eye width of 0.75 UI

### Noise and Skew

| Parameter | Min | Nom | Max | Unit | |
| --- | --- | --- | --- | --- | --- |
| I/O Vcc noise | | | 80 | mVpp | at 8 Gb/s |
| I/O Vcc noise | | | 40-50 | mVpp | at 16 Gb/s |
| I/O Vcc noise | | | 30 | mVpp | at 32 Gb/s |

## Interconnect

- Should be designed with 50 ohm characteristic impedance

### Loss and Crosstalk

#### With Rx Termination

| Data Rate | 4, 8 Gb/s | 12, 16 Gb/s |
| --- | --- | --- |
| VTF Loss (dB) | L(0) > -4.5 <br/> L(f\_N) > -7.5 | L(0) > -4.5 <br/> L(f\_N) > -6.5 | 
| VTF Crosstalk (dB) | XT(f\_N) < min(3 * L (f\_N) - 11.5, -25) | XT(f\_N) < min(3 * L (f\_N) - 11.5, -25) | 

#### Without Rx Termination

| Data Rate | 4, 8 Gb/s | 12, 16 Gb/s |
| --- | --- | --- |
| VTF Loss (dB) | L(f\_N) > -1.25 | L(f\_N) > -1.15 | 
| VTF Crosstalk (dB) | XT(f\_N) < min(7 * L (f\_N) - 12.5, -15) | XT(f\_N) < min(4 * L (f\_N) - 13.5, -17) | 

## Raw BER Requirements

| Data Rate (GT/s) | 4 | 8 | 12 | 16 |
| --- | --- | --- | --- |  --- | 
| BER | 1E-27 | 1E-27 | 1E-15 | 1E-15 |

## Valid and Clock Gating

During idle state:
- Data lanes must send the last UI for at least 1 UI and up to 8 UIs and then Hi-Z
- Valid lane must be held low
- Clock idle state level must alternate between differential high and differential low during consecutive clock gating events
- Must precondition data lanes to a 0 or 1, block must drive differential low for at least 1 UI or up to a maximum of 8 UIs before normal transmission

## Sideband

### Electrical Parameters

| Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- |
| Supply voltage (VCCAON) | 0.65 | | | V |
| TX Swing | 0.8 * VCCAON | | | V |
| Input High Voltage | 0.7 * VCCAON | | | V |
| Input Low Voltage | | | 0.3 * VCCAON | V |
| Output High Voltage | 0.9 * VCCAON | | | V |
| Output Low Voltage | | | 0.1 * VCCAON | V |
| Sideband Data Setup Time | 200 | | | ps |
| Sideband Data Hold Time | 200 | | | ps |

