# AlfieTempMonitor

An Arduino-based temperature monitor for Alfie's enclosure.

## Hardware

| Pin  | Connection |
|------|------------|
| A0   | TMP36 Vout (temperature sensor) |
| D1   | Bi-colour LED anode A (green) |
| D2   | Bi-colour LED anode B (red) |
| D3   | Fan-toggle button (one leg; other leg to D4) |
| D4   | Fan-toggle button ground rail (OUTPUT LOW) |
| D8   | Threshold ▲ button (other leg to GND) |
| D9   | Threshold ▼ button (other leg to GND) |
| D11  | Fan PWM control (MOSFET gate) |
| SDA/SCL | 16×2 I²C LCD (address 0x27) |

## Controls

- **Threshold ▲ / ▼ (D8 / D9):** Raise or lower the temperature threshold by 1 °C per press.  
  Hold for continuous adjustment (slow → fast acceleration).  
  The new threshold is displayed for 2 seconds.

- **Fan toggle (D3 ↔ D4):** Press once to turn the fan **on**; press again to turn it **off**.  
  The fan operates independently of the LED — toggling the fan does not affect the LED colour, and a temperature change does not affect the fan.

## LED colours

| Colour | Meaning |
|--------|---------|
| Green  | Temperature is at or below threshold |
| Red    | Temperature is above threshold |
