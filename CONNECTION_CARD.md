# 🔌 DUAL MPU6050 CONNECTION CARD

## Quick Reference - Print & Keep!

```
╔═══════════════════════════════════════════════════════════╗
║         DUAL MPU6050 ESP32 WIRING REFERENCE               ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  ESP32 PIN  │  MPU #1 (0x68)  │  MPU #2 (0x69)          ║
║  ──────────────────────────────────────────────          ║
║  3.3V       │     VCC         │     VCC                  ║
║  GND        │     GND         │     GND                  ║
║  GPIO21     │     SDA         │     SDA                  ║
║  GPIO22     │     SCL         │     SCL                  ║
║  GND        │     AD0         │     ─                    ║
║  3.3V       │     ─           │     AD0                  ║
║             │                 │                           ║
║  ⚡ CRITICAL: Different AD0 connections!                 ║
║     MPU #1 AD0 → GND    (Address 0x68)                   ║
║     MPU #2 AD0 → 3.3V   (Address 0x69)                   ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║                   CONTROL MAPPING                         ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║   MPU #1 (Right Hand - Address 0x68)                     ║
║   ┌────────────────────────────┐                         ║
║   │  Tilt LEFT/RIGHT  → Roll   │                         ║
║   │  Tilt FORWARD/BACK → Pitch │                         ║
║   └────────────────────────────┘                         ║
║                                                           ║
║   MPU #2 (Left Hand - Address 0x69)                      ║
║   ┌──────────────────────────────┐                       ║
║   │  Tilt LEFT/RIGHT  → Throttle │                       ║
║   │  Tilt FORWARD/BACK → Yaw     │                       ║
║   └──────────────────────────────┘                       ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║                  VERIFICATION STEPS                       ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  □ 1. Upload i2c_scanner.ino                             ║
║  □ 2. Verify "Found device at 0x68"                      ║
║  □ 3. Verify "Found device at 0x69"                      ║
║  □ 4. Upload esp32_combined.ino                          ║
║  □ 5. Keep sensors STILL during calibration              ║
║  □ 6. Test each axis independently                       ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║              TROUBLESHOOTING CHECKLIST                    ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  Can't find 0x68:                                         ║
║    ✓ Check MPU #1 AD0 connected to GND                   ║
║    ✓ Verify power & I2C connections                      ║
║                                                           ║
║  Can't find 0x69:                                         ║
║    ✓ Check MPU #2 AD0 connected to 3.3V                  ║
║    ✓ Ensure it's not touching GND                        ║
║                                                           ║
║  Jittery readings:                                        ║
║    ✓ Keep wires short (<20cm)                            ║
║    ✓ Reduce I2C speed to 100kHz                          ║
║    ✓ Add 4.7kΩ pull-ups on SDA/SCL                       ║
║    ✓ Recalibrate with sensors still                      ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║                CONFIGURATION VALUES                       ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  Receiver MAC (line 24):                                  ║
║  uint8_t receiverMAC[] = {0x__, 0x__, ...};              ║
║                                                           ║
║  Sensitivity (lines 45-48):                               ║
║  #define ROLL_SCALE 30.0     ← Adjust sensitivity        ║
║  #define PITCH_SCALE 30.0    ← Higher = more sensitive   ║
║  #define THROTTLE_SCALE 30.0 ← Lower = less sensitive    ║
║  #define YAW_SCALE 30.0                                   ║
║                                                           ║
║  Update rate (line 44):                                   ║
║  #define UPDATE_RATE_MS 20   ← 50Hz (20ms cycles)        ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║                 I2C BUS DIAGRAM                           ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║            ESP32                                          ║
║       ┌──────────────┐                                    ║
║   3.3V│              │                                    ║
║   GND │              │                                    ║
║  GPIO21(SDA)         │                                    ║
║  GPIO22(SCL)         │                                    ║
║       └──┬───┬───┬───┘                                    ║
║          │   │   │                                        ║
║      ┌───┴┐  │  ┌┴───┐                                   ║
║      │VCC │  │  │VCC │                                    ║
║      │GND │  │  │GND │                                    ║
║      │SDA │◄─┴──┤SDA │                                    ║
║      │SCL │◄────┤SCL │                                    ║
║   ┌──┤AD0 │     │AD0 ├──┐                                ║
║  GND │    │    3.3V   │  │                               ║
║      └────┘     └─────┘  │                               ║
║      MPU#1       MPU#2   │                               ║
║      0x68        0x69    │                               ║
║                          │                                ║
║  ⚡ KEY DIFFERENCE! ─────┘                                ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║               EXPECTED SERIAL OUTPUT                      ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  Initializing MPU6050 #1 (0x68 - Roll/Pitch)...         ║
║  MPU6050 #1 OK                                            ║
║  Initializing MPU6050 #2 (0x69 - Throttle/Yaw)...       ║
║  MPU6050 #2 OK                                            ║
║  Calibrating sensors... Keep both MPUs still!             ║
║  Calibration complete!                                    ║
║  Transmitter MAC: XX:XX:XX:XX:XX:XX                      ║
║  ESP-NOW Ready                                            ║
║                                                           ║
║  === DUAL MPU6050 SYSTEM READY ===                       ║
║  MPU1 (0x68): Roll & Pitch                               ║
║  MPU2 (0x69): Throttle & Yaw                             ║
║  ================================                         ║
║                                                           ║
║  DUAL:-15.2,8.4,50.0,12.1,...                            ║
║        ^^^^^ ^^^ ^^^^ ^^^^                               ║
║        Roll  Pit Thr  Yaw                                ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║                   SPECIFICATIONS                          ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  Update Rate:     50 Hz (20ms intervals)                  ║
║  I2C Speed:       400 kHz (can reduce to 100 kHz)        ║
║  Power Draw:      ~10 mA (both MPUs)                     ║
║  Data Size:       52 bytes per packet                    ║
║  ESP-NOW Range:   ~200 meters (line of sight)            ║
║  Control Range:   Roll/Pitch/Yaw: -100 to +100          ║
║                   Throttle: 0 to 100                     ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║                  DOCUMENTATION FILES                      ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  📘 DUAL_MPU_WIRING.md     - Complete wiring guide       ║
║  🚀 DUAL_MPU_QUICKSTART.md - Quick setup instructions    ║
║  📋 DUAL_MPU_SUMMARY.md    - Comprehensive reference     ║
║  🔍 i2c_scanner.ino        - Hardware verification tool  ║
║  ⭐ esp32_combined.ino     - Main dual MPU code          ║
║                                                           ║
╠═══════════════════════════════════════════════════════════╣
║                SUPPORT & RESOURCES                        ║
╠═══════════════════════════════════════════════════════════╣
║                                                           ║
║  I2C Address Info:                                        ║
║    • MPU6050 default address: 0x68                       ║
║    • When AD0 = HIGH: address becomes 0x69               ║
║    • This allows 2 sensors on same I2C bus               ║
║                                                           ║
║  Power Requirements:                                      ║
║    • MPU6050: 3.3V @ 3-5mA each                          ║
║    • ESP32: 3.3V @ 80-240mA                              ║
║    • USB power is sufficient for both                    ║
║                                                           ║
║  Pin Functions:                                           ║
║    • VCC: Power supply (3.3V)                            ║
║    • GND: Ground reference                               ║
║    • SCL: I2C clock line                                 ║
║    • SDA: I2C data line                                  ║
║    • AD0: Address select (LOW=0x68, HIGH=0x69)          ║
║    • INT: Interrupt (not used)                           ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝

Version: 2.0       Date: 2026-02-08       Platform: ESP32

┌────────────────────────────────────────────────────────┐
│  ⚡ REMEMBER: AD0 pins MUST be different!              │
│     MPU #1 → GND    (0x68)                             │
│     MPU #2 → 3.3V   (0x69)                             │
│  This is what allows both sensors to work together!    │
└────────────────────────────────────────────────────────┘

🚁 For detailed setup: See DUAL_MPU_QUICKSTART.md
📘 For troubleshooting: See DUAL_MPU_WIRING.md  
📋 For full reference: See DUAL_MPU_SUMMARY.md
```

---

**Print this card and keep it handy during setup!**
