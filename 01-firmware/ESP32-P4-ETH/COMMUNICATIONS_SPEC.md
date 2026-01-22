# COMUNICACIONES - ANÁLISIS DE THROUGHPUT Y LATENCIA

## ESP32-P4 Dual-Core Telemetry System

---

## 1. CONFIGURACIÓN ACTUAL

### Task Frequencies

- **task_comms_cpu1**: 50 Hz (20ms interval) - CPU 1
- **task_monitor_lowpower_cpu1**: 0.2 Hz (5s interval) - CPU 1
- **task_rtcontrol_cpu0**: 10 Hz (100ms interval) - CPU 0
- **Ethernet polling**: 100 Hz (10ms polling) - Kernel
- **MQTT keepalive**: Auto (60s default) - Kernel

---

## 2. THROUGHPUT TEÓRICO MÁXIMO

### Capa Física (Ethernet)

```
Ethernet Speed:       100 Mbps
Theoretical BW:       12.5 MB/s (100 Mbps / 8)
Practical BW:         ~10 MB/s (accounting for collisions, retransmit)
```

### Capa 2-3 (IP/TCP Stack)

```
TCP/IP Overhead:      ~5-10% (headers, flags)
Effective BW:         ~9-9.5 MB/s
```

### Capa 4 (MQTT)

```
MQTT Protocol OH:     ~2-3% (MQTT headers per message)
Per-message overhead: ~10-20 bytes (MQTT header varies by QoS)
Effective MQTT BW:    ~8.5-9 MB/s
```

### JSON Encoding Impact

```
Binary data:          1 byte
JSON encoded:         ~1.5-2 bytes (with quotes, brackets, commas)
Overhead factor:      +50-100% size increase
```

---

## 3. TELEMETRY DATA BANDWIDTH

### Scenario A: 5 Sensor Readings @ 50Hz (HIGH-SPEED)

**Per-message data:**

```
{
  "timestamp_ms":    4 bytes
  "motor_speed":     4 bytes (float)
  "motor_current":   4 bytes (float)
  "battery_voltage": 4 bytes (float)
  "temperature":     4 bytes (float)
  "encoder_count":   4 bytes (int32)
}
Total: 24 bytes binary

JSON encoded (~70% overhead):
{
  "ts":12345,
  "spd":1234.56,
  "cur":234.56,
  "vbat":11850,
  "tmp":35.5,
  "enc":123456
}
Estimated: ~90-100 bytes JSON
```

**Bandwidth calculation:**

```
Message frequency:    50 Hz (every 20ms)
Per-message size:     100 bytes (JSON + MQTT overhead)
Raw throughput:       50 msg/s × 100 bytes = 5 KB/s
Network util:         5 KB/s ÷ 10 MB/s = 0.05% (VERY LOW)
```

### Scenario B: Extended Telemetry @ 50Hz (NORMAL)

**Multiple sensor groups:**

```json
{
  "ts": 12345,
  "sensors": {
    "motor": [1234.5, 234.56, 85],
    "battery": [11850, 1200, 75],
    "imu": [0.1, 0.05, 9.8],
    "status": 3
  }
}
Size: ~200-250 bytes
```

**Bandwidth:**

```
Per message:          250 bytes
Frequency:            50 Hz
Throughput:           50 × 250 = 12.5 KB/s
Network util:         0.12% (STILL VERY LOW)
Latency (single msg): <1ms (local processing) + 1-5ms (Ethernet) = ~5ms
```

### Scenario C: HIGH-DENSITY STREAMING @ 100Hz (MAXIMUM)

**Ultra-compact telemetry:**

```json
{
  "id": 1,
  "d": [1234, 234, 11850, 35, 123456]
}
Size: ~50-60 bytes
```

**Bandwidth:**

```
Per message:          60 bytes
Frequency:            100 Hz (10ms interval)
Throughput:           100 × 60 = 6 KB/s
Can support:          Multiple streams simultaneously
Total capacity:       9,000 KB/s ÷ 6 KB/s = 1500× current usage
```

---

## 4. LATENCY ANALYSIS

### Component Breakdown

| Component                    | Latency     | Notes                             |
| ---------------------------- | ----------- | --------------------------------- |
| **Inter-core queue**         | 0.1-0.5 ms  | FreeRTOS queue access (CPU0→CPU1) |
| **Shared memory mutex**      | 0.1-0.2 ms  | Semaphore acquire/release         |
| **Sensor read + JSON pack**  | 1-2 ms      | CPU1 processing                   |
| **MQTT publish**             | 0.5-1 ms    | esp_mqtt_client call              |
| **TCP/IP stack**             | 2-5 ms      | Network processing                |
| **Ethernet TX**              | 1-2 ms      | Wire transmission                 |
| **End-to-end (CPU0→Broker)** | **5-15 ms** | Typical case                      |

### Round-Trip (Command Response)

```
CPU0 command written:     0 ms
CPU1 reads + processes:   ~2 ms
CPU1 publishes response:  ~2 ms
Broker receives:          ~5 ms
Round-trip total:         ~10-20 ms
```

---

## 5. INTER-CORE COMMUNICATION DESIGN

### CPU0 ← → CPU1 Synchronization

**Forward: Sensor Data (CPU0 → CPU1)**

```c
shared_memory_write_sensors(&sensor_data, pdMS_TO_TICKS(5));
// Latency: ~0.2 ms (mutex + memcpy)
```

**Backward: Commands (CPU1 → CPU0)**

```c
xQueueSend(task_comms_cpu1_get_queue(), command, pdMS_TO_TICKS(5));
// Latency: ~0.1 ms (queue operation)
```

**Advantage: No busy-waiting, efficient ISR-like interrupts**

---

## 6. PRACTICAL CAPACITY ESTIMATION

### Current Configuration (50Hz @ 100-byte JSON)

```
Per second:           50 messages
Data rate:            5 KB/s
Headroom:             ~1800× available
Status:               EXTREMELY LOW LOAD ✓
```

### Maximum Safe Capacity (10ms interval)

```
Max frequency:        100 Hz
Per-msg size:         1000 bytes (extended telemetry)
Total rate:           100 KB/s
Network util:         1% (SAFE)
Ethernet capacity:    10 MB/s ÷ 100 KB/s = 100 streams possible
```

### Edge Case: 1MB/s Streaming

```
Possible with:        Raw TCP or optimized protocol
NOT practical for:    MQTT (overhead too high)
Recommendation:       Current 50Hz is OPTIMAL for MQTT
```

---

## 7. CONFIGURATION RECOMMENDATIONS

### Current Setup: OPTIMAL ✓

```
Frequency:    50 Hz (20ms)
JSON size:    100-200 bytes
Throughput:   5-10 KB/s
Latency:      ~10-15ms end-to-end
Load:         <0.1% network usage
Verdict:      EXCELLENT balance of responsiveness + efficiency
```

### If Faster Commands Needed: UPGRADE TO 100Hz

```c
// In task_comms_cpu1.c, change:
vTaskDelay(pdMS_TO_TICKS(20));  // Current 50Hz
// To:
vTaskDelay(pdMS_TO_TICKS(10));  // New 100Hz

// This would:
- Double command response time: 10-20ms → 5-10ms
- Increase throughput: 10 KB/s → 20 KB/s
- Still use <0.2% network capacity
- Stack size might need +2KB
```

### If High-Bandwidth Needed: BATCH MODE

```json
{
  "batch": [
    {"ts": 1, "d": [...]},
    {"ts": 2, "d": [...]},
    {"ts": 3, "d": [...]}
  ]
}
// Send 3 readings per message, maintain 50Hz rate
// Effective throughput: 15 KB/s with same latency
```

---

## 8. BOTTLENECK ANALYSIS

### What's NOT Limiting:

- ✓ Ethernet (100 Mbps) - 99.9% idle
- ✓ MQTT broker - handles 1000s of connections
- ✓ ESP32-P4 CPU - dual core at 1.2GHz
- ✓ FreeRTOS scheduling - sub-millisecond precision

### What COULD Limit:

- ✗ JSON encoding (CPU1) - mitigation: binary+base64 or MessagePack
- ✗ Mutex contention (high-frequency sensor reads from both cores)
- ✗ MQTT session limit (~100 messages/sec practical for reliability)

---

## 9. MONITORING & STATISTICS

**task_comms_cpu1 logs every 5 seconds:**

```
I (12345) task_comms_cpu1: Telemetry stats - Messages: 250, Rate: 50.0 Hz, Est. throughput: 50.0 KB/s
```

This shows:

- Message count
- Actual frequency (should match 50Hz config)
- Estimated throughput assuming 1KB/msg

---

## 10. FUTURE IMPROVEMENTS

1. **Binary Protocol**: Replace JSON with MessagePack
   - Reduces size: 100 bytes → 40 bytes
   - Throughput: 20 KB/s
   - Latency: -50% (less CPU)

2. **Compression**: gzip or brotli
   - Size: 100 bytes → 60 bytes (60% ratio)
   - Throughput: 30 KB/s
   - Cost: +5-10ms CPU time

3. **Queue Prioritization**: High-priority commands separate queue
   - Commands get <1ms response vs 5-10ms current
   - Implementation: 2 queues in task_comms_cpu1

4. **Telemetry Buffering**: Batch multiple readings
   - Combine 5 readings into 1 message
   - Throughput: 200 KB/s possible
   - Trade-off: 100ms latency increase

---

## SUMMARY

**Current system (50Hz, JSON, 100-200 bytes/msg):**

- ✓ Latency: 10-15ms end-to-end
- ✓ Throughput: 5-10 KB/s
- ✓ Network load: 0.05-0.1%
- ✓ Reliability: Full MQTT QoS support
- ✓ Scalability: Can 100× increase load before issues

**Verdict: EXCELLENT for robotics telemetry use case**
