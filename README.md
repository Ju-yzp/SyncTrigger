# SyncTrigger

A lightweight, **header-only** C++ synchronization framework designed for multi-sensor systems (e.g., Camera, IMU, LiDAR). It ensures high-performance data alignment driven by a primary trigger sensor.

---

<img src="./docs/demo.gif" width="600" alt="demo">

## Key Features
* **⚡ Header-Only**: Zero compilation required. Integration is as simple as including a header file.
* **🎯 Trigger-Based Alignment**: Uses the **Primary Sensor** as the master clock to drive data fusion across all auxiliary sensors.
* **🔄 Smart Retry (Two-Strike Policy)**:
    * **First Strike**: If auxiliary data is missing, the frame is buffered for one retry cycle.
    * **Second Strike**: If sync fails again on the next signal, the stale frame is dropped to prevent pipeline stall and maintain liveness.
* **🛠 Type-Safe & Flexible**: Leverages C++20 variadic templates to support an arbitrary number of sensors and data types.
* **🚀 Real-Time Optimized**: Built for low-latency Linux environments using non-blocking I/O (`eventfd` and `poll`).

---

## Design Logic

The synchronizer operates as a deterministic state machine:

1.  **Poll**: Wait for the **Primary Sensor** (Trigger) to push new data.
2.  **Check**: Verify if all auxiliary buffers contain data matching the Trigger's timestamp.
3.  **Process or Defer**:
    * **Ready**: Extract all synchronized data and dispatch a complete `DataPackage`.
    * **Pending (1st fail)**: Hold the frame and wait for the next cycle.
    * **Drop (2nd fail)**: Discard the stale trigger frame to ensure system throughput.

---

## Future Roadmap

* **Policy-Based Tuning**: Toggle between "Strict Sync" and "Best-Effort Interpolation."
* **Lock-Free Optimization**: Minimizing mutex contention to further reduce jitter in high-frequency systems.

---

## Requirements

* **Standard**: C++20 or higher.
* **Platform**: Linux (requires `eventfd` and `poll`).
