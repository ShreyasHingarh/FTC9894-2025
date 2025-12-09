2025 FTC Robot Code: 

This repository contains the robot control software for the 2025 FTC season. The codebase is built around the Road Runner Actions framework and is organized into modular subsystems shared between autonomous and tele-operated operation. All routines are designed to be stateful, non-blocking, and easily extensible.

Autonomous Mode
- Motion Planning
- Powered by the Road Runner library for high-precision trajectory generation.
- Autonomous routines built using the Actions API for clean sequencing and parallel action execution.
- Odometry and encoder feedback used for all autonomous movement.
- Architecture
- Unified subsystem model allows reuse of sorting, alignment, and sensing logic across autonomous and tele-op.
- All logic is fully non-blocking to maintain consistent scheduling and avoid stalls.

Sorter Subsystem
- Automatic Intake & Sorting: A macro sequences artifact intake and organizes game pieces for launching.
- Automatic Firing:
- Prioritizes launching artifacts in motif order when possible.
- Falls back to standard ordered firing when motif sequencing is not available.
- Fail-Safes: Built-in procedures allow the sorter to re-zero during a match if desynchronized.

Sensor Integration
- Camera: AprilTag-based autoAlign function enables precise alignment to scoring positions.
- Color Sensors: Two color sensors at different angles improve reliability of ball-color detection in the sorter.
- Encoders: Provide position feedback for all autonomous movement and correction routines.

Future Development
- Integrate an additional camera using OpenCV or an object-detection model to determine the optimal firing order based on trough contents.
- Add a dedicated color sensor for automatic sorter re-zeroing.-
- Implement variable launch speeds and bring AprilTag-based auto-alignment into tele-op.
