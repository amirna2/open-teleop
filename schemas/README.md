# Open-Teleop Interface Schemas

This directory contains FlatBuffers schema definitions that serve as contracts between ROS2 bridge nodes and the Go controller.

## Schemas

- `diagnostics.fbs` - System diagnostics data schema
- `teleop.fbs` - Teleoperation commands schema (future)
- `video.fbs` - Video streaming metadata schema (future)

## Usage

These schemas are used to generate code for both Python (ROS2) and Go (Controller) sides.
Run the `scripts/generate_interfaces.sh` script to generate updated code after making changes.

## Versioning

When making changes to schemas:
1. Try to only add new fields (backward compatible)
2. Document breaking changes in CHANGELOG.md
3. Consider versioning schemas if breaking changes are necessary 