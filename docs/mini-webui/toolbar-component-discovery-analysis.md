# Toolbar Component Discovery Analysis

## Problem Statement

The mini-web-ui requirements specify a toolbar with "Add Component" functionality (FR-3.2.2 - FR-3.2.4), but implementation faces fundamental architectural challenges around component discovery and data presentation.

## Configuration Analysis

### Available Config API
- **Endpoint**: `GET /api/v1/config/teleop` 
- **Response**: Raw YAML configuration
- **Legacy Usage**: Already used by existing config editor

### Config Structure Example
```yaml
topic_mappings:
  # Video streams with encoder params
  - ros_topic: "/camera/image_raw"
    ott: "teleop.video.main_camera"
    message_type: "sensor_msgs/msg/Image"
    encoder_params:
      encoding_format: "video/h264"
      width: 640
      height: 480

  # Telemetry data  
  - ros_topic: "/robot/battery_state"
    ott: "teleop.sensor.battery_state"
    message_type: "sensor_msgs/msg/BatteryState"
    direction: "OUTBOUND"

  # Control interfaces
  - ros_topic: "/robot/cmd_vel"
    ott: "teleop.control.velocity"
    message_type: "geometry_msgs/msg/Twist"
    direction: "INBOUND"

media_mappings:
  video:
    - device_id: "video0"
      ott: "teleop.video.navigation_camera"
      encoding_format: "video/h264"
```

## Discovery Challenges

### 1. Component Type Inference Problem
**Challenge**: No explicit "component type" field in config

**Current Approach (Assumptions)**:
- Video components: `message_type` contains "Image" OR has `encoder_params`
- Telemetry components: `direction: "OUTBOUND"` + non-Image message types
- Control components: `direction: "INBOUND"`

**Issues**:
- Fragile heuristics that could break with config changes
- No guarantee about component UI capabilities
- Mixed data types (e.g., compressed images, raw sensor data)

### 2. Data Presentation Problem
**Challenge**: Frontend needs to know how to render each component type

**Example**: `sensor_msgs/msg/BatteryState`
```json
{
  "header": {...},
  "voltage": 12.6,
  "current": -0.5,
  "charge": 0.85,
  "capacity": 100.0,
  "design_capacity": 100.0,
  "percentage": 85.0,
  "power_supply_status": 2,
  "power_supply_health": 1
}
```

**Questions**:
- How does frontend know to display this as battery gauge vs. raw JSON?
- What about `nav_msgs/msg/Odometry` - should it be a position display, velocity graph, or raw data?
- How to handle unknown message types?

### 3. Component-to-Stream Mapping Problem
**Challenge**: Frontend receives data via WebSocket/API but needs to associate it with components

**Current Flow**:
1. Config defines: `ros_topic: "/robot/battery_state"` → `ott: "teleop.sensor.battery_state"`
2. Backend processes and sends data with some identifier
3. Frontend receives data and needs to route it to correct component

**Missing Link**: How does frontend map received data to the right component instance?

## Possible Solutions

### Option 1: Component Registry Approach
- Define explicit component types in config or separate registry
- Each type specifies UI rendering template
- Frontend gets both config AND rendering instructions

### Option 2: Dynamic Component Discovery
- Backend exposes `/api/v1/components/available` endpoint
- Returns processed list with rendering hints:
```json
{
  "video_streams": [
    {"id": "main_camera", "name": "Main Camera", "ott": "teleop.video.main_camera"}
  ],
  "telemetry": [
    {"id": "battery", "name": "Battery Status", "ott": "teleop.sensor.battery_state", "render_type": "battery_gauge"}
  ]
}
```

### Option 3: Configuration Editor First
- Implement config editor to understand data flow
- Learn from editing experience what component metadata is needed
- Design component system based on deeper config understanding

## Recommendations

### Immediate: Focus on Configuration Editor
**Rationale**:
1. **Known Requirements**: Config editor has clear, existing functionality to port
2. **Learning Opportunity**: Will reveal config structure nuances and data flow patterns
3. **Foundation**: Understanding config deeply is prerequisite for component discovery
4. **Incremental Progress**: Achievable milestone vs. speculative component system

### Future: Component System Design
After config editor implementation:
1. **Analyze Data Flow**: Understand how backend processes config into frontend data
2. **Define Component Schema**: Create explicit component type definitions
3. **Design Presentation Layer**: Map ROS message types to UI components
4. **Implement Discovery**: Build robust component discovery based on learned patterns

## Next Steps

1. **Implement Configuration Editor** in Svelte mini-web-ui
2. **Document Data Flow Patterns** discovered during implementation
3. **Design Component Type System** based on real config editing experience
4. **Return to Toolbar Implementation** with solid foundation

## Questions for Further Discussion

1. Should component types be defined in config, or as separate metadata?
2. How should frontend handle unknown/new message types?
3. What level of UI customization is needed for different telemetry types?
4. Should components be user-customizable or predefined templates?

---
*Analysis Date: June 8, 2025*  
*Recommendation: Implement Configuration Editor first, then revisit component discovery*