# Web UI Configuration Editor Prototype Design

## 1. Goal

To replace the existing raw YAML text area in the Web UI's Configuration tab with a structured, dynamic form builder. This aims to improve user experience, reduce errors, and make editing the operational configuration (`open_teleop_config.yaml`) more intuitive.

## 2. Chosen Approach: Structured Form

A single-page form divided into logical sections corresponding to the major parts of the `open_teleop_config.yaml` file. Standard form elements will be used, with dynamic behavior for list management and conditional fields.

## 3. UI Sections Design

### 3.1. Read-Only Metadata

*   **Purpose:** Display key identifying information about the configuration file.
*   **Fields:**
    *   `version`
    *   `config_id`
    *   `lastUpdated`
    *   `robot_id`
*   **UI:** Displayed as simple text labels and values at the top of the page. Not editable through this interface.

### 3.2. Topic Mappings (`topic_mappings`)

*   **Purpose:** Manage the list of ROS topic mappings.
*   **UI Structure:** A dynamic list where each mapping is presented in a visually distinct block, potentially collapsible for better overview.
*   **Fields per Mapping:**
    *   `ros_topic`: (Future: Searchable dropdown/autocomplete populated by ROS Topic Discovery API). Currently: Text input.
    *   `ott`: Text input.
    *   `direction`: Dropdown (`INBOUND`, `OUTBOUND`).
    *   `priority`: Dropdown (`HIGH`, `STANDARD`, `LOW`).
    *   `message_type`: Dropdown populated with known ROS message types. (Future: Populate from a dedicated API endpoint reflecting available types in the ROS environment). Triggers conditional field display.
    *   `source_type`: Dropdown (`ROS2_CDR`, potentially others).
*   **Conditional Fields:** Based on the selected `message_type`, additional fields will appear within the mapping's block.
    *   *Example (sensor_msgs/msg/Image):* `Resolution` (Dropdown), `Frame Rate` (Number input).
    *   *Example (sensor_msgs/msg/CompressedImage):* `Compression Quality` (Number input/Slider).
*   **Interaction:**
    *   `[Add New Mapping]` button: Appends a new blank mapping form block to the list or opens a modal dialog for editing the new mapping.
    *   `[Remove]` button: Per mapping block.
    *   Expand/Collapse (`[v]`/`[^]`): Optional per mapping block for better navigation if the list grows long.
*   **Visual Concept:**
    ```
    == Topic Mappings ==
    [-----------------------------------------------------------------------------]
    | [ Mapping 1 ]                                                     [v] (Expand/Collapse) |
    |   ROS Topic: /image        OTT: teleop.sensor.image                             |
    |   Direction: OUTBOUND      Priority: STANDARD                                   |
    |   Message Type: sensor_msgs/msg/Image                                         |
    |   Source Type: ROS2_CDR                                                        |
    |   [ Additional Properties (Conditional) ]                                       |
    |     Resolution: [Dropdown: 640x480 | 1280x720]                                  |
    |     Frame Rate: [Number Input]                                                  |
    |                                                                    [ Remove ] |
    [-----------------------------------------------------------------------------]
    | [ Mapping 2 ]                                                     [v] (Expand/Collapse) |
    |   # ... fields for mapping 2 ...                                            |
    |                                                                    [ Remove ] |
    [-----------------------------------------------------------------------------]

    [ Add New Mapping Button ]
    ```

### 3.3. Defaults (`defaults`)

*   **Purpose:** Configure default values used for topic mappings if not specified.
*   **UI Structure:** Collapsible fieldset containing labeled dropdowns.
*   **Fields:**
    *   `priority`: Dropdown (`HIGH`, `STANDARD`, `LOW`).
    *   `direction`: Dropdown (`INBOUND`, `OUTBOUND`).
    *   `source_type`: Dropdown (`ROS2_CDR`, potentially others).
*   **Visual Concept:**
    ```html
    <fieldset>
        <legend>Defaults <button type="button">+/-</button></legend>
        <div>
            <label>Default Priority:</label> <select>...</select>
            <label>Default Direction:</label> <select>...</select>
            <label>Default Source Type:</label> <select>...</select>
        </div>
    </fieldset>
    ```

### 3.4. Throttle Rates (`throttle_rates`)

*   **Purpose:** Configure message throttling limits per priority level.
*   **UI Structure:** Collapsible fieldset containing labeled number inputs.
*   **Fields:**
    *   `high_hz`: Number input (`min="0"`).
    *   `standard_hz`: Number input (`min="0"`).
    *   `low_hz`: Number input (`min="0"`).
*   **UI Elements:** Include helper text like "(0 = no limit)".
*   **Visual Concept:**
    ```html
    <fieldset>
        <legend>Throttle Rates (Max Hz) <button type="button">+/-</button></legend>
        <div>
            <label>High Priority:</label> <input type="number" min="0"> <span>(0 = no limit)</span>
            <label>Standard Priority:</label> <input type="number" min="0"> <span>(0 = no limit)</span>
            <label>Low Priority:</label> <input type="number" min="0"> <span>(0 = no limit)</span>
        </div>
    </fieldset>
    ```

### 3.5. Action Buttons & Status

*   **Buttons:** `[Save Configuration]`, `[Load Current Config]` (or similar).
*   **Status Area:** A paragraph element (`<p>`) to display feedback messages (e.g., "Saving...", "Saved successfully", "Error: ...").

## 4. Future Considerations & Backend Needs

*   **ROS Topic Discovery:** Requires a backend API endpoint (e.g., `/api/ros/topics`) to provide the list of topics and potentially their message types to populate the `ros_topic` selector.
*   **Message Type Discovery & Schemas:** Requires a backend API (e.g., `/api/ros/message_types`) to populate the `message_type` dropdown. This API should leverage **ROS 2 type introspection** (likely implemented in the ROS Gateway or via a CGO module in the controller) to dynamically discover all available message types, including custom ones, in the target ROS environment. This introspection capability may also be used to fetch schema details for driving conditional UI fields or validation.
*   **YAML Generation:** The frontend will need logic to gather data from all form fields and construct the correct YAML structure before sending it to the backend save endpoint.
*   **Validation:** Implement client-side validation (e.g., required fields, number ranges) in addition to any backend validation.

## 5. Implementation Tasks (High Level)

1.  Create basic HTML structure for the new config tab UI.
2.  Implement JavaScript logic for collapsible sections.
3.  Implement dynamic list management for Topic Mappings (add/remove).
4.  Implement conditional field display based on selected `message_type` (initially hardcoded for common types).
5.  Integrate with backend API to load existing configuration data into the form.
6.  Implement logic to gather form data and send it to the backend save API.
7.  (Future) Integrate ROS Topic Discovery API into the `ros_topic` field.
8.  (Future) Refine conditional field logic using backend schema information if available.
9.  Add comprehensive client-side validation.
10. Styling and layout refinement. 