# Mini Web UI Functional Requirements

## 1. Overview

This document outlines the functional requirements for the "Mini Web UI," a local web interface for the Open-Teleop platform. The UI's primary purpose is to provide a streamlined and intuitive way to verify the operational status of core teleoperation features, including video streaming, robot control, and configuration management. The design prioritizes simplicity, clarity, and reusability of existing components to avoid feature creep.

## 2. Core UI Structure & Layout

### 2.1. Main Layout
- **FR-2.1.1:** The UI shall consist of a main layout containing a persistent left sidebar (LEFT BAR) for navigation, a main content area for displaying views (MAIN AREA), and a persistent footer (FOOTER) for system health status.

### 2.2. Left Sidebar
- **FR-2.2.1:** The sidebar shall contain two primary navigation options:
    - **Teleop:** The default view for robot operation.
    - **Configuration:** A view for managing the teleop configuration.
- **FR-2.2.2:** The currently active view shall be visually highlighted in the sidebar.

### 2.3. Footer
- **FR-2.3.1:** The footer shall display essential system health metrics.
- **FR-2.3.2:** Health metrics must include at a minimum: CPU usage, memory usage, and network throughput.
- **FR-2.3.3:** Each metric shall be represented by a clear icon and a compact visual indicator (e.g., a simple gauge or percentage).

## 3. Teleop View

### 3.1. Default Layout
- **FR-3.1.1:** The Teleop view shall load with a default layout consisting of two primary components:
    - A single, large video card displaying the primary video stream. The card shall occupy 100% of the main area.
    - A single robot control card overlaying the main area and positioned by default to the bottom right corner of the main area

### 3.2. Toolbar
- **FR-3.2.1:** A toolbar shall be present at the top of the Teleop view.
- **FR-3.2.2:** The toolbar shall provide functionality to add additional components (cards) to the view from a predefined list.
- **FR-3.2.3:** Addable components must include, but are not limited to:
    - Additional video stream cards (e.g., "Rear Camera", "Arm Camera").
    - Additional control cards (e.g., "Secondary Joystick").
    - Telemetry overlay cards (e.g., "Occupancy Grid", "Point Cloud").
- **FR-3.2.4:** Each addable component must be directly associated with a specific, non-customizable stream defined in the teleop configuration.

### 3.3. Card-Based Design
- **FR-3.3.1:** All functional components in the Teleop view shall be displayed within self-contained "cards."
- **FR-3.3.2:** Cards shall be arranged in a grid-like fashion.
- **FR-3.3.3:** Each card must have a header displaying its title (e.g., "Primary Video Stream").

## 4. Component-Specific Requirements

### 4.1. Video Cards
- **FR-4.1.1:** Each video card shall display a live video stream.
- **FR-4.1.2:** A user must be able to click on any video card to swap it into the main, larger video card slot.
- **FR-4.1.3:** Each video card header shall contain a status icon indicating the stream's state (e.g., Connected, Disconnected, Error).
- **FR-4.1.4:** Video cards must support telemetry overlays (e.g., occupancy grids).
- **FR-4.1.5:** An overlay toggle button must be available on video cards to show or hide telemetry data.

### 4.2. Control Cards
- **FR-4.2.1:** Control cards shall display a single, non-customizable virtual joystick interface for robot control.
- **FR-4.2.2:** The joystick's visual style and behavior shall be consistent across all control cards.

### 4.3. Robot Health
- **FR-4.3.1:** Robot-specific health metrics (e.g., battery level, temperature) shall be displayed as an overlay card.
- **FR-4.3.2:** This robot health card shall be addable via the Teleop view toolbar.

### 4.4. Status Indicators
- **FR-4.4.1:** All status indicators throughout the UI (e.g., for connections, streams) must use a consistent set of icons (e.g., green for connected, yellow for warning, red for error).
- **FR-4.4.2:** Hovering over a status icon must display a tooltip with a brief, human-readable description of the status.

## 5. Configuration View

### 5.1. Editor
- **FR-5.1.1:** The Configuration view shall reuse the existing implementation of a simple YAML text editor.
- **FR-5.1.2:** The view must provide controls to load the current configuration from the backend and save modified configurations.

## 6. Non-Functional Requirements

### 6.1. Performance
- **NFR-6.1.1:** The UI must remain responsive and performant, even with multiple video streams active.
### 6.2. Simplicity
- **NFR-6.2.1:** The UI shall avoid complex customization options. Component selection and layout adjustments are limited to the functionality provided by the toolbar.
### 6.3. Reusability
- **NFR-6.3.1:** The implementation shall reuse existing backend APIs and frontend components where possible to minimize development effort. 