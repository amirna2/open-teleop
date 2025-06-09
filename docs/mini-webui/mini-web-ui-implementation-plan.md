# Mini Web UI Implementation Plan

## Overview

This document outlines the implementation plan for replacing the current web UI with a modern Svelte-based mini-web-ui that meets the functional requirements defined in `mini-web-ui-requirements.md`.

## Analysis of Current Implementation

### Current Architecture
- **Static HTML/JS/CSS**: Traditional multi-page application with dashboard layout
- **Backend Integration**: WebSocket for real-time control, REST API for configuration
- **Video Streaming**: **WebCodecs H.264 Player** with custom NAL unit parsing and canvas rendering
- **Core APIs**: Well-defined API layer in `api.js` with WebSocket and REST endpoints

### Critical Video Implementation (MUST BE PRESERVED)
**WebCodecs H.264 Player** (`video-simpler.js`):
- **WebCodecs API**: Direct H.264 decoding using browser's WebCodecs interface
- **Binary WebSocket**: Raw H.264 NAL units received via `/ws/video` endpoint
- **Real-time Processing**: SPS/PPS extraction and EncodedVideoChunk creation
- **Canvas Rendering**: Low-latency frame rendering to canvas element
- **Performance Monitoring**: FPS tracking, decode error rates, frame statistics
- **Hardware Acceleration**: Configurable hardware/software decoding preference

### Reusable Components
1. **WebCodecs Video System** (`video-simpler.js`): **CRITICAL - MUST BE PRESERVED EXACTLY**
   - Custom H.264 decoder configuration
   - NAL unit parsing and frame processing
   - Real-time statistics and error handling
   - Canvas-based rendering pipeline

2. **API Layer** (`api.js`): WebSocket connection, teleop commands, config management
3. **Joystick Implementation** (`teleop.js`): Virtual joystick with touch/mouse support
4. **Configuration Editor** (`config.js`): YAML editor with validation
5. **FlatBuffers Integration**: Message serialization system

### Current Limitations
- Monolithic layout with fixed dashboard structure
- Limited customization and card-based design
- No dynamic component addition/removal
- Basic status indicators without comprehensive health metrics

## Implementation Strategy

### Phase 1: Foundation & Setup

1. **Svelte Project Setup**
   - Initialize Svelte project within `controller/web/svelte/` 
   - Configure Vite build system to output to `static/dist/`
   - Set up TypeScript support for better development experience
   - Configure build output to serve from Go controller

2. **Build Integration**
   - Modify Go controller to serve Svelte build artifacts from `/static/dist/`
   - Update `scripts/build.sh` to include Svelte compilation step
   - Ensure development workflow with hot reloading compatibility

3. **Core Layout Implementation**
   - Create main layout with sidebar, main area, and footer
   - Implement navigation between Teleop and Configuration views
   - Add persistent footer with system health metrics placeholder

### Phase 2: WebCodecs Video Integration (CRITICAL)

1. **Preserve WebCodecs Implementation**
   - **Port `video-simpler.js` to TypeScript module with ZERO functional changes**
   - **Maintain exact WebCodecs decoder configuration and canvas rendering**
   - **Preserve all NAL unit parsing, SPS/PPS extraction logic**
   - **Keep performance monitoring and statistics tracking**

2. **Svelte Video Components**
   - Create VideoCard component that wraps the WebCodecs canvas
   - Implement video stream status indicators
   - Add telemetry overlay support and toggle controls
   - Ensure click-to-swap functionality for main video display

3. **Video State Management**
   - Create Svelte store for video connection state
   - Integrate existing video statistics with reactive UI updates
   - Maintain WebSocket connection management for video stream

### Phase 3: Control & Teleop Components

1. **Card-Based Architecture**
   - Implement base Card component with header and content slots
   - Create overlay positioning system for movable cards
   - **NO RESIZE functionality** - cards maintain fixed dimensions
   - Focus on simple drag-to-move overlay cards only

2. **Control Components**
   - Port joystick implementation to Svelte ControlCard component
   - **Maintain existing touch/mouse interaction patterns exactly**
   - Ensure consistent visual style across control cards
   - Preserve WebSocket command sending functionality

### Phase 4: Toolbar & Dynamic Components

1. **Toolbar Implementation**
   - Create toolbar component with "Add Component" functionality
   - Implement dropdown/modal for component selection
   - Connect to configuration to determine available streams

2. **Component Management**
   - Implement dynamic component addition/removal as overlay cards
   - **Fixed-size overlay cards** positioned over main video
   - Simple drag-to-move functionality for repositioning overlays
   - **NO RESIZE** - components maintain predefined dimensions

3. **Status & Health Metrics**
   - Implement robot health overlay card
   - Create consistent status indicator system
   - Add tooltip support for status explanations

### Phase 5: Configuration View

1. **Configuration Editor**
   - Port existing YAML editor to Svelte component
   - **Maintain exact load/save functionality using existing API**
   - Improve user experience with better validation feedback

### Phase 6: System Health & Polish

1. **Footer Health Metrics**
   - Implement CPU, memory, and network monitoring
   - Add visual indicators (gauges, percentages)
   - Connect to backend health APIs (to be implemented if needed)

2. **Polish & Testing**
   - Responsive design improvements
   - **Performance testing to ensure video streaming latency is unchanged**
   - Cross-browser compatibility testing
   - Error handling and edge case management

## Technical Architecture

### Directory Structure
```
controller/web/
├── svelte/                       # New Svelte application
│   ├── src/
│   │   ├── lib/
│   │   │   ├── components/
│   │   │   │   ├── layout/
│   │   │   │   │   ├── MainLayout.svelte
│   │   │   │   │   ├── Sidebar.svelte
│   │   │   │   │   └── Footer.svelte
│   │   │   │   ├── cards/
│   │   │   │   │   ├── BaseCard.svelte        # Fixed-size card with move capability
│   │   │   │   │   ├── VideoCard.svelte       # Full-screen main video (NO RESIZE)
│   │   │   │   │   ├── ControlCard.svelte     # Fixed-size overlay (movable)
│   │   │   │   │   └── HealthCard.svelte      # Fixed-size overlay (movable)
│   │   │   │   ├── ui/
│   │   │   │   │   ├── Toolbar.svelte
│   │   │   │   │   ├── StatusIndicator.svelte
│   │   │   │   │   └── Joystick.svelte
│   │   │   │   └── config/
│   │   │   │       └── ConfigEditor.svelte
│   │   │   ├── stores/           # Svelte stores for state
│   │   │   │   ├── websocket.ts
│   │   │   │   ├── config.ts
│   │   │   │   ├── video.ts
│   │   │   │   └── health.ts
│   │   │   ├── video/            # CRITICAL: WebCodecs preservation
│   │   │   │   ├── webcodecs-player.ts  # Exact port of video-simpler.js
│   │   │   │   ├── video-stats.ts
│   │   │   │   └── video-utils.ts
│   │   │   ├── api/              # Ported API layer
│   │   │   │   ├── websocket.ts
│   │   │   │   ├── config.ts
│   │   │   │   └── teleop.ts
│   │   │   └── utils/
│   │   │       ├── joystick.ts
│   │   │       └── status.ts
│   │   ├── routes/
│   │   │   ├── +layout.svelte
│   │   │   ├── +page.svelte          # Default to Teleop view
│   │   │   ├── teleop/
│   │   │   │   └── +page.svelte
│   │   │   └── config/
│   │   │       └── +page.svelte
│   │   └── app.html
│   ├── static/                   # Static assets
│   ├── package.json
│   ├── vite.config.js
│   └── tsconfig.json
├── static/                       # Current implementation (preserve during dev)
│   └── dist/                     # Svelte build output
└── legacy/                       # Move current static files here for reference
```

### Critical Video Integration Points

1. **WebCodecs Preservation**
   ```typescript
   // webcodecs-player.ts - EXACT functional port of video-simpler.js
   export class WebCodecsVideoPlayer {
     private decoder: VideoDecoder | null = null;
     private canvas: HTMLCanvasElement | null = null;
     private ctx: CanvasRenderingContext2D | null = null;
     
     // PRESERVE: All existing decoder configuration logic
     // PRESERVE: NAL unit parsing and SPS/PPS extraction
     // PRESERVE: Performance statistics and error handling
     // PRESERVE: Canvas rendering pipeline
   }
   ```

2. **Svelte Video Component Integration**
   ```svelte
   <!-- VideoCard.svelte -->
   <script lang="ts">
     import { WebCodecsVideoPlayer } from '$lib/video/webcodecs-player';
     
     let canvasElement: HTMLCanvasElement;
     let videoPlayer: WebCodecsVideoPlayer;
     
     onMount(() => {
       videoPlayer = new WebCodecsVideoPlayer(canvasElement);
       videoPlayer.connect(); // Same WebSocket connection logic
     });
   </script>
   
   <div class="video-card">
     <canvas bind:this={canvasElement} />
     <!-- Status indicators, overlay controls -->
   </div>
   ```

### Build Integration
- Vite configured to output to `static/dist/`
- Go controller serve from `/static/dist/` for Svelte app
- Development mode proxy for hot reloading during development
- Production build integrated into `scripts/build.sh`
- Preserve existing `/static/` for legacy files during transition

### State Management
- Svelte stores for global state (WebSocket, configuration, health)
- Video store manages WebCodecs player state and statistics
- Component-level state for UI interactions
- Reactive updates for real-time data without affecting video performance

## Migration Strategy

### Parallel Development Approach
1. **Develop Svelte UI in `/controller/web/svelte/`**
2. **Build output to `/controller/web/static/dist/`**
3. **Preserve current implementation in `/controller/web/static/`**
4. **Go controller routing to switch between implementations**

### WebCodecs Preservation Strategy
1. **Zero functional changes to video streaming logic**
2. **Exact port of decoder configuration and canvas rendering**
3. **Maintain all performance optimizations and error handling**
4. **Preserve WebSocket binary message processing**
5. **Keep statistics tracking and monitoring capabilities**

### Risk Mitigation
- Preserve all existing API contracts
- Maintain WebCodecs performance characteristics
- Incremental deployment with A/B testing capability
- Comprehensive video streaming performance validation
- Fallback to current implementation if issues arise

## Success Criteria

### Functional Requirements Met
- ✅ Card-based teleop interface with dynamic component addition
- ✅ Video stream management with click-to-swap functionality  
- ✅ Persistent system health footer
- ✅ Configuration editor with existing functionality
- ✅ Status indicators throughout the interface

### Performance Requirements (CRITICAL)
- ✅ **Video streaming latency identical to current implementation**
- ✅ **WebCodecs decoder performance unchanged**
- ✅ **Frame decode rates and error rates maintained**
- ✅ Smooth UI interactions without video interference
- ✅ Multiple video stream support without degradation

### Technical Requirements
- ✅ **100% preservation of WebCodecs video implementation**
- ✅ **Exact maintenance of NAL unit parsing logic**
- ✅ **Canvas rendering pipeline unchanged**
- ✅ Clean, modular Svelte component architecture
- ✅ Type-safe API interactions

## Implementation Phases Breakdown

### Phase 1: Infrastructure
- Svelte project setup with TypeScript
- Build system integration with existing Go controller
- Basic layout structure (sidebar, main area, footer)

### Phase 2: Video System (CRITICAL PHASE)
- **Exact port of WebCodecs implementation to TypeScript**
- **Video component integration with preserved functionality**
- **Performance validation and testing**

### Phase 3: Control Systems
- Joystick component migration
- WebSocket control integration
- Card-based architecture implementation

### Phase 4: Dynamic UI
- Toolbar and component management
- Configuration view migration
- Status indicators and health metrics

### Phase 5: Testing & Polish
- Cross-browser compatibility
- Performance optimization
- Error handling and edge cases
- Documentation updates

## Next Steps

1. **Phase 1 Implementation**: Begin with Svelte setup and build integration
2. **WebCodecs Analysis**: Detailed code review of `video-simpler.js` for exact porting requirements
3. **Performance Baseline**: Establish current video performance metrics for validation
4. **API Contracts**: Document all existing API interfaces for preservation

This plan ensures the advanced WebCodecs video streaming capabilities are fully preserved while modernizing the UI architecture with Svelte.