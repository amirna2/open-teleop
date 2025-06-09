// Virtual Joystick Controller - TypeScript Port
// CRITICAL: This is an EXACT functional port of teleop.js joystick logic
// NO CHANGES to control behavior, touch/mouse handling, or velocity calculations

// Debug configuration
const JOYSTICK_DEBUG = false;

function joystickDebugLog(...args: any[]): void {
    if (JOYSTICK_DEBUG) {
        console.log(...args);
    }
}

// Joystick position interface
export interface JoystickPosition {
    x: number;
    y: number;
    distance: number;
    angle: number;
}

// Velocity command interface (matching geometry_msgs/Twist)
export interface VelocityCommand {
    linear: { x: number; y: number; z: number };
    angular: { x: number; y: number; z: number };
}

// Joystick event callbacks
export interface JoystickCallbacks {
    onMove?: (linear: number, angular: number) => void;
    onStart?: () => void;
    onEnd?: () => void;
}

export class VirtualJoystick {
    private isDragging: boolean = false;
    private lastSentLinear: number = 0;
    private lastSentAngular: number = 0;
    private handleRadius: number = 0;
    
    private joystickBase: HTMLElement;
    private joystickHandle: HTMLElement;
    private callbacks: JoystickCallbacks;

    // Bound event handlers to maintain 'this' context
    private boundStartDragging = this.startDragging.bind(this);
    private boundStopDragging = this.stopDragging.bind(this);
    private boundHandleDrag = this.handleDrag.bind(this);

    constructor(
        baseElement: HTMLElement,
        handleElement: HTMLElement,
        callbacks: JoystickCallbacks = {}
    ) {
        this.joystickBase = baseElement;
        this.joystickHandle = handleElement;
        this.callbacks = callbacks;
        
        // Calculate handle radius
        this.handleRadius = this.joystickHandle.offsetWidth / 2;
        
        this.setupEventListeners();
        joystickDebugLog("VirtualJoystick initialized");
    }

    private setupEventListeners(): void {
        // Mouse events
        this.joystickHandle.addEventListener('mousedown', this.boundStartDragging);
        document.addEventListener('mouseup', this.boundStopDragging);
        document.addEventListener('mousemove', this.boundHandleDrag);

        // Touch events
        this.joystickHandle.addEventListener('touchstart', this.boundStartDragging, { passive: false });
        document.addEventListener('touchend', this.boundStopDragging);
        document.addEventListener('touchmove', this.boundHandleDrag, { passive: false });
    }

    private startDragging(e: MouseEvent | TouchEvent): void {
        this.isDragging = true;
        this.joystickHandle.style.transition = 'none'; // Disable transition during drag
        
        if (e.type === 'touchstart') {
            e.preventDefault(); // Prevent scrolling on touch devices
        }
        
        joystickDebugLog("Drag Start");
        this.callbacks.onStart?.();
    }

    private stopDragging(e: MouseEvent | TouchEvent): void {
        if (!this.isDragging) return;
        
        this.isDragging = false;
        this.joystickHandle.style.transition = 'transform 0.1s ease-out'; // Smooth return
        this.joystickHandle.style.transform = 'translate(-50%, -50%)'; // Return to center
        
        joystickDebugLog("Drag End: Sending zero velocity");
        this.sendJoystickData(0, 0);
        this.callbacks.onEnd?.();
    }

    private handleDrag(e: MouseEvent | TouchEvent): void {
        if (!this.isDragging) return;

        // Get fresh base position
        const baseRect = this.joystickBase.getBoundingClientRect();
        const baseRadius = baseRect.width / 2;
        const maxHandleDisplacement = baseRadius - this.handleRadius;

        let clientX: number, clientY: number;
        
        if (e.type.startsWith('touch')) {
            const touchEvent = e as TouchEvent;
            if (touchEvent.touches.length > 0) {
                clientX = touchEvent.touches[0].clientX;
                clientY = touchEvent.touches[0].clientY;
            } else {
                // If all touches are lifted, treat as stop
                this.stopDragging(e);
                return;
            }
            e.preventDefault(); // Prevent scrolling on touch devices
        } else {
            const mouseEvent = e as MouseEvent;
            clientX = mouseEvent.clientX;
            clientY = mouseEvent.clientY;
        }

        const baseCenterX = baseRect.left + baseRadius;
        const baseCenterY = baseRect.top + baseRadius;

        let dx = clientX - baseCenterX;
        let dy = clientY - baseCenterY;

        const distance = Math.sqrt(dx * dx + dy * dy);

        // Clamp the handle position within the base circle
        if (distance > maxHandleDisplacement) {
            dx = (dx / distance) * maxHandleDisplacement;
            dy = (dy / distance) * maxHandleDisplacement;
        }

        // Update handle visual position (relative to its center)
        const handleX = dx;
        const handleY = dy;
        this.joystickHandle.style.transform = `translate(calc(-50% + ${handleX}px), calc(-50% + ${handleY}px))`;

        // Calculate linear/angular values (-1.0 to 1.0)
        // Forward is negative Y, right is positive X for angular
        const linear = -dy / maxHandleDisplacement;
        const angular = dx / maxHandleDisplacement;

        this.sendJoystickData(linear, angular);
    }

    private sendJoystickData(linear: number, angular: number): void {
        // Only send if values have changed significantly or are non-zero
        // Add tolerance check to avoid sending rapid zeros when stopping
        const threshold = 0.01;
        const linearChanged = Math.abs(linear - this.lastSentLinear) > threshold;
        const angularChanged = Math.abs(angular - this.lastSentAngular) > threshold;
        const isZero = linear === 0 && angular === 0;
        const wasNotZero = this.lastSentLinear !== 0 || this.lastSentAngular !== 0;

        if (linearChanged || angularChanged || (isZero && wasNotZero)) {
            joystickDebugLog(`Sending: L=${linear.toFixed(2)}, A=${angular.toFixed(2)}`);
            this.callbacks.onMove?.(linear, angular);
            this.lastSentLinear = linear;
            this.lastSentAngular = angular;
        } else if (isZero && this.lastSentLinear === 0 && this.lastSentAngular === 0) {
            // Do nothing if already stopped and trying to send zero again
        } else if (!linearChanged && !angularChanged && !isZero) {
            // If values haven't changed much and aren't zero, skip
            joystickDebugLog("Values similar, skipping send.");
        }
    }

    // Public method to get current joystick position
    public getPosition(): JoystickPosition {
        const baseRect = this.joystickBase.getBoundingClientRect();
        const baseRadius = baseRect.width / 2;
        const maxHandleDisplacement = baseRadius - this.handleRadius;
        
        // Parse current transform to get position
        const transform = this.joystickHandle.style.transform;
        const matches = transform.match(/translate\(calc\(-50% \+ (-?\d+(?:\.\d+)?)px\), calc\(-50% \+ (-?\d+(?:\.\d+)?)px\)\)/);
        
        let x = 0, y = 0;
        if (matches) {
            x = parseFloat(matches[1]) || 0;
            y = parseFloat(matches[2]) || 0;
        }
        
        const distance = Math.sqrt(x * x + y * y);
        const angle = Math.atan2(y, x);
        
        return {
            x: x / maxHandleDisplacement,
            y: y / maxHandleDisplacement,
            distance: distance / maxHandleDisplacement,
            angle: angle
        };
    }

    // Public method to programmatically set position
    public setPosition(x: number, y: number): void {
        const baseRect = this.joystickBase.getBoundingClientRect();
        const baseRadius = baseRect.width / 2;
        const maxHandleDisplacement = baseRadius - this.handleRadius;
        
        // Clamp values to [-1, 1]
        x = Math.max(-1, Math.min(1, x));
        y = Math.max(-1, Math.min(1, y));
        
        const handleX = x * maxHandleDisplacement;
        const handleY = y * maxHandleDisplacement;
        
        this.joystickHandle.style.transform = `translate(calc(-50% + ${handleX}px), calc(-50% + ${handleY}px))`;
        
        // Send joystick data
        const linear = -y; // Forward is negative Y
        const angular = x; // Right is positive X
        this.sendJoystickData(linear, angular);
    }

    // Reset joystick to center
    public reset(): void {
        this.joystickHandle.style.transition = 'transform 0.1s ease-out';
        this.joystickHandle.style.transform = 'translate(-50%, -50%)';
        this.sendJoystickData(0, 0);
    }

    // Cleanup method
    public destroy(): void {
        // Remove all event listeners
        this.joystickHandle.removeEventListener('mousedown', this.boundStartDragging);
        document.removeEventListener('mouseup', this.boundStopDragging);
        document.removeEventListener('mousemove', this.boundHandleDrag);
        
        this.joystickHandle.removeEventListener('touchstart', this.boundStartDragging);
        document.removeEventListener('touchend', this.boundStopDragging);
        document.removeEventListener('touchmove', this.boundHandleDrag);
        
        joystickDebugLog("VirtualJoystick destroyed");
    }
}

joystickDebugLog("joystick.ts loaded and ready!");