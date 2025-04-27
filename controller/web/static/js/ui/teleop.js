// UI Logic for the Teleop Tab (Joystick, WebSocket)

console.log("teleop.js loaded");

function initTeleop() {
    console.log('Initializing Teleop Tab...');

    const statusDiv = document.getElementById('status');
    const joystickContainer = document.getElementById('joystick-container');
    const joystickBase = document.getElementById('joystick-base');
    const joystickHandle = document.getElementById('joystick-handle');

    if (!statusDiv || !joystickContainer || !joystickBase || !joystickHandle) {
        console.error("Teleop UI elements not found!");
        return;
    }

    let isDragging = false;
    let lastSentLinear = 0;
    let lastSentAngular = 0;

    const handleRadius = joystickHandle.offsetWidth / 2;

    // Register the status update function with the API module
    if (typeof registerWsStatusCallback === 'function') {
        registerWsStatusCallback((message, color) => {
            statusDiv.textContent = message;
            statusDiv.style.color = color || 'black';
        });
    } else {
        console.error("registerWsStatusCallback function not found in api.js");
    }

    // --- Event Listeners ---
    joystickHandle.addEventListener('mousedown', startDragging);
    document.addEventListener('mouseup', stopDragging);
    document.addEventListener('mousemove', handleDrag);

    joystickHandle.addEventListener('touchstart', startDragging, { passive: false });
    document.addEventListener('touchend', stopDragging);
    document.addEventListener('touchmove', handleDrag, { passive: false });

    function startDragging(e) {
        isDragging = true;
        joystickHandle.style.transition = 'none'; // Disable transition during drag
        if (e.type === 'touchstart') {
            e.preventDefault(); // Prevent scrolling on touch devices
        }
        // console.log("Drag Start");
    }

    function stopDragging(e) {
        if (!isDragging) return;
        isDragging = false;
        joystickHandle.style.transition = 'transform 0.1s ease-out'; // Smooth return
        joystickHandle.style.transform = 'translate(-50%, -50%)'; // Return to center
        // console.log("Drag End: Sending zero velocity");
        sendJoystickData(0, 0);
    }

    function handleDrag(e) {
        if (!isDragging) return;

        // Get fresh base position
        const baseRect = joystickBase.getBoundingClientRect();
        const baseRadius = baseRect.width / 2;
        const maxHandleDisplacement = baseRadius - handleRadius;

        let clientX, clientY;
        if (e.type.startsWith('touch')) {
            if (e.touches.length > 0) {
                 clientX = e.touches[0].clientX;
                 clientY = e.touches[0].clientY;
            } else {
                // If all touches are lifted, treat as stop (though touchend should handle this)
                stopDragging(e);
                return;
            }
             e.preventDefault(); // Prevent scrolling on touch devices
        } else {
            clientX = e.clientX;
            clientY = e.clientY;
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
        joystickHandle.style.transform = `translate(calc(-50% + ${handleX}px), calc(-50% + ${handleY}px))`;

        // Calculate linear/angular values (-1.0 to 1.0)
        // Assuming forward is negative Y, right is positive X for angular
        const linear = -dy / maxHandleDisplacement;
        const angular = dx / maxHandleDisplacement;

        sendJoystickData(linear, angular);
    }

    function sendJoystickData(linear, angular) {
        // Only send if values have changed significantly or are non-zero
        // Add tolerance check to avoid sending rapid zeros when stopping
        if (Math.abs(linear - lastSentLinear) > 0.01 || Math.abs(angular - lastSentAngular) > 0.01 || (linear === 0 && angular === 0 && (lastSentLinear !== 0 || lastSentAngular !== 0))) {
             // console.log(`Attempting send: L=${linear.toFixed(2)}, A=${angular.toFixed(2)}`);
             if (typeof sendTeleopCommand === 'function') {
                sendTeleopCommand(linear, angular);
             } else {
                 console.error("sendTeleopCommand function not found in api.js");
             }
            lastSentLinear = linear;
            lastSentAngular = angular;
        } else if (linear === 0 && angular === 0 && lastSentLinear === 0 && lastSentAngular === 0) {
             // Do nothing if already stopped and trying to send zero again
        } else if (Math.abs(linear - lastSentLinear) <= 0.01 && Math.abs(angular - lastSentAngular) <= 0.01 && (linear !==0 || angular !== 0)) {
            // If values haven't changed much and aren't zero, still send periodically? Maybe not needed for joystick.
            // console.log("Values similar, skipping send.");
        }
    }

    // --- Initial Connection ---
    if (typeof connectWebSocket === 'function') {
        connectWebSocket();
    } else {
        console.error("connectWebSocket function not found in api.js");
    }
} 