// Basic structure for joystick control logic

document.addEventListener('DOMContentLoaded', (event) => {
    console.log('DOM fully loaded and parsed');

    const statusDiv = document.getElementById('status');
    const joystickContainer = document.getElementById('joystick-container');
    const joystickBase = document.getElementById('joystick-base');
    const joystickHandle = document.getElementById('joystick-handle');

    let isDragging = false;
    let ws; // WebSocket placeholder
    let sendInterval; // Interval timer placeholder
    let lastSentLinear = 0;
    let lastSentAngular = 0;

    const baseRect = joystickBase.getBoundingClientRect();
    const baseRadius = baseRect.width / 2;
    const handleRadius = joystickHandle.offsetWidth / 2;
    const maxHandleDisplacement = baseRadius - handleRadius;

    // --- WebSocket Connection ---
    function connectWebSocket() {
        // Determine WebSocket URL based on page location
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${wsProtocol}//${window.location.host}/ws/control`;
        console.log(`Connecting to WebSocket: ${wsUrl}`);
        statusDiv.textContent = 'Status: Connecting...';

        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            console.log('WebSocket connection opened');
            statusDiv.textContent = 'Status: Connected';
            statusDiv.style.color = 'green';
        };

        ws.onclose = (event) => {
            console.log('WebSocket connection closed:', event.code, event.reason);
            statusDiv.textContent = `Status: Disconnected (${event.code})`;
            statusDiv.style.color = 'red';
            ws = null;
            // Attempt to reconnect after a delay
            setTimeout(connectWebSocket, 5000); // Reconnect after 5 seconds
        };

        ws.onerror = (error) => {
            console.error('WebSocket error:', error);
            statusDiv.textContent = 'Status: Error';
            statusDiv.style.color = 'red';
            // The onclose event will likely follow, triggering reconnection attempt
        };

        // We don't expect messages from server in this simple case
        // ws.onmessage = (event) => {
        //     console.log('WebSocket message received:', event.data);
        // };
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
        console.log("Drag Start");
    }

    function stopDragging(e) {
        if (!isDragging) return;
        isDragging = false;
        joystickHandle.style.transition = 'transform 0.1s ease-out'; // Smooth return
        joystickHandle.style.transform = 'translate(-50%, -50%)'; // Return to center
        console.log("Drag End: Sending zero velocity");
        sendJoystickData(0, 0);
    }

    function handleDrag(e) {
        if (!isDragging) return;

        let clientX, clientY;
        if (e.type.startsWith('touch')) {
            if (e.touches.length > 0) {
                 clientX = e.touches[0].clientX;
                 clientY = e.touches[0].clientY;
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
         if (Math.abs(linear - lastSentLinear) > 0.01 || Math.abs(angular - lastSentAngular) > 0.01 || linear !== 0 || angular !== 0 || (lastSentLinear === 0 && lastSentAngular === 0 && (linear !== 0 || angular !== 0)) || (linear === 0 && angular === 0 && (lastSentLinear !== 0 || lastSentAngular !== 0)))
        {
            // Format the message as expected by the backend (geometry_msgs/Twist structure)
            const message = JSON.stringify({
                linear: { x: linear, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angular }
            });

            if (ws && ws.readyState === WebSocket.OPEN) {
                console.log(`Sending WS: ${message}`);
                ws.send(message);
            } else {
                 console.log(`WS not open. Skipping send: linear=${linear.toFixed(2)}, angular=${angular.toFixed(2)}`);
            }
            lastSentLinear = linear;
            lastSentAngular = angular;
        }
    }

    // --- Initial Connection ---
    connectWebSocket();

}); 