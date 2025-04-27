// Functions for interacting with the backend Controller API

console.log("api.js loaded");

let ws = null;
let wsStatusCallback = null; // Callback function to update UI status

// Function to register a callback for WebSocket status changes
function registerWsStatusCallback(callback) {
    wsStatusCallback = callback;
}

// Function to update the UI status via the callback
function updateStatus(message, color) {
    if (wsStatusCallback) {
        wsStatusCallback(message, color);
    } else {
        console.log(`Status Update: ${message}`); // Fallback logging
    }
}

// --- WebSocket Connection ---
function connectWebSocket() {
    // Determine WebSocket URL based on page location
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.host}/ws/control`;
    console.log(`Connecting to WebSocket: ${wsUrl}`);
    updateStatus('Status: Connecting...', 'orange');

    // Close existing connection if any
    if (ws && ws.readyState !== WebSocket.CLOSED) {
        ws.close();
    }

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        console.log('WebSocket connection opened');
        updateStatus('Status: Connected', 'green');
    };

    ws.onclose = (event) => {
        console.log('WebSocket connection closed:', event.code, event.reason);
        updateStatus(`Status: Disconnected (${event.code})`, 'red');
        ws = null;
        // Attempt to reconnect after a delay
        setTimeout(connectWebSocket, 5000); // Reconnect after 5 seconds
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        updateStatus('Status: Error', 'red');
        // The onclose event will likely follow, triggering reconnection attempt
    };

    // We don't expect messages from server in this simple control case
    // ws.onmessage = (event) => {
    //     console.log('WebSocket message received:', event.data);
    // };
}

// --- Send Teleop Command (WebSocket) ---
function sendTeleopCommand(linear, angular) {
    // Format the message as expected by the backend (geometry_msgs/Twist structure)
    const message = JSON.stringify({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
    });

    if (ws && ws.readyState === WebSocket.OPEN) {
        // console.log(`Sending WS: ${message}`); // Debug log
        ws.send(message);
        return true; // Indicate success
    } else {
        // console.log(`WS not open. Skipping send: linear=${linear.toFixed(2)}, angular=${angular.toFixed(2)}`);
        return false; // Indicate failure
    }
}

// --- Get Teleop Configuration (REST API) ---
async function getTeleopConfig() {
    console.log("Fetching teleop config from /api/v1/config/teleop");
    try {
        const response = await fetch('/api/v1/config/teleop', {
            method: 'GET',
            headers: {
                'Accept': 'application/x-yaml, application/yaml, text/yaml'
            }
        });

        if (!response.ok) {
            const errorData = await response.json().catch(() => ({ error: 'Failed to parse error response' }));
            console.error('Error fetching config:', response.status, errorData.error);
            throw new Error(`HTTP error ${response.status}: ${errorData.error || 'Unknown error'}`);
        }

        if (response.status === 204 || response.headers.get('content-length') === '0') {
             console.log('Received empty response (204 or zero length)');
             return ''; // Return empty string for no content
        }

        const yamlData = await response.text();
        console.log("Config fetched successfully.");
        return yamlData;
    } catch (error) {
        console.error('Error in getTeleopConfig:', error);
        throw error; // Re-throw the error to be handled by the caller
    }
}


// --- Update Teleop Configuration (REST API) ---
async function updateTeleopConfig(yamlData) {
    console.log("Sending updated teleop config to /api/v1/config/teleop");
    try {
        const response = await fetch('/api/v1/config/teleop', {
            method: 'PUT',
            headers: {
                'Content-Type': 'application/x-yaml'
            },
            body: yamlData
        });

        const responseData = await response.json(); // Always expect JSON response for status/error

        if (!response.ok) {
            console.error('Error updating config:', response.status, responseData.error);
            throw new Error(`HTTP error ${response.status}: ${responseData.error || 'Unknown error'}`);
        }

        console.log("Config updated successfully:", responseData.message);
        return responseData.message; // Return success message
    } catch (error) {
        console.error('Error in updateTeleopConfig:', error);
        throw error; // Re-throw the error to be handled by the caller
    }
}

// Example functions:
// async function getTeleopConfig() { ... }
// async function updateTeleopConfig(yamlData) { ... }
// function connectWebSocket() { ... }
// function sendWebSocketMessage(message) { ... } 