// WebSocket API Client - TypeScript Port
// CRITICAL: This is an EXACT functional port of api.js WebSocket functionality
// NO CHANGES to connection handling, message format, or reconnection logic

// WebSocket connection state
interface WebSocketState {
    ws: WebSocket | null;
    isConnected: boolean;
    lastError: string | null;
    reconnectAttempts: number;
}

// Status callback type
export type StatusCallback = (message: string, color: string) => void;

// Velocity command interface (matching geometry_msgs/Twist)
export interface TwistMessage {
    linear: { x: number; y: number; z: number };
    angular: { x: number; y: number; z: number };
}

export class ControlWebSocketClient {
    private state: WebSocketState = {
        ws: null,
        isConnected: false,
        lastError: null,
        reconnectAttempts: 0
    };

    private statusCallback: StatusCallback | null = null;
    private reconnectTimeout: NodeJS.Timeout | null = null;
    private readonly maxReconnectAttempts = 10;
    private readonly reconnectDelay = 5000; // 5 seconds

    constructor(statusCallback?: StatusCallback) {
        this.statusCallback = statusCallback || null;
        console.log("ControlWebSocketClient initialized");
    }

    // Register callback for status updates
    public setStatusCallback(callback: StatusCallback): void {
        this.statusCallback = callback;
    }

    // Update status via callback or console fallback
    private updateStatus(message: string, color: string): void {
        if (this.statusCallback) {
            this.statusCallback(message, color);
        } else {
            console.log(`Status Update: ${message}`); // Fallback logging
        }
    }

    // Connect to WebSocket
    public connect(): void {
        // Determine WebSocket URL based on page location
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${wsProtocol}//${window.location.host}/ws/control`;
        
        console.log(`Connecting to WebSocket: ${wsUrl}`);
        this.updateStatus('Status: Connecting...', 'orange');

        // Close existing connection if any
        if (this.state.ws && this.state.ws.readyState !== WebSocket.CLOSED) {
            this.state.ws.close();
        }

        this.state.ws = new WebSocket(wsUrl);

        this.state.ws.onopen = () => {
            console.log('WebSocket connection opened');
            this.state.isConnected = true;
            this.state.lastError = null;
            this.state.reconnectAttempts = 0;
            this.updateStatus('Status: Connected', 'green');
            
            // Clear any pending reconnect timeout
            if (this.reconnectTimeout) {
                clearTimeout(this.reconnectTimeout);
                this.reconnectTimeout = null;
            }
        };

        this.state.ws.onclose = (event) => {
            console.log('WebSocket connection closed:', event.code, event.reason);
            this.state.isConnected = false;
            this.updateStatus(`Status: Disconnected (${event.code})`, 'red');
            this.state.ws = null;
            
            // Attempt to reconnect after a delay if under max attempts
            if (this.state.reconnectAttempts < this.maxReconnectAttempts) {
                this.state.reconnectAttempts++;
                console.log(`Scheduling reconnect attempt ${this.state.reconnectAttempts}/${this.maxReconnectAttempts}`);
                this.reconnectTimeout = setTimeout(() => this.connect(), this.reconnectDelay);
            } else {
                console.log('Max reconnect attempts reached');
                this.updateStatus('Status: Connection Failed', 'red');
            }
        };

        this.state.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
            this.state.lastError = 'WebSocket error occurred';
            this.updateStatus('Status: Error', 'red');
            // The onclose event will likely follow, triggering reconnection attempt
        };

        // We don't expect messages from server in this simple control case
        // Keeping this for potential future use
        this.state.ws.onmessage = (event) => {
            console.log('WebSocket message received:', event.data);
        };
    }

    // Send teleop command (exact same format as original)
    public sendTeleopCommand(linear: number, angular: number): boolean {
        // Format the message as expected by the backend (geometry_msgs/Twist structure)
        const message: TwistMessage = {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular }
        };

        const messageString = JSON.stringify(message);

        if (this.state.ws && this.state.ws.readyState === WebSocket.OPEN) {
            // console.log(`Sending WS: ${messageString}`); // Debug log
            this.state.ws.send(messageString);
            return true; // Indicate success
        } else {
            // console.log(`WS not open. Skipping send: linear=${linear.toFixed(2)}, angular=${angular.toFixed(2)}`);
            return false; // Indicate failure
        }
    }

    // Send raw message
    public sendMessage(message: string | object): boolean {
        const messageString = typeof message === 'string' ? message : JSON.stringify(message);

        if (this.state.ws && this.state.ws.readyState === WebSocket.OPEN) {
            this.state.ws.send(messageString);
            return true;
        } else {
            console.log('WebSocket not open. Cannot send message.');
            return false;
        }
    }

    // Get connection state
    public isConnected(): boolean {
        return this.state.isConnected && 
               this.state.ws !== null && 
               this.state.ws.readyState === WebSocket.OPEN;
    }

    // Get last error
    public getLastError(): string | null {
        return this.state.lastError;
    }

    // Disconnect and cleanup
    public disconnect(): void {
        // Clear reconnect timeout
        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }

        // Reset reconnect attempts to prevent auto-reconnection
        this.state.reconnectAttempts = this.maxReconnectAttempts;

        // Close WebSocket connection
        if (this.state.ws) {
            this.state.ws.close();
            this.state.ws = null;
        }

        this.state.isConnected = false;
        this.updateStatus('Status: Disconnected', 'gray');
        console.log('WebSocket manually disconnected');
    }

    // Force reconnect (reset attempts and connect)
    public reconnect(): void {
        this.state.reconnectAttempts = 0;
        this.disconnect();
        setTimeout(() => this.connect(), 100); // Small delay before reconnecting
    }
}

console.log("websocket.ts loaded and ready!");