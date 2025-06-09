// Control system state management
// Manages WebSocket connection and joystick state

import { writable, derived } from 'svelte/store';
import { ControlWebSocketClient } from '$lib/api/websocket';

// Control connection state
export interface ControlState {
    isConnected: boolean;
    status: string;
    statusColor: string;
    lastCommand: {
        linear: number;
        angular: number;
        timestamp: Date;
    } | null;
    commandsSent: number;
    connectionAttempts: number;
}

// Create control state store
export const controlState = writable<ControlState>({
    isConnected: false,
    status: 'Disconnected',
    statusColor: 'red',
    lastCommand: null,
    commandsSent: 0,
    connectionAttempts: 0
});

// WebSocket client instance (singleton)
let wsClient: ControlWebSocketClient | null = null;

// Control actions
export const controlActions = {
    // Initialize WebSocket connection
    initialize: () => {
        if (!wsClient) {
            wsClient = new ControlWebSocketClient((message: string, color: string) => {
                controlState.update(state => ({
                    ...state,
                    status: message,
                    statusColor: color,
                    isConnected: color === 'green'
                }));
            });
        }
        wsClient.connect();
        
        controlState.update(state => ({
            ...state,
            connectionAttempts: state.connectionAttempts + 1
        }));
    },

    // Send teleop command
    sendCommand: (linear: number, angular: number) => {
        if (wsClient) {
            const success = wsClient.sendTeleopCommand(linear, angular);
            if (success) {
                controlState.update(state => ({
                    ...state,
                    lastCommand: {
                        linear,
                        angular,
                        timestamp: new Date()
                    },
                    commandsSent: state.commandsSent + 1
                }));
            }
            return success;
        }
        return false;
    },

    // Disconnect WebSocket
    disconnect: () => {
        if (wsClient) {
            wsClient.disconnect();
        }
    },

    // Reconnect WebSocket
    reconnect: () => {
        if (wsClient) {
            wsClient.reconnect();
            controlState.update(state => ({
                ...state,
                connectionAttempts: state.connectionAttempts + 1
            }));
        } else {
            controlActions.initialize();
        }
    },

    // Check if connected
    isConnected: () => {
        return wsClient ? wsClient.isConnected() : false;
    },

    // Get WebSocket client instance (for advanced usage)
    getClient: () => wsClient
};

// Derived store for connection status
export const isControlConnected = derived(
    controlState,
    $state => $state.isConnected
);

// Derived store for status display
export const controlStatus = derived(
    controlState,
    $state => ({
        message: $state.status,
        color: $state.statusColor
    })
);

// Derived store for command statistics
export const controlStats = derived(
    controlState,
    $state => ({
        commandsSent: $state.commandsSent,
        lastCommand: $state.lastCommand,
        connectionAttempts: $state.connectionAttempts
    })
);