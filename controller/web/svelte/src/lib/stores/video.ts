// Video state management store
// Maintains connection state and statistics for video streams

import { writable, derived } from 'svelte/store';

// Video stream connection state
export interface VideoStream {
    id: string;
    title: string;
    connectionStatus: 'connected' | 'disconnected' | 'error';
    lastConnected?: Date;
    statsEnabled: boolean;
}

// Global video statistics (aggregated from all streams)
export interface GlobalVideoStats {
    totalStreams: number;
    activeStreams: number;
    totalFramesReceived: number;
    totalDecodeErrors: number;
    averageFPS: number;
}

// Create stores
export const videoStreams = writable<Map<string, VideoStream>>(new Map());
export const primaryVideoStream = writable<string>('primary');
export const globalVideoStats = writable<GlobalVideoStats>({
    totalStreams: 0,
    activeStreams: 0,
    totalFramesReceived: 0,
    totalDecodeErrors: 0,
    averageFPS: 0
});

// Derived store for easy access to primary stream
export const primaryStream = derived(
    [videoStreams, primaryVideoStream],
    ([$streams, $primaryId]) => {
        return $streams.get($primaryId) || null;
    }
);

// Video stream management functions
export const videoStreamActions = {
    // Register a new video stream
    registerStream: (id: string, title: string) => {
        videoStreams.update(streams => {
            streams.set(id, {
                id,
                title,
                connectionStatus: 'disconnected',
                statsEnabled: false
            });
            return streams;
        });
    },

    // Update stream connection status
    updateConnectionStatus: (id: string, status: 'connected' | 'disconnected' | 'error') => {
        videoStreams.update(streams => {
            const stream = streams.get(id);
            if (stream) {
                stream.connectionStatus = status;
                if (status === 'connected') {
                    stream.lastConnected = new Date();
                }
                streams.set(id, stream);
            }
            return streams;
        });
    },

    // Toggle stats display for a stream
    toggleStats: (id: string) => {
        videoStreams.update(streams => {
            const stream = streams.get(id);
            if (stream) {
                stream.statsEnabled = !stream.statsEnabled;
                streams.set(id, stream);
            }
            return streams;
        });
    },

    // Set primary video stream
    setPrimaryStream: (id: string) => {
        primaryVideoStream.set(id);
    },

    // Remove a stream
    removeStream: (id: string) => {
        videoStreams.update(streams => {
            streams.delete(id);
            return streams;
        });
    },

    // Update global statistics
    updateGlobalStats: (stats: Partial<GlobalVideoStats>) => {
        globalVideoStats.update(current => ({
            ...current,
            ...stats
        }));
    }
};

// Initialize with default primary stream
videoStreamActions.registerStream('primary', 'Primary Video Stream');