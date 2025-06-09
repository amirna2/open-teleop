// WebCodecs H.264 Player - TypeScript Port
// CRITICAL: This is an EXACT functional port of video-simpler.js
// NO CHANGES to video logic, performance characteristics, or behavior

// Video debug configuration - Control logging granularly
interface VideoDebugConfig {
    enabled: boolean;
    connection: boolean;    // WebSocket connection events
    frames: boolean;        // Per-frame processing (very verbose, floods console)
    parsing: boolean;       // NAL unit parsing details (extremely verbose)
    stats: boolean;         // Stats updates (once per second)
    decoder: boolean;       // Decoder configuration events
    errors: boolean;        // Always log errors
}

const VIDEO_DEBUG: VideoDebugConfig = {
    enabled: true,
    connection: true,
    frames: false,
    parsing: false,
    stats: false,
    decoder: true,
    errors: true
};

function debugLog(category: keyof VideoDebugConfig | 'error', ...args: any[]): void {
    if (VIDEO_DEBUG.enabled && (VIDEO_DEBUG[category] || category === 'error')) {
        console.log(`[VIDEO-${category.toUpperCase()}]`, ...args);
    }
}

// Stats interface matching original exactly
interface VideoStats {
    framesReceived: number;
    framesDecoded: number;
    decodeErrors: number;
    keyFrames: number;
    totalFrameSize: number;
    lastStatsUpdate: number;
}

// Frame parsing result interface
interface FrameParseResult {
    sps: Uint8Array | null;
    pps: Uint8Array | null;
    isKeyFrame: boolean;
    complete: boolean;
}

// Status callback types
type StatusCallback = (message: string, color: string) => void;
type HeaderStatusCallback = (status: string, state: 'connected' | 'disconnected') => void;

export class WebCodecsVideoPlayer {
    private videoWs: WebSocket | null = null;
    private decoder: VideoDecoder | null = null;
    private canvas: HTMLCanvasElement | null = null;
    private ctx: CanvasRenderingContext2D | null = null;
    private videoStreamingActive: boolean = false;
    private lastSPS: Uint8Array | null = null;
    private lastPPS: Uint8Array | null = null;
    private decoderConfigured: boolean = false;

    // Stats tracking - exact same structure as original
    private frameCount: number = 0;
    private fps: number = 0;
    private lastFrameTime: number = 0;
    private stats: VideoStats = {
        framesReceived: 0,
        framesDecoded: 0,
        decodeErrors: 0,
        keyFrames: 0,
        totalFrameSize: 0,
        lastStatsUpdate: 0
    };

    // Callbacks for status updates
    private onStatusUpdate?: StatusCallback;
    private onHeaderStatusUpdate?: HeaderStatusCallback;

    constructor(
        canvasElement: HTMLCanvasElement,
        statusCallback?: StatusCallback,
        headerStatusCallback?: HeaderStatusCallback
    ) {
        debugLog("connection", "WebCodecs player initialized");
        this.canvas = canvasElement;
        this.ctx = this.canvas.getContext('2d');
        this.onStatusUpdate = statusCallback;
        this.onHeaderStatusUpdate = headerStatusCallback;
        
        // Apply same styling as original
        this.canvas.style.width = '100%';
        this.canvas.style.height = '100%';
        this.canvas.style.backgroundColor = 'black';
    }

    public initialize(): void {
        debugLog("connection", "Initializing Video with WebCodecs…");
        this.videoStreamingActive = true;
        this.connectVideoWebSocket();
    }

    private connectVideoWebSocket(): void {
        const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
        const wsUrl = `${wsProtocol}//${window.location.host}/ws/video`;
        debugLog("connection", `🔗 Connecting to Video WebSocket: ${wsUrl}`);

        if (this.videoWs) this.videoWs.close();
        this.videoWs = new WebSocket(wsUrl);
        this.videoWs.binaryType = "arraybuffer";

        this.videoWs.onopen = () => {
            debugLog("connection", "✅ Video WebSocket connected");
            this.frameCount = 0;
            this.lastFrameTime = 0;
            this.fps = 0;
            this.updateVideoStatus("Video: Connected", "green");
            this.onHeaderStatusUpdate?.("Connected", "connected");
        };

        this.videoWs.onmessage = (evt) => this.handleVideoMessage(evt.data);

        this.videoWs.onclose = () => {
            debugLog("connection", "❌ Video WebSocket disconnected");
            this.updateVideoStatus("Video: Disconnected", "red");
            this.onHeaderStatusUpdate?.("Disconnected", "disconnected");
            if (this.videoStreamingActive) {
                setTimeout(() => this.connectVideoWebSocket(), 5000);
            }
        };

        this.videoWs.onerror = (err) => {
            debugLog("error", "❌ WebSocket error:", err);
            this.updateVideoStatus("Video: Error", "red");
            this.onHeaderStatusUpdate?.("Error", "disconnected");
        };
    }

    private handleVideoMessage(data: ArrayBuffer): void {
        this.frameCount++;
        this.stats.framesReceived++;
        this.stats.totalFrameSize += data.byteLength;

        const now = Date.now();
        if (this.lastFrameTime) {
            this.fps = Math.round((1000 / (now - this.lastFrameTime)) * 10) / 10;
        }
        this.lastFrameTime = now;

        const uint8Array = new Uint8Array(data);
        const { sps, pps, isKeyFrame, complete } = this.extractSPSPPSandKeyFrame(uint8Array);

        if (!complete) {
            console.warn('⚠️ Incomplete frame received, skipping.');
            return;
        }

        if (isKeyFrame) {
            this.stats.keyFrames++;
            debugLog("frames", `🔑 Key frame detected! Frame ${this.frameCount}`);
        }

        if (sps && pps) {
            if (!this.arraysEqual(sps, this.lastSPS) || !this.arraysEqual(pps, this.lastPPS)) {
                this.lastSPS = sps;
                this.lastPPS = pps;
                debugLog("decoder", "📋 New SPS/PPS detected, configuring decoder…");
                this.configureDecoder(sps, pps);
                this.decoderConfigured = true;
            } else if (!this.decoderConfigured) {
                debugLog("decoder", "📋 First SPS/PPS detected, configuring decoder…");
                this.configureDecoder(sps, pps);
                this.decoderConfigured = true;
            }
        }

        if (this.decoder && this.decoder.state === 'configured') {
            try {
                const chunk = new EncodedVideoChunk({
                    type: isKeyFrame ? 'key' : 'delta',
                    timestamp: performance.now() * 1000,
                    data: uint8Array
                });
                this.decoder.decode(chunk);
            } catch (e) {
                debugLog("error", '❌ WebCodecs decode error:', e);
                this.stats.decodeErrors++;
            }
        }

        if (this.frameCount === 1) {
            this.updateVideoStatus("Video: Streaming", "green");
            this.onHeaderStatusUpdate?.("Streaming", "connected");
        }

        this.updateVideoStats();
    }

    private arraysEqual(a: Uint8Array | null, b: Uint8Array | null): boolean {
        if (!a || !b || a.length !== b.length) return false;
        for (let i = 0; i < a.length; i++) {
            if (a[i] !== b[i]) return false;
        }
        return true;
    }

    private extractSPSPPSandKeyFrame(buffer: Uint8Array): FrameParseResult {
        let offset = 0;
        let sps: Uint8Array | null = null;
        let pps: Uint8Array | null = null;
        let isKeyFrame = false;
        let complete = true;

        debugLog("parsing", `🔍 Parsing frame: ${buffer.length} bytes`);

        while (offset + 4 < buffer.length) {
            const nalLength = (buffer[offset] << 24) | (buffer[offset+1] << 16) |
                              (buffer[offset+2] << 8) | buffer[offset+3];
            
            debugLog("parsing", `📐 NAL at offset ${offset}: length=${nalLength}`);
            
            if (nalLength <= 0 || offset + 4 + nalLength > buffer.length) {
                debugLog("error", `❌ Invalid NAL: length=${nalLength}, would exceed buffer (${offset + 4 + nalLength} > ${buffer.length})`);
                complete = false;
                break;
            }

            const nalType = buffer[offset + 4] & 0x1F;
            debugLog("parsing", `🔢 NAL type: ${nalType}`);
            
            if (nalType === 7) sps = buffer.slice(offset, offset + 4 + nalLength);
            if (nalType === 8) pps = buffer.slice(offset, offset + 4 + nalLength);
            if (nalType === 5 || nalType === 7) isKeyFrame = true;

            offset += 4 + nalLength;
        }

        debugLog("parsing", `✅ Frame parsing result: complete=${complete}, isKeyFrame=${isKeyFrame}, sps=${!!sps}, pps=${!!pps}`);
        return { sps, pps, isKeyFrame, complete };
    }

    private configureDecoder(sps: Uint8Array, pps: Uint8Array): void {
        const description = this.createAVCDecoderConfigFromAVC(sps, pps);
        if (this.decoder) this.decoder.close();
        
        this.decoder = new VideoDecoder({
            output: (frame) => {
                if (this.canvas && this.ctx) {
                    this.canvas.width = frame.displayWidth;
                    this.canvas.height = frame.displayHeight;
                    this.ctx.drawImage(frame, 0, 0);
                    this.stats.framesDecoded++;
                    frame.close();
                }
            },
            error: (e) => {
                debugLog("error", '❌ WebCodecs error:', e);
                this.stats.decodeErrors++;
            }
        });

        this.decoder.configure({
            codec: 'avc1.42E01E',
            description: description,
            hardwareAcceleration: 'prefer-software',
            optimizeForLatency: true
        });
        
        debugLog("decoder", "✅ Decoder configured");
    }

    private createAVCDecoderConfigFromAVC(sps: Uint8Array, pps: Uint8Array): Uint8Array {
        const spsData = sps.slice(4);
        const ppsData = pps.slice(4);
        const avccSize = 8 + 2 + spsData.length + 1 + 2 + ppsData.length;
        const avcc = new Uint8Array(avccSize);
        let offset = 0;
        
        avcc[offset++] = 0x01;
        avcc[offset++] = spsData[1];
        avcc[offset++] = spsData[2];
        avcc[offset++] = spsData[3];
        avcc[offset++] = 0xFF;
        avcc[offset++] = 0xE1;
        avcc[offset++] = (spsData.length >> 8) & 0xFF;
        avcc[offset++] = spsData.length & 0xFF;
        avcc.set(spsData, offset);
        offset += spsData.length;
        avcc[offset++] = 0x01;
        avcc[offset++] = (ppsData.length >> 8) & 0xFF;
        avcc[offset++] = ppsData.length & 0xFF;
        avcc.set(ppsData, offset);
        
        return avcc;
    }

    private updateVideoStats(): void {
        const now = performance.now();
        if (now - this.stats.lastStatsUpdate < 1000) return;
        this.stats.lastStatsUpdate = now;

        debugLog("stats", `📊 Updating stats: Frames=${this.stats.framesReceived}, Decoded=${this.stats.framesDecoded}, FPS=${this.fps}`);
    }

    private updateVideoStatus(msg: string, color: string): void {
        this.onStatusUpdate?.(msg, color);
        debugLog("connection", `📺 ${msg}`);
    }

    public cleanup(): void {
        debugLog("connection", "🧹 Cleaning up video…");
        this.videoStreamingActive = false;
        
        if (this.videoWs) {
            this.videoWs.close();
            this.videoWs = null;
        }
        
        if (this.decoder) {
            this.decoder.close();
            this.decoder = null;
        }
        
        // Reset stats
        this.frameCount = 0;
        this.fps = 0;
        this.stats = {
            framesReceived: 0,
            framesDecoded: 0,
            decodeErrors: 0,
            keyFrames: 0,
            totalFrameSize: 0,
            lastStatsUpdate: 0
        };
        
        this.updateVideoStatus("Video: Disconnected", "red");
        this.onHeaderStatusUpdate?.("Disconnected", "disconnected");
    }

    // Public getters for stats (for Svelte reactivity)
    public getStats(): VideoStats {
        return { ...this.stats };
    }

    public getFPS(): number {
        return this.fps;
    }

    public getFrameCount(): number {
        return this.frameCount;
    }

    public isConfigured(): boolean {
        return this.decoderConfigured;
    }

    public isActive(): boolean {
        return this.videoStreamingActive;
    }
}

debugLog("connection", "🔧 webcodecs-player.ts fully loaded and ready!");