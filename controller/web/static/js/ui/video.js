// UI Logic for Video Streaming (WebSocket + WebCodecs H.264 Decoder)

// Debug configuration - Set VIDEO_DEBUG to true for verbose logging
// This will show detailed frame processing, WebSocket events, and decoder state changes
const VIDEO_DEBUG = false;

function debugLog(...args) {
    if (VIDEO_DEBUG) {
        console.log(...args);
    }
}

console.log("video.js loaded");

// Video streaming state
let videoWs = null;
let videoPlayer = null;
let videoStatusCallback = null;

// Video statistics
let frameCount = 0;
let lastFrameTime = 0;
let fps = 0;

// Frame buffering for key frame synchronization
let frameBuffer = [];
let waitingForKeyFrame = true;

// Enhanced statistics tracking
let stats = {
    framesReceived: 0,
    framesDecoded: 0,
    decodeErrors: 0,
    keyFrames: 0,
    totalFrameSize: 0,
    lastStatsUpdate: 0
};

function initVideo() {
    console.log('🎬 Initializing Video Streaming...');

    // Get video elements
    videoPlayer = document.getElementById('video-player');
    const videoStatusDiv = document.getElementById('video-status');

    debugLog('🔍 Looking for video elements...');
    debugLog('videoPlayer:', videoPlayer);
    debugLog('videoStatusDiv:', videoStatusDiv);

    if (!videoPlayer || !videoStatusDiv) {
        console.error("❌ Video UI elements not found!");
        debugLog('Available elements with video in ID:');
        const videoElements = document.querySelectorAll('[id*="video"]');
        videoElements.forEach(el => debugLog(`  - ${el.id}: ${el.tagName}`));
        return;
    }

    debugLog('✅ Video UI elements found successfully');

    // Register status update callback
    videoStatusCallback = (message, color) => {
        debugLog(`📺 Video Status Update: ${message} (${color})`);
        videoStatusDiv.textContent = message;
        videoStatusDiv.style.color = color || 'black';
    };

    updateVideoStatus('Video: Initializing...', 'orange');

    // Initialize WebCodecs H.264 decoder
    initWebCodecsDecoder();

    // Connect to video WebSocket
    debugLog('🔌 Starting WebSocket connection...');
    connectVideoWebSocket();
}

function connectVideoWebSocket() {
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.host}/ws/video`;
    
    debugLog(`🔗 Connecting to Video WebSocket: ${wsUrl}`);
    updateVideoStatus('Video: Connecting...', 'orange');

    // Close existing connection if any
    if (videoWs && videoWs.readyState !== WebSocket.CLOSED) {
        debugLog('🔄 Closing existing WebSocket connection');
        videoWs.close();
    }

    try {
        videoWs = new WebSocket(wsUrl);
        videoWs.binaryType = 'arraybuffer'; // Important for binary data
        
        debugLog('📡 WebSocket created, setting up event handlers...');

        videoWs.onopen = () => {
            console.log('✅ Video WebSocket connected');
            updateVideoStatus('Video: Connected', 'green');
        };

        videoWs.onclose = (event) => {
            console.log(`❌ Video WebSocket disconnected: ${event.code}`);
            updateVideoStatus(`Video: Disconnected (${event.code})`, 'red');
            videoWs = null;
            
            // Attempt to reconnect after a delay
            debugLog('⏰ Scheduling reconnection in 5 seconds...');
            setTimeout(connectVideoWebSocket, 5000);
        };

        videoWs.onerror = (error) => {
            console.error('❌ Video WebSocket error:', error);
            updateVideoStatus('Video: Error', 'red');
        };

        videoWs.onmessage = (event) => {
            handleVideoMessage(event.data);
        };
        
        debugLog('🎯 WebSocket event handlers set up successfully');
        
    } catch (error) {
        console.error('💥 Failed to create WebSocket:', error);
        updateVideoStatus('Video: Failed to connect', 'red');
    }
}

function initWebCodecsDecoder() {
    // Check for WebCodecs API support
    if (!window.VideoDecoder) {
        console.error('❌ WebCodecs API not supported in this browser');
        updateVideoStatus('Video: WebCodecs not supported', 'red');
        return;
    }

    debugLog('🎬 Initializing WebCodecs H.264 decoder...');
    
    // Close existing decoder if any
    if (window.videoDecoder && window.videoDecoder.state !== 'closed') {
        debugLog('🔄 Closing existing VideoDecoder...');
        try {
            window.videoDecoder.close();
        } catch (e) {
            debugLog('⚠️ Error closing existing decoder:', e);
        }
    }
    
    // Create or reuse canvas for video rendering
    let canvas = videoPlayer;
    if (!canvas || canvas.tagName !== 'CANVAS') {
        canvas = document.createElement('canvas');
        canvas.width = 320;
        canvas.height = 240;
        canvas.style.width = '100%';
        canvas.style.height = '100%';
        canvas.style.objectFit = 'contain';
        
        // Replace video element with canvas
        if (videoPlayer && videoPlayer.parentNode) {
            videoPlayer.parentNode.replaceChild(canvas, videoPlayer);
        }
        videoPlayer = canvas;
    }
    
    const ctx = canvas.getContext('2d');
    
    // Initialize VideoDecoder
    const decoder = new VideoDecoder({
        output: (frame) => {
            debugLog(`🎬 Decoded frame: ${frame.displayWidth}x${frame.displayHeight}`);
            
            // Track successful decode
            stats.framesDecoded++;
            
            // Draw frame to canvas
            ctx.drawImage(frame, 0, 0, canvas.width, canvas.height);
            frame.close();
            
            // Don't update status here - it's handled in handleVideoMessage
        },
        error: (error) => {
            console.error('❌ VideoDecoder error:', error);
            
            // Track decoder errors
            stats.decodeErrors++;
            
            // Don't update status for every error to avoid UI spam
            debugLog('🔄 Decoder error occurred, will reinitialize...');
            
            // Reset state and reinitialize
            waitingForKeyFrame = true;
            frameBuffer = [];
            
            // Try to reinitialize after a short delay
            setTimeout(() => {
                debugLog('🔄 Attempting to reinitialize decoder after error...');
                initWebCodecsDecoder();
            }, 500); // Shorter delay for faster recovery
        }
    });
    
    // Configure decoder for H.264
    const config = {
        codec: 'avc1.42E01E', // H.264 Baseline Profile Level 3.0
        codedWidth: 320,
        codedHeight: 240,
    };
    
    try {
        decoder.configure(config);
        console.log('✅ WebCodecs VideoDecoder configured');
        
        // Store decoder for use in frame processing
        window.videoDecoder = decoder;
        
        // Reset frame buffering state
        waitingForKeyFrame = true;
        frameBuffer = [];
        
        updateVideoStatus('Video: WebCodecs ready', 'orange');
        
    } catch (error) {
        console.error('❌ Failed to configure VideoDecoder:', error);
        updateVideoStatus('Video: WebCodecs config failed', 'red');
    }
}

function decodeH264WithWebCodecs(h264Data) {
    if (!window.videoDecoder) {
        debugLog('❌ VideoDecoder not available');
        return;
    }
    
    // Check decoder state
    if (window.videoDecoder.state === 'closed') {
        debugLog('⚠️ VideoDecoder is closed, reinitializing...');
        initWebCodecsDecoder();
        waitingForKeyFrame = true; // Reset key frame waiting
        return;
    }
    
    if (window.videoDecoder.state !== 'configured') {
        debugLog(`⚠️ VideoDecoder not ready (state: ${window.videoDecoder.state}), skipping frame`);
        return;
    }
    
    const isKey = isKeyFrame(h264Data);
    
    // If we're waiting for a key frame and this isn't one, buffer it
    if (waitingForKeyFrame && !isKey) {
        debugLog('⏳ Buffering delta frame while waiting for key frame');
        frameBuffer.push(h264Data);
        return;
    }
    
    // If this is a key frame, we can start decoding
    if (isKey && waitingForKeyFrame) {
        debugLog('🔑 Key frame received, starting decode sequence');
        waitingForKeyFrame = false;
        
        // Decode the key frame first
        decodeFrame(h264Data, 'key');
        
        // Then decode any buffered frames
        while (frameBuffer.length > 0) {
            const bufferedFrame = frameBuffer.shift();
            decodeFrame(bufferedFrame, 'delta');
        }
        return;
    }
    
    // Normal decoding for subsequent frames
    if (!waitingForKeyFrame) {
        decodeFrame(h264Data, isKey ? 'key' : 'delta');
    }
}

function decodeFrame(h264Data, frameType) {
    try {
        // Additional validation before creating chunk
        if (!h264Data || h264Data.length === 0) {
            debugLog('⚠️ Empty H.264 data, skipping frame');
            return;
        }
        
        const chunk = new EncodedVideoChunk({
            type: frameType,
            timestamp: performance.now() * 1000, // Convert to microseconds
            data: h264Data
        });
        
        debugLog(`🎬 Decoding ${frameType} frame: ${h264Data.length} bytes`);
        window.videoDecoder.decode(chunk);
        
    } catch (error) {
        debugLog('❌ Failed to decode H.264 frame:', error);
        
        // If we get a DataError about key frames, reset and wait for next key frame
        if (error.name === 'DataError' && error.message.includes('key frame')) {
            debugLog('🔄 Key frame required, resetting decoder state...');
            waitingForKeyFrame = true;
            frameBuffer = [];
            return;
        }
        
        // For other DataErrors (corrupted frames), just skip and continue
        if (error.name === 'DataError') {
            debugLog('⚠️ Corrupted frame detected, skipping...');
            stats.decodeErrors++;
            return;
        }
        
        // If decoder is in error state, try to reinitialize
        if (error.name === 'InvalidStateError') {
            debugLog('🔄 Decoder in invalid state, reinitializing...');
            initWebCodecsDecoder();
            waitingForKeyFrame = true;
            frameBuffer = [];
        }
    }
}

function isKeyFrame(h264Data) {
    // Check if this is a key frame (I-frame) by looking for SPS/PPS/IDR NAL units
    // NAL unit type is in the first byte after start code
    for (let i = 0; i < h264Data.length - 4; i++) {
        if (h264Data[i] === 0x00 && h264Data[i+1] === 0x00 && 
            h264Data[i+2] === 0x00 && h264Data[i+3] === 0x01) {
            const nalType = h264Data[i+4] & 0x1F;
            // NAL types: 7=SPS, 8=PPS, 5=IDR (key frame)
            if (nalType === 5 || nalType === 7 || nalType === 8) {
                debugLog(`🔑 Key frame detected (NAL type: ${nalType})`);
                return true;
            }
        }
    }
    return false;
}

function handleVideoMessage(data) {
    frameCount++;
    stats.framesReceived++;
    stats.totalFrameSize += data.byteLength;
    
    // Update FPS calculation
    const currentTime = Date.now();
    if (lastFrameTime > 0) {
        const timeDiff = currentTime - lastFrameTime;
        fps = Math.round(1000 / timeDiff * 10) / 10; // Round to 1 decimal
    }
    lastFrameTime = currentTime;

    debugLog(`📦 Received video frame #${frameCount}: ${data.byteLength} bytes`);
    
    // Parse FlatBuffer to extract H.264 payload
    try {
        const uint8Array = new Uint8Array(data);
        
        // Parse the FlatBuffer to extract the H.264 payload
        const h264Payload = extractH264FromFlatBuffer(uint8Array);
        
        if (h264Payload && h264Payload.length > 0) {
            debugLog(`🎬 Extracted H.264 payload: ${h264Payload.length} bytes`);
            
            // Check if this is a key frame for statistics
            if (isKeyFrame(h264Payload)) {
                stats.keyFrames++;
            }
            
            // Decode using WebCodecs
            if (window.videoDecoder) {
                decodeH264WithWebCodecs(h264Payload);
            } else {
                debugLog('⚠️ WebCodecs decoder not available');
                stats.decodeErrors++;
            }
        } else {
            debugLog(`⚠️ No H.264 payload found in FlatBuffer`);
            stats.decodeErrors++;
        }
        
    } catch (error) {
        console.error('❌ Error processing video frame:', error);
        stats.decodeErrors++;
    }

    // Update UI only once when streaming starts to prevent any shaking
    if (frameCount === 1) {
        // One-time status update when streaming starts
        updateVideoStatus('Video: Streaming', 'green');
        // Initialize video info display once
        initializeVideoInfoDisplay();
    }
    
    // Update statistics (throttled to once per second)
    updateVideoStats();
}

function extractH264FromFlatBuffer(uint8Array) {
    try {
        // First try to find H.264 data using NAL unit search
        const h264Data = findH264Data(uint8Array);
        
        if (h264Data && h264Data.length > 4) {
            // Validate that we have a complete NAL unit
            if (isValidH264Data(h264Data)) {
                return h264Data;
            } else {
                debugLog('⚠️ Invalid H.264 data detected, skipping frame');
                return null;
            }
        }
        
        debugLog('⚠️ No valid H.264 data found in FlatBuffer');
        return null;
        
    } catch (error) {
        console.error('❌ Error parsing FlatBuffer:', error);
        return null;
    }
}

function isValidH264Data(data) {
    // Basic validation: check for proper NAL unit structure
    if (data.length < 5) return false;
    
    // Check for start code (0x00 0x00 0x00 0x01 or 0x00 0x00 0x01)
    let hasStartCode = false;
    if (data[0] === 0x00 && data[1] === 0x00) {
        if (data[2] === 0x00 && data[3] === 0x01) {
            hasStartCode = true;
        } else if (data[2] === 0x01) {
            hasStartCode = true;
        }
    }
    
    if (!hasStartCode) {
        debugLog('⚠️ H.264 data missing start code');
        return false;
    }
    
    return true;
}

function findH264Data(uint8Array) {
    // Look for H.264 NAL unit start codes
    for (let i = 0; i < uint8Array.length - 4; i++) {
        if ((uint8Array[i] === 0x00 && uint8Array[i+1] === 0x00 && uint8Array[i+2] === 0x00 && uint8Array[i+3] === 0x01) ||
            (uint8Array[i] === 0x00 && uint8Array[i+1] === 0x00 && uint8Array[i+2] === 0x01)) {
            debugLog(`🔍 Found H.264 NAL start code at offset ${i}`);
            return uint8Array.slice(i);
        }
    }
    return null;
}



function updateVideoStatus(message, color) {
    if (videoStatusCallback) {
        videoStatusCallback(message, color);
    } else {
        debugLog(`📺 Video Status: ${message}`);
    }
}

function initializeVideoInfoDisplay() {
    // Set static video info once when streaming starts
    const resolutionSpan = document.getElementById('video-resolution');
    const codecSpan = document.getElementById('video-codec');

    if (resolutionSpan) {
        resolutionSpan.textContent = `Resolution: 320x240`;
    }
    
    if (codecSpan) {
        codecSpan.textContent = `Codec: H.264`;
    }
}

function updateVideoStats() {
    // Update statistics display (throttled to avoid UI shaking)
    const currentTime = Date.now();
    if (currentTime - stats.lastStatsUpdate < 1000) {
        return; // Update only once per second
    }
    stats.lastStatsUpdate = currentTime;

    const framesElement = document.getElementById('stats-frames');
    const decodedElement = document.getElementById('stats-decoded');
    const errorsElement = document.getElementById('stats-errors');
    const errorRateElement = document.getElementById('stats-error-rate');
    const fpsElement = document.getElementById('stats-fps');
    const frameSizeElement = document.getElementById('stats-frame-size');
    const keyFramesElement = document.getElementById('stats-key-frames');

    if (framesElement) {
        framesElement.textContent = stats.framesReceived.toLocaleString();
    }
    
    if (decodedElement) {
        decodedElement.textContent = stats.framesDecoded.toLocaleString();
    }
    
    if (errorsElement) {
        errorsElement.textContent = stats.decodeErrors.toLocaleString();
    }
    
    if (errorRateElement) {
        const errorRate = stats.framesReceived > 0 ? 
            ((stats.decodeErrors / stats.framesReceived) * 100).toFixed(1) : '0.0';
        errorRateElement.textContent = `${errorRate}%`;
        
        // Color code the error rate
        if (parseFloat(errorRate) > 5) {
            errorRateElement.style.color = '#dc3545'; // Red for high error rate
        } else if (parseFloat(errorRate) > 1) {
            errorRateElement.style.color = '#fd7e14'; // Orange for medium error rate
        } else {
            errorRateElement.style.color = '#28a745'; // Green for low error rate
        }
    }
    
    if (fpsElement) {
        fpsElement.textContent = fps ? fps.toFixed(1) : '--';
    }
    
    if (frameSizeElement) {
        const avgSize = stats.framesReceived > 0 ? 
            Math.round(stats.totalFrameSize / stats.framesReceived) : 0;
        frameSizeElement.textContent = `${avgSize.toLocaleString()} bytes`;
    }
    
    if (keyFramesElement) {
        keyFramesElement.textContent = stats.keyFrames.toLocaleString();
    }
}

function updateVideoInfo(currentFps, frameSize) {
    // This function is now unused during streaming to prevent shaking
    // Video info is set once in initializeVideoInfoDisplay()
}

// Export function for main.js to call
window.initVideo = initVideo;

debugLog('🔧 initVideo function exported to window'); 