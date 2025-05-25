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
let videoStreamingActive = false; // Flag to control reconnection behavior

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
    
    // Set streaming as active
    videoStreamingActive = true;

    // Get video elements - handle case where video element was replaced with canvas
    let videoElement = document.getElementById('video-player');
    
    // If video-player doesn't exist, it might have been replaced with a canvas
    if (!videoElement) {
        // Look for a canvas in the video container
        const videoContainer = document.getElementById('video-container');
        if (videoContainer) {
            const canvas = videoContainer.querySelector('canvas');
            if (canvas) {
                console.log('🎨 Found existing canvas element, reusing it');
                videoElement = canvas;
                // Give it the video-player ID for consistency
                canvas.id = 'video-player';
            }
        }
    }
    
    videoPlayer = videoElement;
    const videoStatusDiv = document.getElementById('video-status');

    console.log('🔍 Looking for video elements...');
    console.log('videoPlayer:', videoPlayer);
    console.log('videoStatusDiv:', videoStatusDiv);
    console.log('document.readyState:', document.readyState);
    
    debugLog('🔍 Looking for video elements...');
    debugLog('videoPlayer:', videoPlayer);
    debugLog('videoStatusDiv:', videoStatusDiv);

    if (!videoPlayer || !videoStatusDiv) {
        console.error("❌ Video UI elements not found!");
        console.log('videoPlayer:', videoPlayer);
        console.log('videoStatusDiv:', videoStatusDiv);
        console.log('Available elements with video in ID:');
        const videoElements = document.querySelectorAll('[id*="video"]');
        videoElements.forEach(el => console.log(`  - ${el.id}: ${el.tagName} (visible: ${el.offsetParent !== null}, display: ${getComputedStyle(el).display})`));
        
        // Also check all elements in teleop tab
        const teleopElements = document.querySelectorAll('#teleop *[id]');
        console.log('All elements with IDs in teleop tab:');
        teleopElements.forEach(el => console.log(`  - ${el.id}: ${el.tagName}`));
        
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
    // Also directly update the header status
    if (window.updateVideoStatusHeader) {
        window.updateVideoStatusHeader('Initializing', 'initializing');
    }

    // Initialize WebCodecs H.264 decoder
    initWebCodecsDecoder();

    // Connect to video WebSocket
    debugLog('🔌 Starting WebSocket connection...');
    connectVideoWebSocket();
}

function connectVideoWebSocket() {
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.host}/ws/video`;
    
    console.log(`🔗 Connecting to Video WebSocket: ${wsUrl}`);
    debugLog(`🔗 Connecting to Video WebSocket: ${wsUrl}`);
    updateVideoStatus('Video: Connecting...', 'orange');
    // Also directly update the header status
    if (window.updateVideoStatusHeader) {
        window.updateVideoStatusHeader('Connecting', 'warning');
    }

    // Close existing connection if any
    if (videoWs && videoWs.readyState !== WebSocket.CLOSED) {
        console.log('🔄 Closing existing WebSocket connection');
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
            // Also directly update the header status
            if (window.updateVideoStatusHeader) {
                window.updateVideoStatusHeader('Connected', 'connected');
            }
            
            // Reset frame counter for proper "Streaming" status detection
            frameCount = 0;
            lastFrameTime = 0;
            fps = 0;
            
            // Fallback: Check if we're receiving frames after 2 seconds
            setTimeout(() => {
                if (frameCount > 0 && videoWs && videoWs.readyState === WebSocket.OPEN) {
                    console.log('🔄 Fallback: Ensuring streaming status is updated');
                    updateVideoStatus('Video: Streaming', 'green');
                    if (window.updateVideoStatusHeader) {
                        window.updateVideoStatusHeader('Streaming', 'connected');
                    }
                }
            }, 2000);
        };

        videoWs.onclose = (event) => {
            console.log(`❌ Video WebSocket disconnected: ${event.code}`);
            updateVideoStatus(`Video: Disconnected (${event.code})`, 'red');
            // Also directly update the header status
            if (window.updateVideoStatusHeader) {
                window.updateVideoStatusHeader('Disconnected', 'disconnected');
            }
            videoWs = null;
            
            // Only attempt to reconnect if streaming is still active
            if (videoStreamingActive) {
                debugLog('⏰ Scheduling reconnection in 5 seconds...');
                setTimeout(connectVideoWebSocket, 5000);
            } else {
                debugLog('🛑 Video streaming inactive - not reconnecting');
            }
        };

        videoWs.onerror = (error) => {
            console.error('❌ Video WebSocket error:', error);
            updateVideoStatus('Video: Error', 'red');
            // Also directly update the header status
            if (window.updateVideoStatusHeader) {
                window.updateVideoStatusHeader('Error', 'disconnected');
            }
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
    
    // Always recreate canvas to ensure proper sizing
    if (true) {
        canvas = document.createElement('canvas');
        canvas.id = 'video-player'; // Ensure it has the proper ID
        
        // Set canvas to a reasonable size for better quality
        // Use a larger canvas size that will be scaled down by CSS
        canvas.width = 800;
        canvas.height = 600;
        console.log(`📐 Setting canvas size to: ${canvas.width}x${canvas.height}`);
        
        canvas.style.width = '100%';
        canvas.style.height = '100%';
        canvas.style.objectFit = 'contain';
        canvas.style.backgroundColor = '#000';
        
        // Replace video element with canvas
        if (videoPlayer && videoPlayer.parentNode) {
            videoPlayer.parentNode.replaceChild(canvas, videoPlayer);
        }
        videoPlayer = canvas;
        console.log(`🎨 Created new canvas element: ${canvas.width}x${canvas.height}`);
    }
    
    const ctx = canvas.getContext('2d');
    
    // Initialize VideoDecoder
    const decoder = new VideoDecoder({
        output: (frame) => {
            debugLog(`🎬 Decoded frame: ${frame.displayWidth}x${frame.displayHeight}`);
            
            // Track successful decode
            stats.framesDecoded++;
            
            // Calculate aspect-ratio-aware scaling and centering
            const frameWidth = frame.displayWidth;
            const frameHeight = frame.displayHeight;
            const canvasWidth = canvas.width;
            const canvasHeight = canvas.height;
            
            // Calculate scale to fit frame in canvas while maintaining aspect ratio
            const scaleX = canvasWidth / frameWidth;
            const scaleY = canvasHeight / frameHeight;
            const scale = Math.min(scaleX, scaleY); // Use smaller scale to fit entirely
            
            // Calculate centered position
            const scaledWidth = frameWidth * scale;
            const scaledHeight = frameHeight * scale;
            const x = (canvasWidth - scaledWidth) / 2;
            const y = (canvasHeight - scaledHeight) / 2;
            
            // FORCE CENTER POSITION FOR DEBUGGING
            console.log(`BEFORE: x=${x}, y=${y}, scaledWidth=${scaledWidth}, scaledHeight=${scaledHeight}`);
            // Force to exact center
            const centerX = canvasWidth / 2 - scaledWidth / 2;
            const centerY = canvasHeight / 2 - scaledHeight / 2;
            console.log(`FORCED CENTER: centerX=${centerX}, centerY=${centerY}`);
            
            // Debug logging for first few frames
            if (stats.framesDecoded <= 5) {
                console.log(`🎬 Frame scaling: ${frameWidth}x${frameHeight} → ${scaledWidth.toFixed(1)}x${scaledHeight.toFixed(1)} at (${x.toFixed(1)}, ${y.toFixed(1)}) on ${canvasWidth}x${canvasHeight} canvas`);
                console.log(`🎬 Scale factors: scaleX=${scaleX.toFixed(2)}, scaleY=${scaleY.toFixed(2)}, final scale=${scale.toFixed(2)}`);
            }
            
            // Clear canvas with black background
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, canvasWidth, canvasHeight);
            
            // Draw the video frame centered on the canvas
            ctx.drawImage(frame, centerX, centerY, scaledWidth, scaledHeight);
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

    // Update UI when streaming starts to prevent any shaking
    if (frameCount === 1) {
        // One-time status update when streaming starts
        updateVideoStatus('Video: Streaming', 'green');
        // Also directly update the header status to ensure synchronization
        if (window.updateVideoStatusHeader) {
            window.updateVideoStatusHeader('Streaming', 'connected');
        }
        // Initialize video info display once
        initializeVideoInfoDisplay();
        console.log('🎬 Video streaming status updated to "Streaming"');
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
    
    // Safety check: If we have active frames but status isn't "Streaming", update it
    if (stats.framesReceived > 10 && stats.framesDecoded > 5) {
        const videoStatusDiv = document.getElementById('video-status');
        if (videoStatusDiv && !videoStatusDiv.textContent.includes('Streaming')) {
            console.log('🔧 Safety check: Updating status to Streaming based on active frame processing');
            updateVideoStatus('Video: Streaming', 'green');
            if (window.updateVideoStatusHeader) {
                window.updateVideoStatusHeader('Streaming', 'connected');
            }
        }
    }

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

function cleanupVideo() {
    console.log('🧹 Cleaning up Video Streaming resources...');
    
    // Mark streaming as inactive to prevent reconnection
    videoStreamingActive = false;
    
    // Close WebSocket connection
    if (videoWs && videoWs.readyState !== WebSocket.CLOSED) {
        console.log('🔌 Closing video WebSocket connection');
        videoWs.close();
        videoWs = null;
    }
    
    // Close VideoDecoder
    if (window.videoDecoder && window.videoDecoder.state !== 'closed') {
        console.log('🎬 Closing VideoDecoder');
        try {
            window.videoDecoder.close();
        } catch (e) {
            console.warn('⚠️ Error closing VideoDecoder:', e);
        }
        window.videoDecoder = null;
    }
    
    // Clear frame buffer
    frameBuffer = [];
    waitingForKeyFrame = true;
    
    // Reset statistics
    frameCount = 0;
    lastFrameTime = 0;
    fps = 0;
    stats = {
        framesReceived: 0,
        framesDecoded: 0,
        decodeErrors: 0,
        keyFrames: 0,
        totalFrameSize: 0,
        lastStatsUpdate: 0
    };
    
    // Update UI to show disconnected state
    updateVideoStatus('Video: Disconnected', 'red');
    // Also directly update the header status
    if (window.updateVideoStatusHeader) {
        window.updateVideoStatusHeader('Disconnected', 'disconnected');
    }
    
    // Clear video display (if canvas)
    if (videoPlayer && videoPlayer.tagName === 'CANVAS') {
        const ctx = videoPlayer.getContext('2d');
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, videoPlayer.width, videoPlayer.height);
    }
    
    console.log('✅ Video streaming cleanup completed');
}

// Function to resize canvas when container changes
function resizeCanvas() {
    if (videoPlayer && videoPlayer.tagName === 'CANVAS') {
        // For now, keep canvas at fixed high resolution
        const targetWidth = 800;
        const targetHeight = 600;
        
        if (videoPlayer.width !== targetWidth || videoPlayer.height !== targetHeight) {
            console.log(`🔄 Resizing canvas: ${videoPlayer.width}x${videoPlayer.height} → ${targetWidth}x${targetHeight}`);
            videoPlayer.width = targetWidth;
            videoPlayer.height = targetHeight;
            
            // Clear with black background
            const ctx = videoPlayer.getContext('2d');
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, targetWidth, targetHeight);
        }
    }
}

// Set up window resize listener for responsive canvas
window.addEventListener('resize', () => {
    setTimeout(resizeCanvas, 100); // Small delay to let layout settle
});

// Function to force canvas recreation (for testing)
function recreateCanvas() {
    console.log('🔄 Force recreating canvas...');
    if (window.videoDecoder && window.videoDecoder.state !== 'closed') {
        window.videoDecoder.close();
    }
    initWebCodecsDecoder();
}

// Export functions for main.js to call
window.initVideo = initVideo;
window.cleanupVideo = cleanupVideo;
window.resizeCanvas = resizeCanvas;
window.recreateCanvas = recreateCanvas;

debugLog('🔧 initVideo, cleanupVideo, resizeCanvas, and recreateCanvas functions exported to window'); 