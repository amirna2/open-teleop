// UI Logic for Video Streaming (WebSocket + WebCodecs H.264 Decoder)

console.log("video.js loaded");

// Video streaming state
let videoWs = null;
let videoPlayer = null;
let videoStatusCallback = null;

// Video statistics
let frameCount = 0;
let lastFrameTime = 0;
let fps = 0;

function initVideo() {
    console.log('🎬 Initializing Video Streaming...');

    // Get video elements
    videoPlayer = document.getElementById('video-player');
    const videoStatusDiv = document.getElementById('video-status');

    console.log('🔍 Looking for video elements...');
    console.log('videoPlayer:', videoPlayer);
    console.log('videoStatusDiv:', videoStatusDiv);

    if (!videoPlayer || !videoStatusDiv) {
        console.error("❌ Video UI elements not found!");
        console.log('Available elements with video in ID:');
        const videoElements = document.querySelectorAll('[id*="video"]');
        videoElements.forEach(el => console.log(`  - ${el.id}: ${el.tagName}`));
        return;
    }

    console.log('✅ Video UI elements found successfully');

    // Register status update callback
    videoStatusCallback = (message, color) => {
        console.log(`📺 Video Status Update: ${message} (${color})`);
        videoStatusDiv.textContent = message;
        videoStatusDiv.style.color = color || 'black';
    };

    updateVideoStatus('Video: Initializing...', 'orange');

    // Initialize WebCodecs H.264 decoder
    initWebCodecsDecoder();

    // Connect to video WebSocket
    console.log('🔌 Starting WebSocket connection...');
    connectVideoWebSocket();
}

function connectVideoWebSocket() {
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.host}/ws/video`;
    
    console.log(`🔗 Connecting to Video WebSocket: ${wsUrl}`);
    updateVideoStatus('Video: Connecting...', 'orange');

    // Close existing connection if any
    if (videoWs && videoWs.readyState !== WebSocket.CLOSED) {
        console.log('🔄 Closing existing WebSocket connection');
        videoWs.close();
    }

    try {
        videoWs = new WebSocket(wsUrl);
        videoWs.binaryType = 'arraybuffer'; // Important for binary data
        
        console.log('📡 WebSocket created, setting up event handlers...');

        videoWs.onopen = () => {
            console.log('✅ Video WebSocket connection opened successfully!');
            updateVideoStatus('Video: Connected', 'green');
        };

        videoWs.onclose = (event) => {
            console.log(`❌ Video WebSocket connection closed: ${event.code} - ${event.reason}`);
            updateVideoStatus(`Video: Disconnected (${event.code})`, 'red');
            videoWs = null;
            
            // Attempt to reconnect after a delay
            console.log('⏰ Scheduling reconnection in 5 seconds...');
            setTimeout(connectVideoWebSocket, 5000);
        };

        videoWs.onerror = (error) => {
            console.error('❌ Video WebSocket error:', error);
            updateVideoStatus('Video: Error', 'red');
        };

        videoWs.onmessage = (event) => {
            handleVideoMessage(event.data);
        };
        
        console.log('🎯 WebSocket event handlers set up successfully');
        
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

    console.log('🎬 Initializing WebCodecs H.264 decoder...');
    
    // Close existing decoder if any
    if (window.videoDecoder && window.videoDecoder.state !== 'closed') {
        console.log('🔄 Closing existing VideoDecoder...');
        try {
            window.videoDecoder.close();
        } catch (e) {
            console.warn('⚠️ Error closing existing decoder:', e);
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
            console.log(`🎬 Decoded frame: ${frame.displayWidth}x${frame.displayHeight}`);
            
            // Draw frame to canvas
            ctx.drawImage(frame, 0, 0, canvas.width, canvas.height);
            frame.close();
            
            updateVideoStatus(`Video: Streaming (${frameCount} frames)`, 'green');
        },
        error: (error) => {
            console.error('❌ VideoDecoder error:', error);
            updateVideoStatus('Video: Decoder error', 'red');
            
            // Try to reinitialize after a short delay
            setTimeout(() => {
                console.log('🔄 Attempting to reinitialize decoder after error...');
                initWebCodecsDecoder();
            }, 1000);
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
        updateVideoStatus('Video: WebCodecs ready', 'orange');
        
    } catch (error) {
        console.error('❌ Failed to configure VideoDecoder:', error);
        updateVideoStatus('Video: WebCodecs config failed', 'red');
    }
}

function decodeH264WithWebCodecs(h264Data) {
    if (!window.videoDecoder) {
        console.error('❌ VideoDecoder not available');
        return;
    }
    
    // Check decoder state
    if (window.videoDecoder.state === 'closed') {
        console.warn('⚠️ VideoDecoder is closed, reinitializing...');
        initWebCodecsDecoder();
        return; // Skip this frame, decoder will be ready for next one
    }
    
    if (window.videoDecoder.state !== 'configured') {
        console.warn(`⚠️ VideoDecoder not ready (state: ${window.videoDecoder.state}), skipping frame`);
        return;
    }
    
    try {
        // Create EncodedVideoChunk from H.264 data
        const chunk = new EncodedVideoChunk({
            type: isKeyFrame(h264Data) ? 'key' : 'delta',
            timestamp: performance.now() * 1000, // Convert to microseconds
            data: h264Data
        });
        
        console.log(`🎬 Decoding ${chunk.type} frame: ${h264Data.length} bytes (decoder state: ${window.videoDecoder.state})`);
        window.videoDecoder.decode(chunk);
        
    } catch (error) {
        console.error('❌ Failed to decode H.264 frame:', error);
        
        // If decoder is in error state, try to reinitialize
        if (error.name === 'InvalidStateError') {
            console.warn('🔄 Decoder in invalid state, reinitializing...');
            initWebCodecsDecoder();
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
                return true;
            }
        }
    }
    return false;
}

function handleVideoMessage(data) {
    frameCount++;
    
    // Update FPS calculation
    const currentTime = Date.now();
    if (lastFrameTime > 0) {
        const timeDiff = currentTime - lastFrameTime;
        fps = Math.round(1000 / timeDiff * 10) / 10; // Round to 1 decimal
    }
    lastFrameTime = currentTime;

    console.log(`📦 Received video frame #${frameCount}: ${data.byteLength} bytes`);
    
    // Parse FlatBuffer to extract H.264 payload
    try {
        const uint8Array = new Uint8Array(data);
        
        // Parse the FlatBuffer to extract the H.264 payload
        const h264Payload = extractH264FromFlatBuffer(uint8Array);
        
        if (h264Payload && h264Payload.length > 0) {
            console.log(`🎬 Extracted H.264 payload: ${h264Payload.length} bytes`);
            
            // Decode using WebCodecs
            if (window.videoDecoder) {
                decodeH264WithWebCodecs(h264Payload);
            } else {
                console.warn('⚠️ WebCodecs decoder not available');
            }
        } else {
            console.log(`⚠️ No H.264 payload found in FlatBuffer`);
        }
        
    } catch (error) {
        console.error('❌ Error processing video frame:', error);
    }

    // Update UI
    updateVideoStatus(`Video: Streaming (${frameCount} frames)`, 'green');
    updateVideoInfo(fps, data.byteLength);
}

function extractH264FromFlatBuffer(uint8Array) {
    try {
        const buf = new flatbuffers.ByteBuffer(uint8Array);
        
        // Read the FlatBuffer root table (OttMessage)
        const rootOffset = buf.readInt32(buf.position()) + buf.position();
        buf.setPosition(rootOffset);
        
        // Read the vtable
        const vtableOffset = buf.position() - buf.readInt16(buf.position());
        const vtableLength = buf.readInt16(vtableOffset);
        const tableLength = buf.readInt16(vtableOffset + 2);
        
        // Look for the payload field (field index varies, need to find it)
        // This is a simplified approach - in production, use generated FlatBuffer code
        
        // For now, let's search for H.264 NAL unit start codes in the raw data
        return findH264Data(uint8Array);
        
    } catch (error) {
        console.error('❌ Error parsing FlatBuffer:', error);
        return null;
    }
}

function findH264Data(uint8Array) {
    // Look for H.264 NAL unit start codes
    for (let i = 0; i < uint8Array.length - 4; i++) {
        if ((uint8Array[i] === 0x00 && uint8Array[i+1] === 0x00 && uint8Array[i+2] === 0x00 && uint8Array[i+3] === 0x01) ||
            (uint8Array[i] === 0x00 && uint8Array[i+1] === 0x00 && uint8Array[i+2] === 0x01)) {
            console.log(`🔍 Found H.264 NAL start code at offset ${i}`);
            return uint8Array.slice(i);
        }
    }
    return null;
}



function updateVideoStatus(message, color) {
    if (videoStatusCallback) {
        videoStatusCallback(message, color);
    } else {
        console.log(`📺 Video Status: ${message}`);
    }
}

function updateVideoInfo(currentFps, frameSize) {
    const resolutionSpan = document.getElementById('video-resolution');
    const fpsSpan = document.getElementById('video-fps');
    const codecSpan = document.getElementById('video-codec');

    if (resolutionSpan) {
        resolutionSpan.textContent = `Frame Size: ${frameSize} bytes`;
    }
    
    if (fpsSpan) {
        fpsSpan.textContent = `FPS: ${currentFps || '--'}`;
    }
    
    if (codecSpan) {
        codecSpan.textContent = `Frames: ${frameCount}`;
    }
}

// Export function for main.js to call - ensure it's available globally
window.initVideo = initVideo;

// Also make it available without window prefix
if (typeof globalThis !== 'undefined') {
    globalThis.initVideo = initVideo;
}

console.log('🔧 initVideo function exported to window and globalThis');
console.log('🔧 typeof window.initVideo:', typeof window.initVideo);
console.log('🔧 typeof initVideo:', typeof initVideo); 