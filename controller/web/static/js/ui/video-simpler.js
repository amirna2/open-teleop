// video.js – WebCodecs H.264 Player (Refactored)

// Video debug configuration - Control logging granularly
// To enable/disable specific log categories, modify the boolean values below
const VIDEO_DEBUG = {
    enabled: true,
    connection: true,    // WebSocket connection events
    frames: false,       // Per-frame processing (very verbose, floods console)
    parsing: false,      // NAL unit parsing details (extremely verbose, use only for debugging frame issues)
    stats: false,        // Stats updates (once per second)
    decoder: true,       // Decoder configuration events
    errors: true         // Always log errors
};

function debugLog(category, ...args) {
    if (VIDEO_DEBUG.enabled && (VIDEO_DEBUG[category] || category === 'error')) {
        console.log(`[VIDEO-${category.toUpperCase()}]`, ...args);
    }
}

debugLog("connection", "video-simpler.js loaded");

let videoWs = null;
let decoder = null;
let canvas = null;
let ctx = null;
let videoStreamingActive = false;
let lastSPS = null;
let lastPPS = null;
let decoderConfigured = false;

// Stats tracking
let frameCount = 0;
let fps = 0;
let lastFrameTime = 0;
let stats = {
  framesReceived: 0,
  framesDecoded: 0,
  decodeErrors: 0,
  keyFrames: 0,
  totalFrameSize: 0,
  lastStatsUpdate: 0
};

function initVideo() {
    debugLog("connection", "Initializing Video with WebCodecs…");

    const videoElement = document.getElementById("video-player");
    if (!videoElement) {
        console.error('❌ <video id="video-player"> not found!');
        return;
    }

    // Replace video with canvas
    canvas = document.createElement('canvas');
    canvas.id = 'video-canvas';
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    canvas.style.backgroundColor = 'black';
    videoElement.parentNode.replaceChild(canvas, videoElement);
    ctx = canvas.getContext('2d');

    videoStreamingActive = true;
    connectVideoWebSocket();
}

function connectVideoWebSocket() {
    const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsUrl = `${wsProtocol}//${window.location.host}/ws/video`;
    debugLog("connection", `🔗 Connecting to Video WebSocket: ${wsUrl}`);

    if (videoWs) videoWs.close();
    videoWs = new WebSocket(wsUrl);
    videoWs.binaryType = "arraybuffer";

    videoWs.onopen = () => {
        debugLog("connection", "✅ Video WebSocket connected");
        frameCount = 0;
        lastFrameTime = 0;
        fps = 0;
        updateVideoStatus("Video: Connected", "green");
        if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Connected", "connected");
    };

    videoWs.onmessage = (evt) => handleVideoMessage(evt.data);

    videoWs.onclose = () => {
        debugLog("connection", "❌ Video WebSocket disconnected");
        updateVideoStatus("Video: Disconnected", "red");
        if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Disconnected", "disconnected");
        if (videoStreamingActive) setTimeout(connectVideoWebSocket, 5000);
    };

    videoWs.onerror = (err) => {
        debugLog("error", "❌ WebSocket error:", err);
        updateVideoStatus("Video: Error", "red");
        if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Error", "disconnected");
    };
}

function handleVideoMessage(data) {
    frameCount++;
    stats.framesReceived++;
    stats.totalFrameSize += data.byteLength;
  
    const now = Date.now();
    if (lastFrameTime) {
      fps = Math.round((1000 / (now - lastFrameTime)) * 10) / 10;
    }
    lastFrameTime = now;

    const uint8Array = new Uint8Array(data);
    const { sps, pps, isKeyFrame, complete } = extractSPSPPSandKeyFrame(uint8Array);

    if (!complete) {
        console.warn('⚠️ Incomplete frame received, skipping.');
        return;
    }

    if (isKeyFrame) {
        stats.keyFrames++;
        debugLog("frames", `🔑 Key frame detected! Frame ${frameCount}`);
    }

    if (sps && pps) {
        if (!arraysEqual(sps, lastSPS) || !arraysEqual(pps, lastPPS)) {
            lastSPS = sps;
            lastPPS = pps;
            debugLog("decoder", "📋 New SPS/PPS detected, configuring decoder…");
            configureDecoder(sps, pps);
            decoderConfigured = true;
        } else if (!decoderConfigured) {
            debugLog("decoder", "📋 First SPS/PPS detected, configuring decoder…");
            configureDecoder(sps, pps);
            decoderConfigured = true;
        }
    }

    if (decoder && decoder.state === 'configured') {
        try {
            const chunk = new EncodedVideoChunk({
                type: isKeyFrame ? 'key' : 'delta',
                timestamp: performance.now() * 1000,
                data: uint8Array
            });
            decoder.decode(chunk);
        } catch (e) {
            debugLog("error", '❌ WebCodecs decode error:', e);
            stats.decodeErrors++;
        }
    }

    if (frameCount === 1) {
        updateVideoStatus("Video: Streaming", "green");
        if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Streaming", "connected");
        initializeVideoInfoDisplay();
    }
  
    updateVideoStats();
}

function arraysEqual(a, b) {
    if (!a || !b || a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) {
        if (a[i] !== b[i]) return false;
    }
    return true;
}

function extractSPSPPSandKeyFrame(buffer) {
    let offset = 0;
    let sps = null, pps = null, isKeyFrame = false;
    let complete = true;

    debugLog("parsing", `🔍 Parsing frame: ${buffer.length} bytes`);

    while (offset + 4 < buffer.length) {
        const nalLength = (buffer[offset] << 24) | (buffer[offset+1] << 16) |
                          (buffer[offset+2] << 8) | buffer[offset+3];
        
        debugLog("parsing", `📐 NAL at offset ${offset}: length=${nalLength}`);
        
        if (nalLength <= 0 || offset + 4 + nalLength > buffer.length) {
            debugLog("error", `❌ Invalid NAL: length=${nalLength}, would exceed buffer (${offset + 4 + nalLength} > ${buffer.length})`);
            complete = false; // Frame incomplete
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

function configureDecoder(sps, pps) {
    const description = createAVCDecoderConfigFromAVC(sps, pps);
    if (decoder) decoder.close(); // Reset decoder if it exists
    decoder = new VideoDecoder({
        output: frame => {
            canvas.width = frame.displayWidth;
            canvas.height = frame.displayHeight;
            ctx.drawImage(frame, 0, 0);
            stats.framesDecoded++;
            frame.close();
        },
        error: e => {
            debugLog("error", '❌ WebCodecs error:', e);
            stats.decodeErrors++;
        }
    });
    decoder.configure({
        codec: 'avc1.42E01E',
        description: description,
        hardwareAcceleration: 'prefer-software',
        optimizeForLatency: true
    });
    debugLog("decoder", "✅ Decoder configured");
}

function createAVCDecoderConfigFromAVC(sps, pps) {
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

function initializeVideoInfoDisplay() {
  const res = document.getElementById("video-resolution");
  const codec = document.getElementById("video-codec");
  if (res) res.textContent = "Resolution: Auto";
  if (codec) codec.textContent = "Codec: H.264 (WebCodecs)";
}

function updateVideoStats() {
  const now = performance.now();
  if (now - stats.lastStatsUpdate < 1000) return;
  stats.lastStatsUpdate = now;

  debugLog("stats", `📊 Updating stats: Frames=${stats.framesReceived}, Decoded=${stats.framesDecoded}, FPS=${fps}`);

  const fEl = document.getElementById("stats-frames");
  const dEl = document.getElementById("stats-decoded");
  const eEl = document.getElementById("stats-errors");
  const erEl = document.getElementById("stats-error-rate");
  const fpsEl = document.getElementById("stats-fps");
  const szEl = document.getElementById("stats-frame-size");
  const kfEl = document.getElementById("stats-key-frames");

  // Debug: Check if elements exist
  if (!fEl) {
    console.error("❌ stats-frames element not found!");
    return;
  }

  if (fEl) fEl.textContent = stats.framesReceived.toLocaleString();
  if (dEl) dEl.textContent = stats.framesDecoded.toLocaleString();
  if (eEl) eEl.textContent = stats.decodeErrors.toLocaleString();
  if (erEl) {
    const errorRate = stats.framesReceived > 0 ? 
      (stats.decodeErrors / stats.framesReceived * 100).toFixed(1) : 0;
    erEl.textContent = `${errorRate}%`;
  }
  if (fpsEl) fpsEl.textContent = fps ? fps.toFixed(1) : "--";
  if (szEl) {
    const avg = stats.framesReceived > 0
      ? Math.round(stats.totalFrameSize / stats.framesReceived)
      : 0;
    szEl.textContent = `${avg.toLocaleString()} bytes`;
  }
  if (kfEl) kfEl.textContent = stats.keyFrames.toLocaleString();
}

function cleanupVideo() {
    debugLog("connection", "🧹 Cleaning up video…");
    videoStreamingActive = false;
    if (videoWs) {
        videoWs.close();
        videoWs = null;
    }
    if (decoder) {
        decoder.close();
        decoder = null;
    }
    
    // Reset stats
    frameCount = 0;
    fps = 0;
    stats = {
        framesReceived: 0,
        framesDecoded: 0,
        decodeErrors: 0,
        keyFrames: 0,
        totalFrameSize: 0,
        lastStatsUpdate: 0
    };
    
    updateVideoStatus("Video: Disconnected", "red");
    if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Disconnected", "disconnected");
}

function updateVideoStatus(msg, color) {
    const elm = document.getElementById("video-status");
    if (elm) {
        elm.textContent = msg;
        elm.style.color = color || "black";
    } else {
        debugLog("connection", `📺 ${msg}`);
    }
}

window.initVideo = initVideo;
window.cleanupVideo = cleanupVideo;

debugLog("connection", "🔧 video.js fully refactored and ready!");
