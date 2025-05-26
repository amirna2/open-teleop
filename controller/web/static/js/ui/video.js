// video.js – WebCodecs H.264 Player (Complete)

// Keep your original debug toggle
const VIDEO_DEBUG = true;
function debugLog(...args) {
  if (VIDEO_DEBUG) console.log(...args);
}

debugLog("video.js loaded");

let videoWs = null;
let decoder = null;
let canvas = null;
let ctx = null;
let videoStreamingActive = false;

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

let spsData = null;
let ppsData = null;
let decoderConfigured = false;

function isKeyFrame(uint8Array) {
  // Look for IDR (0x65) or SPS (0x67) 
  for (let i = 0; i < uint8Array.length - 4; i++) {
    if (uint8Array[i] === 0x00 && uint8Array[i+1] === 0x00 && 
        uint8Array[i+2] === 0x00 && uint8Array[i+3] === 0x01) {
      const nalType = uint8Array[i+4] & 0x1F;
      if (nalType === 5 || nalType === 7) { // IDR or SPS
        return true;
      }
    }
  }
  return false;
}

// ——————————————————————————
// 1️⃣ initVideo (called by main.js)
// ——————————————————————————
function initVideo() {
  debugLog("Initializing Video with WebCodecs…");

  videoStreamingActive = true;
  const videoElement = document.getElementById("video-player");
  if (!videoElement) {
    console.error('❌ <video id="video-player"> not found!');
    return;
  }

  // Check WebCodecs support
  if (!window.VideoDecoder) {
    console.error('❌ WebCodecs not supported in this browser');
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

  // Initialize WebCodecs decoder
  decoder = new VideoDecoder({
    output: handleDecodedFrame,
    error: (error) => {
      console.error('❌ WebCodecs decoder error:', error);
      stats.decodeErrors++;
    }
  });

  // Don't configure yet - wait for SPS/PPS
  debugLog("✅ WebCodecs decoder created, waiting for SPS/PPS");

  connectVideoWebSocket();
}

function handleDecodedFrame(frame) {
  // Draw frame to canvas
  if (canvas && ctx) {
    canvas.width = frame.displayWidth;
    canvas.height = frame.displayHeight;
    ctx.drawImage(frame, 0, 0);
    stats.framesDecoded++;
  }
  frame.close();
}

// ——————————————————————————
// 2️⃣ WebSocket Setup & Handlers
// ——————————————————————————
function connectVideoWebSocket() {
  const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${wsProtocol}//${window.location.host}/ws/video`;
  debugLog(`🔗 Connecting to Video WebSocket: ${wsUrl}`);

  if (videoWs) videoWs.close();
  videoWs = new WebSocket(wsUrl);
  videoWs.binaryType = "arraybuffer";

  videoWs.onopen = () => {
    debugLog("✅ Video WebSocket connected");
    frameCount = 0;
    lastFrameTime = 0;
    fps = 0;
    updateVideoStatus("Video: Connected", "green");
    if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Connected", "connected");
  };

  videoWs.onmessage = (evt) => handleVideoMessage(evt.data);

  videoWs.onclose = (e) => {
    debugLog(`❌ Video WebSocket disconnected: ${e.code}`);
    updateVideoStatus(`Video: Disconnected (${e.code})`, "red");
    if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Disconnected", "disconnected");
    videoWs = null;
    if (videoStreamingActive) setTimeout(connectVideoWebSocket, 5000);
  };

  videoWs.onerror = (err) => {
    console.error("❌ Video WebSocket error:", err);
    updateVideoStatus("Video: Error", "red");
    if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Error", "disconnected");
  };
}


function extractParameterSetsAVC(uint8Array) {
    // AVC format: [4-byte length][NAL unit][4-byte length][NAL unit]...
    let offset = 0;
    
    while (offset + 4 < uint8Array.length) {
      // Read 4-byte length (big endian)
      const nalLength = (uint8Array[offset] << 24) | 
                       (uint8Array[offset + 1] << 16) | 
                       (uint8Array[offset + 2] << 8) | 
                       uint8Array[offset + 3];
      
      if (nalLength <= 0 || offset + 4 + nalLength > uint8Array.length) {
        break; // Invalid length
      }
      
      // Get NAL unit type (first byte after length, masked with 0x1F)
      const nalType = uint8Array[offset + 4] & 0x1F;
      
      if (nalType === 7 && !spsData) { // SPS
        // Extract SPS including length prefix for AVC format
        spsData = uint8Array.slice(offset, offset + 4 + nalLength);
        debugLog("📋 SPS extracted:", spsData.length, "bytes (AVC format)");
      }
      
      if (nalType === 8 && !ppsData) { // PPS
        // Extract PPS including length prefix for AVC format
        ppsData = uint8Array.slice(offset, offset + 4 + nalLength);
        debugLog("📋 PPS extracted:", ppsData.length, "bytes (AVC format)");
      }
      
      // Move to next NAL unit
      offset += 4 + nalLength;
    }
  }
  
  // Also update createAVCDecoderConfig to handle AVC format SPS/PPS
  function createAVCDecoderConfigFromAVC(sps, pps) {
    // SPS and PPS already have length prefixes, extract the actual data
    const spsData = sps.slice(4); // Remove 4-byte length prefix
    const ppsData = pps.slice(4); // Remove 4-byte length prefix
    
    // Calculate buffer size
    const avccSize = 8 + 2 + spsData.length + 1 + 2 + ppsData.length;
    const avcc = new Uint8Array(avccSize);
    
    let offset = 0;
    avcc[offset++] = 0x01; // version
    avcc[offset++] = spsData[1]; // profile
    avcc[offset++] = spsData[2]; // compatibility
    avcc[offset++] = spsData[3]; // level
    avcc[offset++] = 0xFF; // length size minus 1 (4 bytes)
    avcc[offset++] = 0xE1; // number of SPS (1)
    
    // SPS length (big endian)
    avcc[offset++] = (spsData.length >> 8) & 0xFF;
    avcc[offset++] = spsData.length & 0xFF;
    
    // SPS data
    avcc.set(spsData, offset);
    offset += spsData.length;
    
    // Number of PPS
    avcc[offset++] = 0x01; // number of PPS (1)
    
    // PPS length (big endian)
    avcc[offset++] = (ppsData.length >> 8) & 0xFF;
    avcc[offset++] = ppsData.length & 0xFF;
    
    // PPS data
    avcc.set(ppsData, offset);
    
    debugLog("📋 AVCC created from AVC format:", avcc.length, "bytes");
    return avcc;
  }

// ——————————————————————————
// 3️⃣ Message Processing → WebCodecs
// ——————————————————————————
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
    
    if (frameCount < 5) {
      console.log("Chunk size:", uint8Array.length, "bytes");
      console.log("First 32 bytes:", Array.from(uint8Array).slice(0, 32));
    }
  
    // Extract SPS/PPS from AVC format
    extractParameterSetsAVC(uint8Array);
  
    // Check if keyframe (look for IDR NAL type 5 in AVC format)
    const isKeyFrameDetected = isKeyFrameAVC(uint8Array);
    if (isKeyFrameDetected) {
      stats.keyFrames++;
      debugLog(`🔑 Key frame detected! Frame ${frameCount}`);
    }
  
    // Configure decoder once we have SPS/PPS
    if (!decoderConfigured && spsData && ppsData && decoder) {
      try {
        // Create description from AVC format SPS/PPS
        const description = createAVCDecoderConfigFromAVC(spsData, ppsData);
        
        decoder.configure({
          codec: 'avc1.42E01E',
          description: description,
          hardwareAcceleration: 'prefer-software',
          optimizeForLatency: true
        });
        
        decoderConfigured = true;
        debugLog("✅ WebCodecs decoder configured with AVC SPS/PPS");
      } catch (error) {
        console.error('❌ Failed to configure decoder with description:', error);
        return;
      }
    }
  
    // Feed to WebCodecs decoder only after configured
    if (decoder && decoderConfigured && decoder.state === 'configured') {
      try {
        const chunk = new EncodedVideoChunk({
          type: isKeyFrameDetected ? 'key' : 'delta',
          timestamp: now * 1000,
          data: uint8Array
        });
        
        decoder.decode(chunk);
      } catch (e) {
        console.error('❌ WebCodecs decode error:', e);
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

  function isKeyFrameAVC(uint8Array) {
    // Check AVC format for IDR NAL type 5
    let offset = 0;
    
    while (offset + 4 < uint8Array.length) {
      const nalLength = (uint8Array[offset] << 24) | 
                       (uint8Array[offset + 1] << 16) | 
                       (uint8Array[offset + 2] << 8) | 
                       uint8Array[offset + 3];
      
      if (nalLength <= 0 || offset + 4 + nalLength > uint8Array.length) {
        break;
      }
      
      const nalType = uint8Array[offset + 4] & 0x1F;
      if (nalType === 5 || nalType === 7) { // IDR or SPS
        return true;
      }
      
      offset += 4 + nalLength;
    }
    return false;
  }

function extractParameterSets(uint8Array) {
  // Look for SPS (NAL type 7) and PPS (NAL type 8)
  for (let i = 0; i < uint8Array.length - 4; i++) {
    if (uint8Array[i] === 0x00 && uint8Array[i+1] === 0x00 && 
        uint8Array[i+2] === 0x00 && uint8Array[i+3] === 0x01) {
      
      const nalType = uint8Array[i+4] & 0x1F;
      
      if (nalType === 7 && !spsData) { // SPS
        // Find next start code or end of buffer
        let end = uint8Array.length;
        for (let j = i + 4; j < uint8Array.length - 3; j++) {
          if (uint8Array[j] === 0x00 && uint8Array[j+1] === 0x00 && 
              uint8Array[j+2] === 0x00 && uint8Array[j+3] === 0x01) {
            end = j;
            break;
          }
        }
        spsData = uint8Array.slice(i, end);
        debugLog("📋 SPS extracted:", spsData.length, "bytes");
      }
      
      if (nalType === 8 && !ppsData) { // PPS
        let end = uint8Array.length;
        for (let j = i + 4; j < uint8Array.length - 3; j++) {
          if (uint8Array[j] === 0x00 && uint8Array[j+1] === 0x00 && 
              uint8Array[j+2] === 0x00 && uint8Array[j+3] === 0x01) {
            end = j;
            break;
          }
        }
        ppsData = uint8Array.slice(i, end);
        debugLog("📋 PPS extracted:", ppsData.length, "bytes");
      }
    }
  }
}

function createAVCDecoderConfig(sps, pps) {
  // Create AVCC (AVC Configuration Record) format
  const spsWithoutStartCode = sps.slice(4); // Remove 00 00 00 01
  const ppsWithoutStartCode = pps.slice(4); // Remove 00 00 00 01
  
  // Calculate correct buffer size: 8 bytes header + 2 bytes SPS length + SPS + 1 byte + 2 bytes PPS length + PPS
  const avccSize = 8 + 2 + spsWithoutStartCode.length + 1 + 2 + ppsWithoutStartCode.length;
  const avcc = new Uint8Array(avccSize);
  
  let offset = 0;
  avcc[offset++] = 0x01; // version
  avcc[offset++] = spsWithoutStartCode[1]; // profile
  avcc[offset++] = spsWithoutStartCode[2]; // compatibility
  avcc[offset++] = spsWithoutStartCode[3]; // level
  avcc[offset++] = 0xFF; // length size minus 1 (4 bytes)
  avcc[offset++] = 0xE1; // number of SPS (1)
  
  // SPS length (big endian)
  avcc[offset++] = (spsWithoutStartCode.length >> 8) & 0xFF;
  avcc[offset++] = spsWithoutStartCode.length & 0xFF;
  
  // SPS data
  avcc.set(spsWithoutStartCode, offset);
  offset += spsWithoutStartCode.length;
  
  // Number of PPS
  avcc[offset++] = 0x01; // number of PPS (1)
  
  // PPS length (big endian)
  avcc[offset++] = (ppsWithoutStartCode.length >> 8) & 0xFF;
  avcc[offset++] = ppsWithoutStartCode.length & 0xFF;
  
  // PPS data
  avcc.set(ppsWithoutStartCode, offset);
  
  debugLog("📋 AVCC created:", avcc.length, "bytes");
  return avcc;
}

// ——————————————————————————
// 4️⃣ UI and Stats Helpers
// ——————————————————————————
function updateVideoStatus(msg, color) {
  const elm = document.getElementById("video-status");
  if (elm) {
    elm.textContent = msg;
    elm.style.color = color || "black";
  } else {
    debugLog(`📺 ${msg}`);
  }
}

function initializeVideoInfoDisplay() {
  const res = document.getElementById("video-resolution");
  const codec = document.getElementById("video-codec");
  if (res) res.textContent = "Resolution: Auto";
  if (codec) codec.textContent = "Codec: H.264 (WebCodecs)";
}

function updateVideoStats() {
  const now = Date.now();
  if (now - stats.lastStatsUpdate < 1000) return;
  stats.lastStatsUpdate = now;

  const fEl = document.getElementById("stats-frames");
  const fpsEl = document.getElementById("stats-fps");
  const szEl = document.getElementById("stats-frame-size");

  if (fEl) fEl.textContent = stats.framesReceived.toLocaleString();
  if (fpsEl) fpsEl.textContent = fps ? fps.toFixed(1) : "--";
  if (szEl) {
    const avg =
      stats.framesReceived > 0
        ? Math.round(stats.totalFrameSize / stats.framesReceived)
        : 0;
    szEl.textContent = `${avg.toLocaleString()} bytes`;
  }
}

function cleanupVideo() {
  debugLog("🧹 Cleaning up video…");
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

// ——————————————————————————
// 5️⃣ Expose for main.js
// ——————————————————————————
window.initVideo = initVideo;
window.cleanupVideo = cleanupVideo;

debugLog("🔧 video.js loaded with WebCodecs H.264 decoder!");