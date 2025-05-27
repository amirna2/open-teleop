// video.js – WebCodecs H.264 Player (Refactored)

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
let lastSPS = null;
let lastPPS = null;
let decoderConfigured = false;

function initVideo() {
    debugLog("Initializing Video with WebCodecs…");

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
    debugLog(`🔗 Connecting to Video WebSocket: ${wsUrl}`);

    if (videoWs) videoWs.close();
    videoWs = new WebSocket(wsUrl);
    videoWs.binaryType = "arraybuffer";

    videoWs.onopen = () => {
        debugLog("✅ Video WebSocket connected");
    };

    videoWs.onmessage = (evt) => handleVideoMessage(evt.data);

    videoWs.onclose = () => {
        debugLog("❌ Video WebSocket disconnected");
        updateVideoStatus("Video: Disconnected", "red");
        if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Disconnected", "disconnected");
        if (videoStreamingActive) setTimeout(connectVideoWebSocket, 5000);
    };

    videoWs.onerror = (err) => {
        console.error("❌ WebSocket error:", err);
        updateVideoStatus("Video: Error", "red");
        if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Error", "disconnected");
    };
}

function handleVideoMessage(data) {
    const uint8Array = new Uint8Array(data);
    const { sps, pps, isKeyFrame, complete } = extractSPSPPSandKeyFrame(uint8Array);

    if (!complete) {
        console.warn('⚠️ Incomplete frame received, skipping.');
        return;
    }

    if (sps && pps) {
        if (!arraysEqual(sps, lastSPS) || !arraysEqual(pps, lastPPS)) {
            lastSPS = sps;
            lastPPS = pps;
            debugLog("📋 New SPS/PPS detected, configuring decoder…");
            configureDecoder(sps, pps);
            decoderConfigured = true;
        } else if (!decoderConfigured) {
            debugLog("📋 First SPS/PPS detected, configuring decoder…");
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
            console.error('❌ WebCodecs decode error:', e);
        }
    }
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

    while (offset + 4 < buffer.length) {
        const nalLength = (buffer[offset] << 24) | (buffer[offset+1] << 16) |
                          (buffer[offset+2] << 8) | buffer[offset+3];
        if (nalLength <= 0 || offset + 4 + nalLength > buffer.length) {
            complete = false; // Frame incomplete
            break;
        }

        const nalType = buffer[offset + 4] & 0x1F;
        if (nalType === 7) sps = buffer.slice(offset, offset + 4 + nalLength);
        if (nalType === 8) pps = buffer.slice(offset, offset + 4 + nalLength);
        if (nalType === 5 || nalType === 7) isKeyFrame = true;

        offset += 4 + nalLength;
    }

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
            frame.close();
        },
        error: e => console.error('❌ WebCodecs error:', e)
    });
    decoder.configure({
        codec: 'avc1.42E01E',
        description: description,
        hardwareAcceleration: 'prefer-software',
        optimizeForLatency: true
    });
    debugLog("✅ Decoder configured");
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
    updateVideoStatus("Video: Disconnected", "red");
    if (window.updateVideoStatusHeader) window.updateVideoStatusHeader("Disconnected", "disconnected");
}

function updateVideoStatus(msg, color) {
    const elm = document.getElementById("video-status");
    if (elm) {
        elm.textContent = msg;
        elm.style.color = color || "black";
    } else {
        debugLog(`📺 ${msg}`);
    }
}

window.initVideo = initVideo;
window.cleanupVideo = cleanupVideo;

debugLog("🔧 video.js fully refactored and ready!");
