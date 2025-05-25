// Dashboard UI Logic

// Debug configuration
const DASHBOARD_DEBUG = false;

function dashboardDebugLog(...args) {
    if (DASHBOARD_DEBUG) {
        console.log(...args);
    }
}

dashboardDebugLog("dashboard.js loaded");

// Dashboard state
let configPanelVisible = false;

function initDashboard() {
    dashboardDebugLog('🎛️ Initializing Dashboard...');
    
    // Set up config panel toggle
    const toggleConfigBtn = document.getElementById('toggle-config-btn');
    if (toggleConfigBtn) {
        toggleConfigBtn.addEventListener('click', toggleConfigPanel);
    }
    
    // Initialize system status
    updateSystemStatus('Initializing...', 'initializing');
    
    // Set up status synchronization
    setupStatusSync();
    
    dashboardDebugLog('✅ Dashboard initialized');
}

function toggleConfigPanel() {
    const configPanel = document.getElementById('config-panel');
    const toggleBtn = document.getElementById('toggle-config-btn');
    
    if (configPanel && toggleBtn) {
        configPanelVisible = !configPanelVisible;
        
        if (configPanelVisible) {
            configPanel.classList.remove('hidden');
            toggleBtn.textContent = 'Hide Config';
        } else {
            configPanel.classList.add('hidden');
            toggleBtn.textContent = 'Show Config';
        }
    }
}

function updateSystemStatus(status, type = 'disconnected') {
    const systemStatusEl = document.getElementById('system-status');
    if (systemStatusEl) {
        systemStatusEl.textContent = status;
        
        // Remove existing status classes
        systemStatusEl.classList.remove('status-connected', 'status-disconnected', 'status-warning', 'status-initializing');
        
        // Add appropriate status class
        systemStatusEl.classList.add(`status-${type}`);
    }
}

function updateVideoStatusHeader(status, type = 'disconnected') {
    const videoStatusEl = document.getElementById('video-status-header');
    if (videoStatusEl) {
        videoStatusEl.textContent = status;
        
        // Remove existing status classes
        videoStatusEl.classList.remove('status-connected', 'status-disconnected', 'status-warning', 'status-initializing');
        
        // Add appropriate status class
        videoStatusEl.classList.add(`status-${type}`);
    }
}

function updateControlStatusHeader(status, type = 'disconnected') {
    const controlStatusEl = document.getElementById('control-status-header');
    if (controlStatusEl) {
        controlStatusEl.textContent = status;
        
        // Remove existing status classes
        controlStatusEl.classList.remove('status-connected', 'status-disconnected', 'status-warning', 'status-initializing');
        
        // Add appropriate status class
        controlStatusEl.classList.add(`status-${type}`);
    }
}

function setupStatusSync() {
    // Monitor video status changes and sync to header
    const videoStatusDiv = document.getElementById('video-status');
    if (videoStatusDiv) {
        // Function to sync video status
        const syncVideoStatus = () => {
            const statusText = videoStatusDiv.textContent;
            dashboardDebugLog(`🔄 Syncing video status: "${statusText}"`);
            
            // Check if we have frame statistics indicating active streaming
            const framesElement = document.getElementById('stats-frames');
            const decodedElement = document.getElementById('stats-decoded');
            const hasFrames = framesElement && parseInt(framesElement.textContent.replace(/,/g, '')) > 0;
            const hasDecoded = decodedElement && parseInt(decodedElement.textContent.replace(/,/g, '')) > 0;
            
            // If we have frames being received/decoded but status doesn't show streaming, force update
            if (hasFrames && hasDecoded && !statusText.includes('Streaming')) {
                dashboardDebugLog('🎬 Detected active streaming from statistics, updating status');
                updateVideoStatusHeader('Streaming', 'connected');
                return;
            }
            
            // Parse status and update header with priority order
            if (statusText.includes('Streaming')) {
                updateVideoStatusHeader('Streaming', 'connected');
            } else if (statusText.includes('Connected')) {
                updateVideoStatusHeader('Connected', 'connected');
            } else if (statusText.includes('Connecting')) {
                updateVideoStatusHeader('Connecting', 'warning');
            } else if (statusText.includes('Initializing')) {
                updateVideoStatusHeader('Initializing', 'initializing');
            } else if (statusText.includes('Error')) {
                updateVideoStatusHeader('Error', 'disconnected');
            } else {
                updateVideoStatusHeader('Disconnected', 'disconnected');
            }
        };
        
        // Initial sync
        syncVideoStatus();
        
        // Create a MutationObserver to watch for text changes
        const observer = new MutationObserver((mutations) => {
            mutations.forEach((mutation) => {
                if (mutation.type === 'childList' || mutation.type === 'characterData') {
                    // Small delay to ensure text is fully updated
                    setTimeout(syncVideoStatus, 10);
                }
            });
        });
        
        observer.observe(videoStatusDiv, {
            childList: true,
            characterData: true,
            subtree: true
        });
        
        // Also sync periodically as a fallback (every 2 seconds)
        setInterval(syncVideoStatus, 2000);
    }
    
    // Monitor control status changes and sync to header
    const controlStatusDiv = document.getElementById('status');
    if (controlStatusDiv) {
        const observer = new MutationObserver((mutations) => {
            mutations.forEach((mutation) => {
                if (mutation.type === 'childList' || mutation.type === 'characterData') {
                    const statusText = controlStatusDiv.textContent;
                    
                    // Parse status and update header
                    if (statusText.includes('Connected')) {
                        updateControlStatusHeader('Connected', 'connected');
                        updateSystemStatus('Operational', 'connected');
                    } else if (statusText.includes('Connecting')) {
                        updateControlStatusHeader('Connecting', 'warning');
                        updateSystemStatus('Connecting', 'warning');
                    } else {
                        updateControlStatusHeader('Disconnected', 'disconnected');
                        updateSystemStatus('Disconnected', 'disconnected');
                    }
                }
            });
        });
        
        observer.observe(controlStatusDiv, {
            childList: true,
            characterData: true,
            subtree: true
        });
    }
}

// Export functions for main.js to call
window.initDashboard = initDashboard;
window.updateSystemStatus = updateSystemStatus;
window.updateVideoStatusHeader = updateVideoStatusHeader;
window.updateControlStatusHeader = updateControlStatusHeader;

dashboardDebugLog('🎛️ Dashboard functions exported to window'); 