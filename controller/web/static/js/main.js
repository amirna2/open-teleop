// Main orchestrating script - UPDATED VERSION 2025-01-25

console.log("main.js loaded - VERSION 2025-01-25");

document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM fully loaded and parsed');

    // Initialize Dashboard
    try {
        if (typeof initDashboard === 'function') {
            initDashboard();
            console.log('✅ initDashboard completed successfully');
        } else {
            console.error("initDashboard function not found in js/ui/dashboard.js");
        }
    } catch (error) {
        console.error('❌ Error in initDashboard:', error);
    }

    // Initialize UI Components
    try {
        if (typeof initTeleop === 'function') {
            initTeleop();
            console.log('✅ initTeleop completed successfully');
        } else {
            console.error("initTeleop function not found in js/ui/teleop.js");
        }
    } catch (error) {
        console.error('❌ Error in initTeleop:', error);
    }

    try {
        if (typeof initConfig === 'function') {
            initConfig();
            console.log('✅ initConfig completed successfully');
        } else {
            console.error("initConfig function not found in js/ui/config.js");
        }
    } catch (error) {
        console.error('❌ Error in initConfig:', error);
    }

    // Initialize Video (always initialize in dashboard layout)
    console.log('🎬 Initializing video for dashboard...');
    if (typeof initVideo === 'function') {
        console.log('✅ Found initVideo function, calling it...');
        try {
            initVideo();
        } catch (error) {
            console.error('❌ Error calling initVideo:', error);
        }
    } else if (typeof window.initVideo === 'function') {
        console.log('✅ Found window.initVideo function, calling it...');
        try {
            window.initVideo();
        } catch (error) {
            console.error('❌ Error calling window.initVideo:', error);
        }
    } else {
        console.error("❌ initVideo function not found in js/ui/video.js");
    }

    console.log("Dashboard initialization complete.");
});

// Note: Joystick functionality is now handled in js/ui/teleop.js 