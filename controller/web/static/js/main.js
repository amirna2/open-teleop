// Main orchestrating script - UPDATED VERSION 2025-01-25

console.log("main.js loaded - VERSION 2025-01-25");

document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM fully loaded and parsed');

    // Initialize UI Components
    try {
        if (typeof initTabs === 'function') {
            initTabs();
            console.log('✅ initTabs completed successfully');
        } else {
            console.error("initTabs function not found in js/ui/tabs.js");
        }
    } catch (error) {
        console.error('❌ Error in initTabs:', error);
    }

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

    console.log('🔍 About to check for initVideo function...');
    console.log('🎬 Checking for initVideo function...');
    console.log('initVideo type:', typeof initVideo);
    console.log('window.initVideo type:', typeof window.initVideo);
    console.log('globalThis.initVideo type:', typeof globalThis.initVideo);
    console.log('All window properties with "init":', Object.keys(window).filter(key => key.includes('init')));
    
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
        console.log('Available functions:', Object.keys(window).filter(key => key.includes('init')));
        console.log('All window properties:', Object.keys(window).slice(0, 20));
    }

    console.log("Initialization complete.");
});

// Note: Joystick functionality is now handled in js/ui/teleop.js 