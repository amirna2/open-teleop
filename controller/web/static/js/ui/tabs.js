// Logic for handling tab switching behavior

console.log("tabs.js loaded");

// Functions to initialize tabs, handle clicks, show/hide panels 

function initTabs() {
    // Optional: Add any specific initialization for tabs here if needed.
    // For example, ensuring the default tab is correctly displayed.
    console.log("Initializing Tabs...");
    // Activate the default tab (assuming it's 'teleop')
    const defaultTab = document.querySelector('.tab-link[onclick*="teleop"]');
    if (defaultTab) {
        // Find the corresponding content panel
        const defaultContent = document.getElementById('teleop');
        if (defaultContent) {
             // Ensure only the default tab content is visible initially
             const tabContents = document.querySelectorAll('.tab-content');
             tabContents.forEach(content => content.style.display = 'none');
             defaultContent.style.display = 'block';

             // Ensure only the default tab link is marked active
             const tabLinks = document.querySelectorAll('.tab-link');
             tabLinks.forEach(link => link.classList.remove('active'));
             defaultTab.classList.add('active');
        }
    }
}

function openTab(event, tabName) {
    console.log(`Switching to tab: ${tabName}`);
    
    // Get the currently active tab before switching
    const currentActiveTab = document.querySelector('.tab-content[style*="block"]');
    const currentTabName = currentActiveTab ? currentActiveTab.id : null;
    
    // Handle cleanup when leaving teleop tab
    if (currentTabName === 'teleop' && tabName !== 'teleop') {
        console.log('🧹 Leaving teleop tab - cleaning up video resources');
        if (typeof window.cleanupVideo === 'function') {
            window.cleanupVideo();
        } else {
            console.warn('cleanupVideo function not available');
        }
    }
    
    // Get all elements with class="tab-content" and hide them
    const tabContents = document.querySelectorAll('.tab-content');
    tabContents.forEach(tabContent => {
        tabContent.style.display = 'none';
    });

    // Get all elements with class="tab-link" and remove the class "active"
    const tabLinks = document.querySelectorAll('.tab-link');
    tabLinks.forEach(tabLink => {
        tabLink.classList.remove('active');
    });

    // Show the current tab, and add an "active" class to the button that opened the tab
    const currentTabContent = document.getElementById(tabName);
    if (currentTabContent) {
        currentTabContent.style.display = 'block';
    } else {
        console.error(`Tab content not found for id: ${tabName}`);
    }

    if (event && event.currentTarget) {
        event.currentTarget.classList.add('active');
    } else {
        // Fallback if event is not provided (e.g., programmatic activation)
        const currentTabLink = document.querySelector(`.tab-link[onclick*="${tabName}"]`);
        if (currentTabLink) {
            currentTabLink.classList.add('active');
        }
    }

     // Handle tab-specific initialization
     if (tabName === 'config') {
         // Automatically load config when config tab is opened
         if (typeof loadConfigIntoTextArea === 'function') {
             loadConfigIntoTextArea();
         } else {
             console.warn("loadConfigIntoTextArea function not found in config.js - cannot auto-load.");
         }
     } else if (tabName === 'teleop') {
         // Reinitialize video when returning to teleop tab
         console.log('🎬 Entering teleop tab - initializing video resources');
         if (typeof window.initVideo === 'function') {
             // Longer delay to ensure DOM is fully ready and visible
             setTimeout(() => {
                 console.log('🔄 Attempting to reinitialize video...');
                 try {
                     window.initVideo();
                     console.log('✅ Video reinitialization completed');
                 } catch (error) {
                     console.error('❌ Error reinitializing video:', error);
                 }
             }, 250); // Increased delay
         } else {
             console.warn('initVideo function not available');
         }
     }
} 