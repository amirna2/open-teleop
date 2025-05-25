// UI Logic for the Configuration Tab

// Debug configuration
const CONFIG_DEBUG = false;

function configDebugLog(...args) {
    if (CONFIG_DEBUG) {
        console.log(...args);
    }
}

configDebugLog("config.js loaded");

function initConfig() {
    configDebugLog("Initializing Config Tab...");

    const loadBtn = document.getElementById('load-config-btn');
    const saveBtn = document.getElementById('save-config-btn');
    const configTextArea = document.getElementById('config-yaml');
    const configStatus = document.getElementById('config-status');

    if (!loadBtn || !saveBtn || !configTextArea || !configStatus) {
        console.error("Configuration UI elements not found!");
        return;
    }

    // Add event listeners
    loadBtn.addEventListener('click', loadConfigIntoTextArea);
    saveBtn.addEventListener('click', saveConfigFromTextArea);

     // Initial state for status
     configStatus.textContent = 'Ready. Start typing or load config.';
     configStatus.style.color = 'inherit';

    // Add live validation on input
    configTextArea.addEventListener('input', (event) => {
        const yamlContent = event.target.value;
        if (!yamlContent.trim()) {
            // Handle empty input: clear status or set a specific message
            configStatus.textContent = 'Ready. Start typing or load config.';
            configStatus.style.color = 'inherit';
            return;
        }
        try {
            jsyaml.load(yamlContent); // Attempt to parse
            configStatus.textContent = 'YAML syntax is valid.';
            configStatus.style.color = 'green';
        } catch (e) {
            // Display simplified error message
            configStatus.textContent = `YAML Syntax Error: ${e.reason || e.message}`;
            if (e.mark && e.mark.line) {
                 // Add line number if available
                configStatus.textContent += ` at line ${e.mark.line + 1}`;
            }
            configStatus.style.color = 'red';
        }
    });
}

async function loadConfigIntoTextArea() {
    const configTextArea = document.getElementById('config-yaml');
    const configStatus = document.getElementById('config-status');

    if (!configTextArea || !configStatus) return;

    configDebugLog("Load Config button clicked or tab activated");
    configStatus.textContent = 'Loading configuration...';
    configStatus.style.color = 'orange';
    configTextArea.value = ''; // Clear previous content

    try {
        if (typeof getTeleopConfig !== 'function') {
            throw new Error("getTeleopConfig function not found in api.js");
        }
        const yamlData = await getTeleopConfig();
        configTextArea.value = yamlData;
        configStatus.textContent = 'Configuration loaded successfully.';
        configStatus.style.color = 'green';
        configDebugLog("Configuration loaded into text area.");
    } catch (error) {
        console.error("Failed to load configuration:", error);
        configStatus.textContent = `Error loading configuration: ${error.message}`;
        configStatus.style.color = 'red';
    }
}

async function saveConfigFromTextArea() {
    const configTextArea = document.getElementById('config-yaml');
    const configStatus = document.getElementById('config-status');

    if (!configTextArea || !configStatus) return;

    configDebugLog("Save Config button clicked");
    const yamlData = configTextArea.value;

    if (!yamlData.trim()) {
        configStatus.textContent = 'Error: Configuration cannot be empty.';
        configStatus.style.color = 'red';
        return;
    }

    configStatus.textContent = 'Saving configuration...';
    configStatus.style.color = 'orange';

    try {
        if (typeof updateTeleopConfig !== 'function') {
            throw new Error("updateTeleopConfig function not found in api.js");
        }
        const successMessage = await updateTeleopConfig(yamlData);
        configStatus.textContent = `Configuration saved successfully: ${successMessage}`;
        configStatus.style.color = 'green';
        configDebugLog("Configuration saved.");
    } catch (error) {
        console.error("Failed to save configuration:", error);
        configStatus.textContent = `Error saving configuration: ${error.message}`;
        configStatus.style.color = 'red';
    }
}

// Will interact with functions defined in api.js 