import { writable, derived } from 'svelte/store';
import { browser } from '$app/environment';

// Config state interface
export interface ConfigState {
	yamlContent: string;
	isLoading: boolean;
	isValid: boolean;
	validationError: string | null;
	lastSaved: Date | null;
	hasUnsavedChanges: boolean;
}

// Initial state
const initialState: ConfigState = {
	yamlContent: '',
	isLoading: false,
	isValid: true,
	validationError: null,
	lastSaved: null,
	hasUnsavedChanges: false
};

// Main config store
export const configState = writable<ConfigState>(initialState);

// Status message derived store
export const configStatus = derived(configState, ($config) => {
	if ($config.isLoading) {
		return { message: 'Loading configuration...', type: 'loading' as const };
	}
	
	if ($config.validationError) {
		return { 
			message: `YAML Syntax Error: ${$config.validationError}`, 
			type: 'error' as const 
		};
	}
	
	if (!$config.yamlContent.trim()) {
		return { 
			message: 'Ready. Start typing or load config.', 
			type: 'info' as const 
		};
	}
	
	if ($config.hasUnsavedChanges) {
		return { 
			message: 'YAML syntax is valid. Unsaved changes detected.', 
			type: 'warning' as const 
		};
	}
	
	return { 
		message: 'YAML syntax is valid.', 
		type: 'success' as const 
	};
});

// Config actions
export const configActions = {
	// Load configuration from backend
	async loadConfig(): Promise<void> {
		if (!browser) return;
		
		configState.update(state => ({ ...state, isLoading: true }));
		
		try {
			const response = await fetch('/api/v1/config/teleop', {
				method: 'GET',
				headers: {
					'Accept': 'application/x-yaml, application/yaml, text/yaml'
				}
			});
			
			if (!response.ok) {
				const errorData = await response.json().catch(() => 
					({ error: 'Failed to parse error response' })
				);
				throw new Error(`HTTP error ${response.status}: ${errorData.error || 'Unknown error'}`);
			}
			
			if (response.status === 204 || response.headers.get('content-length') === '0') {
				throw new Error('Received empty configuration');
			}
			
			const yamlContent = await response.text();
			
			configState.update(state => ({
				...state,
				yamlContent,
				isLoading: false,
				isValid: true,
				validationError: null,
				hasUnsavedChanges: false,
				lastSaved: new Date()
			}));
			
			// Validate the loaded content
			configActions.validateYaml(yamlContent);
			
		} catch (error) {
			console.error('Error loading config:', error);
			configState.update(state => ({
				...state,
				isLoading: false,
				validationError: error instanceof Error ? error.message : 'Failed to load configuration'
			}));
		}
	},
	
	// Save configuration to backend
	async saveConfig(): Promise<void> {
		if (!browser) return;
		
		const currentState = configState;
		let yamlContent = '';
		
		configState.subscribe(state => {
			yamlContent = state.yamlContent;
		})();
		
		if (!yamlContent.trim()) {
			configState.update(state => ({
				...state,
				validationError: 'Configuration cannot be empty'
			}));
			return;
		}
		
		configState.update(state => ({ ...state, isLoading: true }));
		
		try {
			const response = await fetch('/api/v1/config/teleop', {
				method: 'PUT',
				headers: {
					'Content-Type': 'application/x-yaml'
				},
				body: yamlContent
			});
			
			const responseData = await response.json();
			
			if (!response.ok) {
				throw new Error(`HTTP error ${response.status}: ${responseData.error || 'Unknown error'}`);
			}
			
			configState.update(state => ({
				...state,
				isLoading: false,
				hasUnsavedChanges: false,
				lastSaved: new Date()
			}));
			
			console.log('Config saved successfully:', responseData.message);
			
		} catch (error) {
			console.error('Error saving config:', error);
			configState.update(state => ({
				...state,
				isLoading: false,
				validationError: error instanceof Error ? error.message : 'Failed to save configuration'
			}));
		}
	},
	
	// Update YAML content with validation
	updateContent(content: string): void {
		configState.update(state => ({
			...state,
			yamlContent: content,
			hasUnsavedChanges: content !== (state.lastSaved ? content : ''),
			validationError: null,
			isValid: true
		}));
		
		// Validate the new content
		configActions.validateYaml(content);
	},
	
	// Validate YAML syntax
	validateYaml(content: string): void {
		if (!content.trim()) {
			configState.update(state => ({
				...state,
				isValid: true,
				validationError: null
			}));
			return;
		}
		
		try {
			// Use js-yaml for validation (assuming it's available globally)
			if (typeof window !== 'undefined' && (window as any).jsyaml) {
				(window as any).jsyaml.load(content);
			}
			
			configState.update(state => ({
				...state,
				isValid: true,
				validationError: null
			}));
		} catch (error: any) {
			let errorMessage = error.reason || error.message || 'Invalid YAML syntax';
			
			// Add line number if available
			if (error.mark && error.mark.line !== undefined) {
				errorMessage += ` at line ${error.mark.line + 1}`;
			}
			
			configState.update(state => ({
				...state,
				isValid: false,
				validationError: errorMessage
			}));
		}
	},
	
	// Reset to initial state
	reset(): void {
		configState.set(initialState);
	}
};