<script lang="ts">
	import { onMount } from 'svelte';
	import { configState, configStatus, configActions } from '$lib/stores/config';

	// Auto-load config on mount
	onMount(() => {
		configActions.loadConfig();
	});

	function handleInput(event: Event) {
		const target = event.target as HTMLTextAreaElement;
		configActions.updateContent(target.value);
	}

	function handleLoadConfig() {
		configActions.loadConfig();
	}

	function handleSaveConfig() {
		configActions.saveConfig();
	}

	// Get status color based on type
	function getStatusColor(type: string): string {
		switch (type) {
			case 'success': return '#14a085';
			case 'error': return '#e74c3c';
			case 'warning': return '#f39c12';
			case 'loading': return '#f39c12';
			default: return '#cccccc';
		}
	}
</script>

<div class="config-editor">
	<div class="config-controls">
		<button 
			class="load-btn" 
			on:click={handleLoadConfig} 
			disabled={$configState.isLoading}
		>
			{$configState.isLoading ? 'Loading...' : 'Load Current Config'}
		</button>
		
		<button 
			class="save-btn" 
			on:click={handleSaveConfig} 
			disabled={$configState.isLoading || !$configState.isValid || !$configState.yamlContent.trim()}
		>
			{$configState.isLoading ? 'Saving...' : 'Save Configuration'}
		</button>
	</div>

	<div class="editor-container">
		<textarea
			class="yaml-editor"
			value={$configState.yamlContent}
			on:input={handleInput}
			placeholder="Click 'Load Current Config' or paste YAML configuration here..."
			rows="20"
			cols="80"
			spellcheck="false"
		></textarea>
		
		<div 
			class="config-status"
			style="color: {getStatusColor($configStatus.type)}"
		>
			{$configStatus.message}
		</div>
	</div>
</div>

<style>
	.config-editor {
		display: flex;
		flex-direction: column;
		gap: 1rem;
		max-width: 90ch;
		margin: 0 auto;
		padding: 1rem;
	}

	.config-controls {
		display: flex;
		gap: 1rem;
		justify-content: flex-start;
	}

	.load-btn, .save-btn {
		background-color: #0d7377;
		color: white;
		border: none;
		padding: 0.5rem 1rem;
		border-radius: 4px;
		cursor: pointer;
		font-size: 0.9rem;
		transition: background-color 0.2s;
	}

	.load-btn:hover:not(:disabled), 
	.save-btn:hover:not(:disabled) {
		background-color: #14a085;
	}

	.load-btn:disabled, 
	.save-btn:disabled {
		background-color: #666;
		cursor: not-allowed;
		opacity: 0.6;
	}

	.editor-container {
		display: flex;
		flex-direction: column;
		gap: 0.5rem;
	}

	.yaml-editor {
		font-family: 'Courier New', 'Monaco', 'Menlo', monospace;
		font-size: 0.9rem;
		border: 1px solid #444;
		background-color: #1e1e1e;
		color: #ffffff;
		padding: 1rem;
		border-radius: 4px;
		resize: vertical;
		min-height: 400px;
		line-height: 1.4;
		tab-size: 2;
		white-space: pre;
		overflow-wrap: normal;
		overflow-x: auto;
	}

	.yaml-editor:focus {
		outline: none;
		border-color: #0d7377;
		box-shadow: 0 0 0 2px rgba(13, 115, 119, 0.2);
	}

	.yaml-editor::placeholder {
		color: #666;
		font-style: italic;
	}

	.config-status {
		font-size: 0.9rem;
		min-height: 1.2em;
		word-wrap: break-word;
		overflow-wrap: break-word;
		transition: color 0.3s ease;
	}

	/* Responsive adjustments */
	@media (max-width: 768px) {
		.config-editor {
			max-width: 100%;
			padding: 0.5rem;
		}

		.config-controls {
			flex-direction: column;
			gap: 0.5rem;
		}

		.yaml-editor {
			font-size: 0.8rem;
			padding: 0.75rem;
		}
	}
</style>