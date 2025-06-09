<script lang="ts">
	import { onMount, onDestroy } from 'svelte';
	import { WebCodecsVideoPlayer } from '$lib/video/webcodecs-player';
	import BaseCard from './BaseCard.svelte';

	// Props
	export let title = 'Video Stream';
	export let streamId = 'primary';
	export let showStats = false;
	export let allowOverlayToggle = true;
	export let resizable = false;
	export let movable = false;
	export let closable = false;

	// State
	let canvasElement: HTMLCanvasElement;
	let videoPlayer: WebCodecsVideoPlayer | null = null;
	let connectionStatus: 'connected' | 'disconnected' | 'error' = 'disconnected';
	let statusMessage = 'Disconnected';
	let statusColor = 'red';
	
	// Stats (reactive to player updates)
	let stats = {
		framesReceived: 0,
		framesDecoded: 0,
		decodeErrors: 0,
		keyFrames: 0,
		totalFrameSize: 0,
		lastStatsUpdate: 0
	};
	let fps = 0;
	let frameCount = 0;

	// Stats update interval
	let statsInterval: NodeJS.Timeout;

	onMount(() => {
		if (canvasElement) {
			// Initialize WebCodecs player with exact same functionality as original
			videoPlayer = new WebCodecsVideoPlayer(
				canvasElement,
				handleStatusUpdate,
				handleHeaderStatusUpdate
			);
			
			videoPlayer.initialize();

			// Update stats every second to maintain reactivity
			statsInterval = setInterval(() => {
				if (videoPlayer) {
					stats = videoPlayer.getStats();
					fps = videoPlayer.getFPS();
					frameCount = videoPlayer.getFrameCount();
				}
			}, 1000);
		}
	});

	onDestroy(() => {
		if (videoPlayer) {
			videoPlayer.cleanup();
		}
		if (statsInterval) {
			clearInterval(statsInterval);
		}
	});

	function handleStatusUpdate(message: string, color: string) {
		statusMessage = message;
		statusColor = color;
	}

	function handleHeaderStatusUpdate(status: string, state: 'connected' | 'disconnected') {
		connectionStatus = state;
	}

	function toggleOverlay() {
		showStats = !showStats;
	}

	function swapToMain() {
		// TODO: Implement click-to-swap functionality in Phase 4
		console.log('Swap to main video requested for:', streamId);
	}

	// Calculate derived stats
	$: errorRate = stats.framesReceived > 0 ? 
		(stats.decodeErrors / stats.framesReceived * 100).toFixed(1) : '0.0';
	$: avgFrameSize = stats.framesReceived > 0 ? 
		Math.round(stats.totalFrameSize / stats.framesReceived) : 0;
</script>

<BaseCard 
	{title}
	cardId={streamId}
	statusColor={connectionStatus}
	{resizable}
	{movable}
	{closable}
>
	<svelte:fragment slot="header-controls">
		{#if allowOverlayToggle}
			<button 
				class="overlay-toggle" 
				class:active={showStats}
				on:click={toggleOverlay} 
				title="Toggle stats overlay"
			>
				📊
			</button>
		{/if}
	</svelte:fragment>
	
	<div class="video-container" on:click={swapToMain} on:keydown={(e) => e.key === 'Enter' && swapToMain()} role="button" tabindex="0">
		<canvas bind:this={canvasElement} class="video-canvas"></canvas>
		
		{#if showStats}
			<div class="video-overlay">
				<div class="stats-grid">
					<div class="stat">
						<span class="label">FPS:</span>
						<span class="value">{fps ? fps.toFixed(1) : '--'}</span>
					</div>
					<div class="stat">
						<span class="label">Frames:</span>
						<span class="value">{stats.framesReceived.toLocaleString()}</span>
					</div>
					<div class="stat">
						<span class="label">Decoded:</span>
						<span class="value">{stats.framesDecoded.toLocaleString()}</span>
					</div>
					<div class="stat">
						<span class="label">Errors:</span>
						<span class="value">{stats.decodeErrors.toLocaleString()}</span>
					</div>
					<div class="stat">
						<span class="label">Error Rate:</span>
						<span class="value">{errorRate}%</span>
					</div>
					<div class="stat">
						<span class="label">Key Frames:</span>
						<span class="value">{stats.keyFrames.toLocaleString()}</span>
					</div>
					<div class="stat">
						<span class="label">Avg Size:</span>
						<span class="value">{avgFrameSize.toLocaleString()} B</span>
					</div>
					<div class="stat">
						<span class="label">Status:</span>
						<span class="value" style="color: {statusColor}">{statusMessage}</span>
					</div>
				</div>
			</div>
		{/if}
		
		{#if connectionStatus === 'disconnected'}
			<div class="connection-overlay">
				<div class="connection-message">
					<span class="icon">📺</span>
					<p>Connecting to video stream...</p>
				</div>
			</div>
		{/if}
	</div>
</BaseCard>

<style>
	/* Header control button specific to video card */
	.overlay-toggle {
		background: none;
		border: none;
		font-size: 1rem;
		cursor: pointer;
		padding: 0.25rem;
		border-radius: 4px;
		transition: background-color 0.2s, opacity 0.2s;
		opacity: 0.6;
	}

	.overlay-toggle:hover {
		background-color: #444;
		opacity: 1;
	}

	.overlay-toggle.active {
		background-color: #14a085;
		opacity: 1;
	}

	.overlay-toggle.active:hover {
		background-color: #0d7377;
	}

	/* Video-specific content styles */
	.video-container {
		height: 100%;
		position: relative;
		cursor: pointer;
		overflow: hidden;
	}

	.video-canvas {
		width: 100%;
		height: 100%;
		display: block;
		background-color: #000;
	}

	.video-overlay {
		position: absolute;
		top: 0;
		left: 0;
		right: 0;
		bottom: 0;
		pointer-events: none;
		display: flex;
		align-items: flex-start;
		justify-content: flex-end;
		padding: 0.75rem;
	}

	.stats-grid {
		background: rgba(0, 0, 0, 0.75);
		backdrop-filter: blur(8px);
		border-radius: 6px;
		padding: 0.75rem;
		display: grid;
		grid-template-columns: 1fr 1fr;
		gap: 0.4rem;
		min-width: 220px;
		border: 1px solid rgba(255, 255, 255, 0.1);
		box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
	}

	.stat {
		display: flex;
		justify-content: space-between;
		align-items: center;
		font-size: 0.75rem;
	}

	.stat .label {
		color: #cccccc;
		font-weight: 500;
	}

	.stat .value {
		color: #ffffff;
		font-family: 'Courier New', monospace;
		font-weight: bold;
	}

	.connection-overlay {
		position: absolute;
		top: 0;
		left: 0;
		right: 0;
		bottom: 0;
		background: rgba(0, 0, 0, 0.8);
		display: flex;
		align-items: center;
		justify-content: center;
	}

	.connection-message {
		text-align: center;
		color: #cccccc;
	}

	.connection-message .icon {
		font-size: 3rem;
		display: block;
		margin-bottom: 0.5rem;
	}

	.connection-message p {
		margin: 0;
		font-size: 0.9rem;
	}

	/* Responsive adjustments */
	@media (max-width: 768px) {
		.stats-grid {
			grid-template-columns: 1fr;
			min-width: 180px;
			padding: 0.5rem;
		}

		.stat {
			font-size: 0.65rem;
		}

		.video-overlay {
			padding: 0.5rem;
		}
	}
</style>