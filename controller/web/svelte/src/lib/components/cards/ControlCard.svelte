<script lang="ts">
	import { onMount, onDestroy } from 'svelte';
	import { VirtualJoystick } from '$lib/utils/joystick';
	import { controlActions, controlState, controlStats } from '$lib/stores/control';
	import BaseCard from './BaseCard.svelte';

	// Props
	export let title = 'Robot Control';
	export let controlId = 'primary';
	export let showStats = false;
	export let autoConnect = true;
	export let resizable = false;
	export let movable = false;
	export let closable = false;

	// Elements
	let joystickBase: HTMLElement;
	let joystickHandle: HTMLElement;
	let joystick: VirtualJoystick | null = null;

	// Local state
	let isDragging = false;
	let currentLinear = 0;
	let currentAngular = 0;

	// Subscribe to control state
	$: connectionStatus = $controlState.isConnected ? 'connected' : 'disconnected';
	$: stats = $controlStats;

	onMount(() => {
		// Initialize joystick
		if (joystickBase && joystickHandle) {
			joystick = new VirtualJoystick(joystickBase, joystickHandle, {
				onMove: (linear: number, angular: number) => {
					currentLinear = linear;
					currentAngular = angular;
					controlActions.sendCommand(linear, angular);
				},
				onStart: () => {
					isDragging = true;
				},
				onEnd: () => {
					isDragging = false;
					currentLinear = 0;
					currentAngular = 0;
				}
			});
		}

		// Auto-connect if enabled
		if (autoConnect) {
			controlActions.initialize();
		}
	});

	onDestroy(() => {
		if (joystick) {
			joystick.destroy();
		}
	});


	function reconnect() {
		controlActions.reconnect();
	}

	function resetJoystick() {
		if (joystick) {
			joystick.reset();
		}
	}

	// Format velocity values for display
	function formatVelocity(value: number): string {
		return value.toFixed(2);
	}

</script>

<BaseCard 
	{title}
	cardId={controlId}
	statusColor={connectionStatus}
	{resizable}
	{movable}
	{closable}
	class={isDragging ? 'dragging' : ''}
>
	<svelte:fragment slot="header-controls">
		<button class="reconnect-btn" on:click={reconnect} title="Reconnect">
			🔄
		</button>
	</svelte:fragment>
	
	<div class="card-content">
		<div class="joystick-container">
			<div class="joystick-base" bind:this={joystickBase}>
				<div class="joystick-handle" bind:this={joystickHandle}></div>
			</div>
		</div>
	</div>
</BaseCard>

<style>
	/* Header control button specific to control card */
	.reconnect-btn {
		background: none;
		border: none;
		font-size: 0.9rem;
		cursor: pointer;
		padding: 0.25rem;
		border-radius: 4px;
		transition: background-color 0.2s;
	}

	.reconnect-btn:hover {
		background-color: #444;
	}

	/* Control-specific content styles */
	.card-content {
		height: 100%;
		padding: 1rem;
		position: relative;
		display: flex;
		flex-direction: column;
		align-items: center;
		justify-content: center;
	}

	.joystick-container {
		display: flex;
		flex-direction: column;
		align-items: center;
		justify-content: center;
		width: 100%;
		height: 100%;
	}

	.joystick-base {
		width: 120px;
		height: 120px;
		background: linear-gradient(135deg, #2a2a2a, #1e1e1e);
		border: 2px solid #444;
		border-radius: 50%;
		position: relative;
		box-shadow: inset 0 2px 8px rgba(0, 0, 0, 0.3);
	}

	.joystick-handle {
		width: 40px;
		height: 40px;
		background: linear-gradient(135deg, #14a085, #0d7377);
		border: 2px solid #0a5d61;
		border-radius: 50%;
		position: absolute;
		top: 50%;
		left: 50%;
		transform: translate(-50%, -50%);
		cursor: grab;
		transition: transform 0.1s ease-out;
		box-shadow: 0 2px 8px rgba(0, 0, 0, 0.4);
	}

	.joystick-handle:active {
		cursor: grabbing;
		box-shadow: 0 4px 12px rgba(20, 160, 133, 0.4);
	}

	/* Unused styles kept for potential future use */
	.velocity-display {
		display: flex;
		gap: 1rem;
		justify-content: center;
	}

	.velocity-item {
		display: flex;
		flex-direction: column;
		align-items: center;
		gap: 0.25rem;
	}

	.velocity-item .label {
		font-size: 0.8rem;
		color: #cccccc;
		font-weight: 500;
	}

	.velocity-item .value {
		font-size: 1rem;
		color: #ffffff;
		font-family: 'Courier New', monospace;
		font-weight: bold;
		min-width: 50px;
		text-align: center;
	}

	.connection-status {
		text-align: center;
	}

	.status-text {
		font-size: 0.9rem;
		font-weight: 500;
	}

	/* Responsive adjustments */
	@media (max-width: 768px) {
		.joystick-base {
			width: 100px;
			height: 100px;
		}

		.joystick-handle {
			width: 32px;
			height: 32px;
		}

		.velocity-display {
			flex-direction: column;
			gap: 0.5rem;
		}
	}
</style>