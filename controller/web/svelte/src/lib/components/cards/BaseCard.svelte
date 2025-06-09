<script lang="ts">
	// Card configuration
	export let title: string = '';
	export let cardId: string = '';
	export let resizable: boolean = false;
	export let movable: boolean = false;
	export let closable: boolean = false;
	
	// Status and controls
	export let statusColor: 'connected' | 'disconnected' | 'error' | '' = '';
	export let showStatus: boolean = true;
	
	// Events
	import { createEventDispatcher } from 'svelte';
	const dispatch = createEventDispatcher();
	
	// State
	let isDragging = false;
	let isResizing = false;
	let cardElement: HTMLElement;
	
	// Drag state
	let dragStart = { x: 0, y: 0 };
	let cardPosition = { x: 0, y: 0 };
	
	function handleClose() {
		dispatch('close', { cardId });
	}
	
	function handleMove() {
		dispatch('move', { cardId });
	}
	
	function handleResize() {
		dispatch('resize', { cardId });
	}
	
	// Drag functionality
	function startDrag(event: MouseEvent) {
		if (!movable) return;
		
		isDragging = true;
		dragStart.x = event.clientX - cardPosition.x;
		dragStart.y = event.clientY - cardPosition.y;
		
		document.addEventListener('mousemove', onDrag);
		document.addEventListener('mouseup', stopDrag);
		event.preventDefault();
	}
	
	function onDrag(event: MouseEvent) {
		if (!isDragging) return;
		
		cardPosition.x = event.clientX - dragStart.x;
		cardPosition.y = event.clientY - dragStart.y;
		
		// Apply position to element
		if (cardElement) {
			cardElement.style.transform = `translate(${cardPosition.x}px, ${cardPosition.y}px)`;
		}
	}
	
	function stopDrag() {
		isDragging = false;
		document.removeEventListener('mousemove', onDrag);
		document.removeEventListener('mouseup', stopDrag);
	}
</script>

<div 
	class="base-card" 
	class:dragging={isDragging}
	class:resizing={isResizing}
	class:movable={movable}
	data-card-id={cardId}
	bind:this={cardElement}
>
	<div class="card-header" on:mousedown={startDrag} class:draggable={movable}>
		<h3 class="card-title">{title}</h3>
		
		<div class="header-controls">
			<!-- Custom header controls slot -->
			<slot name="header-controls" />
			
			<!-- Status indicator -->
			{#if showStatus}
				<span 
					class="status-indicator" 
					class:connected={statusColor === 'connected'} 
					class:disconnected={statusColor === 'disconnected'}
					class:error={statusColor === 'error'}
				>
					●
				</span>
			{/if}
			
			<!-- System controls -->
			{#if movable}
				<button class="control-btn move-btn" on:click={handleMove} title="Move card">
					⋮⋮
				</button>
			{/if}
			
			{#if resizable}
				<button class="control-btn resize-btn" on:click={handleResize} title="Resize card">
					⤢
				</button>
			{/if}
			
			{#if closable}
				<button class="control-btn close-btn" on:click={handleClose} title="Close card">
					✕
				</button>
			{/if}
		</div>
	</div>
	
	<div class="card-content">
		<slot />
	</div>
	
	<!-- Footer slot for optional card footer -->
	{#if $$slots.footer}
		<div class="card-footer">
			<slot name="footer" />
		</div>
	{/if}
</div>

<style>
	.base-card {
		background-color: rgba(30, 30, 30, 0.7);
		border: 1px solid rgba(68, 68, 68, 0.7);
		border-radius: 8px;
		overflow: hidden;
		display: flex;
		flex-direction: column;
		height: 100%;
		transition: border-color 0.3s ease, box-shadow 0.3s ease;
		backdrop-filter: blur(4px);
	}
	
	.base-card.movable {
		position: relative;
	}

	.base-card:hover {
		border-color: #0d7377;
	}

	.base-card.dragging {
		border-color: #14a085;
		box-shadow: 0 4px 12px rgba(20, 160, 133, 0.3);
	}

	.base-card.resizing {
		border-color: #f39c12;
		box-shadow: 0 4px 12px rgba(243, 156, 18, 0.3);
	}

	.card-header {
		display: flex;
		justify-content: space-between;
		align-items: center;
		padding: 0.3rem 1rem;
		background-color: rgba(42, 42, 42, 0.8);
		border-bottom: 1px solid rgba(68, 68, 68, 0.7);
		min-height: 2.5rem;
	}
	
	.card-header.draggable {
		cursor: move;
		user-select: none;
	}
	
	.card-header.draggable:active {
		cursor: grabbing;
	}

	.card-title {
		margin: 0;
		font-size: 1rem;
		color: #ffffff;
		font-weight: 500;
		flex: 1;
		min-width: 0;
		white-space: nowrap;
		overflow: hidden;
		text-overflow: ellipsis;
	}

	.header-controls {
		display: flex;
		align-items: center;
		gap: 0.5rem;
		flex-shrink: 0;
	}

	.status-indicator {
		font-size: 1.2rem;
		color: #666;
		transition: color 0.3s ease;
	}

	.status-indicator.connected {
		color: #14a085;
	}

	.status-indicator.disconnected {
		color: #666;
	}

	.status-indicator.error {
		color: #e74c3c;
	}

	.control-btn {
		background: none;
		border: none;
		color: #999;
		font-size: 0.9rem;
		cursor: pointer;
		padding: 0.25rem;
		border-radius: 4px;
		transition: background-color 0.2s, color 0.2s;
		display: flex;
		align-items: center;
		justify-content: center;
		width: 24px;
		height: 24px;
	}

	.control-btn:hover {
		background-color: #444;
		color: #ffffff;
	}

	.close-btn:hover {
		background-color: #e74c3c;
		color: #ffffff;
	}

	.move-btn:hover {
		background-color: #3498db;
		color: #ffffff;
	}

	.resize-btn:hover {
		background-color: #f39c12;
		color: #ffffff;
	}

	.card-content {
		flex: 1;
		position: relative;
		overflow: hidden;
	}

	.card-footer {
		border-top: 1px solid rgba(68, 68, 68, 0.7);
		background-color: rgba(42, 42, 42, 0.8);
		padding: 0.5rem 1rem;
		font-size: 0.875rem;
		color: #cccccc;
	}

	/* Responsive adjustments */
	@media (max-width: 768px) {
		.card-header {
			padding: 0.25rem 0.75rem;
		}

		.card-title {
			font-size: 0.9rem;
		}

		.control-btn {
			width: 20px;
			height: 20px;
			font-size: 0.8rem;
		}
	}
</style>