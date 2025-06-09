<script lang="ts">
	import { onMount, createEventDispatcher } from 'svelte';
	
	// Grid configuration
	export let columns = 12; // Number of grid columns
	export let rowHeight = 60; // Height of each grid row in pixels
	export let gap = 8; // Gap between grid items
	export let enableDrag = true; // Enable dragging
	export let enableResize = true; // Enable resizing
	
	// Grid items data structure
	export let items: GridItem[] = [];
	
	// Types
	interface GridItem {
		id: string;
		x: number; // Grid column (0-based)
		y: number; // Grid row (0-based)
		w: number; // Width in grid units
		h: number; // Height in grid units
		component: string; // Component type
		props?: Record<string, any>; // Component props
		minW?: number; // Minimum width
		minH?: number; // Minimum height
		maxW?: number; // Maximum width
		maxH?: number; // Maximum height
		locked?: boolean; // Prevent moving/resizing
	}
	
	// Events
	const dispatch = createEventDispatcher<{
		itemMoved: { id: string; x: number; y: number };
		itemResized: { id: string; w: number; h: number };
		itemRemoved: { id: string };
		layoutChanged: { items: GridItem[] };
	}>();
	
	// State
	let gridContainer: HTMLElement;
	let draggedItem: GridItem | null = null;
	let resizeItem: GridItem | null = null;
	let dragOffset = { x: 0, y: 0 };
	let resizeDirection = '';
	let isDragging = false;
	let isResizing = false;
	
	// Grid calculations
	$: gridWidth = gridContainer?.clientWidth || 0;
	$: cellWidth = (gridWidth - gap * (columns - 1)) / columns;
	$: gridHeight = Math.max(...items.map(item => item.y + item.h)) * (rowHeight + gap);
	
	onMount(() => {
		// Set up global event listeners for drag and resize
		document.addEventListener('mousemove', handleMouseMove);
		document.addEventListener('mouseup', handleMouseUp);
		document.addEventListener('keydown', handleKeyDown);
		
		return () => {
			document.removeEventListener('mousemove', handleMouseMove);
			document.removeEventListener('mouseup', handleMouseUp);
			document.removeEventListener('keydown', handleKeyDown);
		};
	});
	
	function handleMouseMove(e: MouseEvent) {
		if (isDragging && draggedItem && enableDrag) {
			const rect = gridContainer.getBoundingClientRect();
			const x = e.clientX - rect.left - dragOffset.x;
			const y = e.clientY - rect.top - dragOffset.y;
			
			// Convert to grid coordinates
			const newX = Math.max(0, Math.min(columns - draggedItem.w, Math.round(x / (cellWidth + gap))));
			const newY = Math.max(0, Math.round(y / (rowHeight + gap)));
			
			// Update item position
			const newItems = items.map(item => 
				item.id === draggedItem!.id ? { ...item, x: newX, y: newY } : item
			);
			items = newItems;
		}
		
		if (isResizing && resizeItem && enableResize) {
			const rect = gridContainer.getBoundingClientRect();
			const x = e.clientX - rect.left;
			const y = e.clientY - rect.top;
			
			// Calculate new size based on resize direction
			let newW = resizeItem.w;
			let newH = resizeItem.h;
			
			const itemRect = getItemRect(resizeItem);
			
			if (resizeDirection.includes('e')) {
				newW = Math.max(
					resizeItem.minW || 1,
					Math.min(
						resizeItem.maxW || columns,
						Math.round((x - itemRect.left) / (cellWidth + gap))
					)
				);
			}
			
			if (resizeDirection.includes('s')) {
				newH = Math.max(
					resizeItem.minH || 1,
					Math.min(
						resizeItem.maxH || 20,
						Math.round((y - itemRect.top) / (rowHeight + gap))
					)
				);
			}
			
			// Update item size
			const newItems = items.map(item => 
				item.id === resizeItem!.id ? { ...item, w: newW, h: newH } : item
			);
			items = newItems;
		}
	}
	
	function handleMouseUp() {
		if (isDragging && draggedItem) {
			dispatch('itemMoved', { 
				id: draggedItem.id, 
				x: draggedItem.x, 
				y: draggedItem.y 
			});
			draggedItem = null;
			isDragging = false;
		}
		
		if (isResizing && resizeItem) {
			dispatch('itemResized', { 
				id: resizeItem.id, 
				w: resizeItem.w, 
				h: resizeItem.h 
			});
			resizeItem = null;
			isResizing = false;
		}
		
		dispatch('layoutChanged', { items });
	}
	
	function handleKeyDown(e: KeyboardEvent) {
		if (e.key === 'Escape') {
			// Cancel drag/resize
			isDragging = false;
			isResizing = false;
			draggedItem = null;
			resizeItem = null;
		}
	}
	
	function startDrag(item: GridItem, e: MouseEvent) {
		if (!enableDrag || item.locked) return;
		
		e.preventDefault();
		draggedItem = item;
		isDragging = true;
		
		const rect = gridContainer.getBoundingClientRect();
		const itemRect = getItemRect(item);
		dragOffset.x = e.clientX - rect.left - itemRect.left;
		dragOffset.y = e.clientY - rect.top - itemRect.top;
	}
	
	function startResize(item: GridItem, direction: string, e: MouseEvent) {
		if (!enableResize || item.locked) return;
		
		e.preventDefault();
		e.stopPropagation();
		resizeItem = item;
		resizeDirection = direction;
		isResizing = true;
	}
	
	function removeItem(id: string) {
		items = items.filter(item => item.id !== id);
		dispatch('itemRemoved', { id });
		dispatch('layoutChanged', { items });
	}
	
	function getItemRect(item: GridItem) {
		return {
			left: item.x * (cellWidth + gap),
			top: item.y * (rowHeight + gap),
			width: item.w * cellWidth + (item.w - 1) * gap,
			height: item.h * rowHeight + (item.h - 1) * gap
		};
	}
	
	function getItemStyle(item: GridItem) {
		const rect = getItemRect(item);
		return `
			position: absolute;
			left: ${rect.left}px;
			top: ${rect.top}px;
			width: ${rect.width}px;
			height: ${rect.height}px;
			z-index: ${isDragging && draggedItem?.id === item.id ? 1000 : 1};
		`;
	}
</script>

<div 
	class="dynamic-grid" 
	bind:this={gridContainer}
	style="height: {gridHeight}px;"
>
	{#each items as item (item.id)}
		<div 
			class="grid-item"
			class:dragging={isDragging && draggedItem?.id === item.id}
			class:resizing={isResizing && resizeItem?.id === item.id}
			class:locked={item.locked}
			style={getItemStyle(item)}
		>
			<!-- Drag handle -->
			{#if enableDrag && !item.locked}
				<div 
					class="drag-handle"
					on:mousedown={(e) => startDrag(item, e)}
					title="Drag to move"
				>
					⋮⋮
				</div>
			{/if}
			
			<!-- Remove button -->
			<button 
				class="remove-btn"
				on:click={() => removeItem(item.id)}
				title="Remove component"
			>
				✕
			</button>
			
			<!-- Content slot -->
			<div class="item-content">
				<slot name="item" {item}>
					<div class="placeholder">
						<p>Component: {item.component}</p>
						<p>Position: {item.x}, {item.y}</p>
						<p>Size: {item.w}×{item.h}</p>
					</div>
				</slot>
			</div>
			
			<!-- Resize handles -->
			{#if enableResize && !item.locked}
				<div 
					class="resize-handle resize-se"
					on:mousedown={(e) => startResize(item, 'se', e)}
					title="Resize"
				></div>
				<div 
					class="resize-handle resize-e"
					on:mousedown={(e) => startResize(item, 'e', e)}
				></div>
				<div 
					class="resize-handle resize-s"
					on:mousedown={(e) => startResize(item, 's', e)}
				></div>
			{/if}
		</div>
	{/each}
</div>

<style>
	.dynamic-grid {
		position: relative;
		width: 100%;
		min-height: 200px;
		background: transparent;
		user-select: none;
	}

	.grid-item {
		border: 1px solid #444;
		border-radius: 8px;
		background-color: #1e1e1e;
		overflow: hidden;
		transition: box-shadow 0.2s ease;
		cursor: default;
	}

	.grid-item:hover {
		border-color: #0d7377;
	}

	.grid-item.dragging {
		border-color: #14a085;
		box-shadow: 0 8px 20px rgba(20, 160, 133, 0.4);
		cursor: grabbing;
	}

	.grid-item.resizing {
		border-color: #f39c12;
		box-shadow: 0 4px 12px rgba(243, 156, 18, 0.3);
	}

	.grid-item.locked {
		border-color: #666;
		opacity: 0.8;
	}

	.drag-handle {
		position: absolute;
		top: 4px;
		left: 4px;
		width: 16px;
		height: 16px;
		background: rgba(255, 255, 255, 0.1);
		border-radius: 2px;
		cursor: grab;
		display: flex;
		align-items: center;
		justify-content: center;
		font-size: 10px;
		color: #999;
		z-index: 10;
		transition: background-color 0.2s;
	}

	.drag-handle:hover {
		background: rgba(255, 255, 255, 0.2);
		color: #fff;
	}

	.drag-handle:active {
		cursor: grabbing;
	}

	.remove-btn {
		position: absolute;
		top: 4px;
		right: 4px;
		width: 20px;
		height: 20px;
		background: rgba(231, 76, 60, 0.8);
		border: none;
		border-radius: 50%;
		color: white;
		font-size: 12px;
		cursor: pointer;
		display: flex;
		align-items: center;
		justify-content: center;
		z-index: 10;
		transition: background-color 0.2s;
	}

	.remove-btn:hover {
		background: rgba(231, 76, 60, 1);
	}

	.item-content {
		width: 100%;
		height: 100%;
		padding: 24px 8px 8px 8px;
		overflow: hidden;
	}

	.placeholder {
		padding: 1rem;
		text-align: center;
		color: #999;
		font-size: 0.875rem;
	}

	.placeholder p {
		margin: 0.25rem 0;
	}

	.resize-handle {
		position: absolute;
		background: rgba(20, 160, 133, 0.8);
		z-index: 10;
	}

	.resize-handle:hover {
		background: rgba(20, 160, 133, 1);
	}

	.resize-se {
		bottom: 0;
		right: 0;
		width: 12px;
		height: 12px;
		cursor: se-resize;
		border-radius: 8px 0 8px 0;
	}

	.resize-e {
		top: 50%;
		right: 0;
		width: 4px;
		height: 20px;
		cursor: e-resize;
		transform: translateY(-50%);
		border-radius: 4px 0 0 4px;
	}

	.resize-s {
		bottom: 0;
		left: 50%;
		width: 20px;
		height: 4px;
		cursor: s-resize;
		transform: translateX(-50%);
		border-radius: 4px 4px 0 0;
	}

	/* Hide controls when not needed */
	.grid-item:not(:hover) .drag-handle,
	.grid-item:not(:hover) .remove-btn,
	.grid-item:not(:hover) .resize-handle {
		opacity: 0;
		transition: opacity 0.2s ease;
	}

	.grid-item:hover .drag-handle,
	.grid-item:hover .remove-btn,
	.grid-item:hover .resize-handle {
		opacity: 1;
	}

	.grid-item.dragging .drag-handle,
	.grid-item.resizing .resize-handle {
		opacity: 1;
	}

	/* Responsive adjustments */
	@media (max-width: 768px) {
		.drag-handle,
		.remove-btn,
		.resize-handle {
			opacity: 1; /* Always show on mobile */
		}
		
		.drag-handle,
		.remove-btn {
			width: 24px;
			height: 24px;
		}
	}
</style>