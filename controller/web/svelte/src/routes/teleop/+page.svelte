<script lang="ts">
	import { onMount } from 'svelte';
	import VideoCard from '$lib/components/cards/VideoCard.svelte';
	import ControlCard from '$lib/components/cards/ControlCard.svelte';
	import { videoStreamActions } from '$lib/stores/video';

	onMount(() => {
		// Ensure primary stream is registered
		videoStreamActions.registerStream('primary', 'Primary Video Stream');
	});
</script>

<div class="teleop-view">
	<div class="toolbar">
		<h1>Teleop Control</h1>
		<button class="add-component-btn">+ Add Component</button>
	</div>
	
	<div class="teleop-grid">
		<!-- Full-screen video background - EXACT preservation of functionality -->
		<div class="video-card-main">
			<VideoCard 
				title="Primary Video Stream" 
				streamId="primary"
				showStats={true}
				allowOverlayToggle={true}
			/>
		</div>
		
		<!-- Control card overlay - can be moved but not resized -->
		<div class="control-card-overlay">
			<ControlCard 
				title="Robot Control" 
				controlId="primary"
				autoConnect={true}
				movable={true}
			/>
		</div>
	</div>
</div>

<style>
	.teleop-view {
		height: 100%;
		display: flex;
		flex-direction: column;
	}

	.toolbar {
		display: flex;
		justify-content: space-between;
		align-items: center;
		padding: 1rem 0;
		border-bottom: 1px solid #444;
		margin-bottom: 1rem;
	}

	.toolbar h1 {
		margin: 0;
		font-size: 1.5rem;
		color: #ffffff;
	}

	.add-component-btn {
		background-color: #0d7377;
		color: white;
		border: none;
		padding: 0.5rem 1rem;
		border-radius: 4px;
		cursor: pointer;
		font-size: 0.9rem;
		transition: background-color 0.2s;
	}

	.add-component-btn:hover {
		background-color: #14a085;
	}

	.teleop-grid {
		flex: 1;
		position: relative;
		display: grid;
		grid-template-columns: 1fr;
		grid-template-rows: 1fr;
	}

	.video-card-main {
		grid-column: 1;
		grid-row: 1;
		z-index: 1;
	}

	.control-card-overlay {
		position: absolute;
		top: auto;
		bottom: 1rem;
		right: 1rem;
		width: 250px;
		height: 200px;
		z-index: 2;
	}
</style>