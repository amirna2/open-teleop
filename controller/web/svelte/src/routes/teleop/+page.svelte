<script lang="ts">
	// TODO: Import actual components when implemented in Phase 2+
</script>

<div class="teleop-view">
	<div class="toolbar">
		<h1>Teleop Control</h1>
		<button class="add-component-btn">+ Add Component</button>
	</div>
	
	<div class="teleop-grid">
		<!-- Default layout: Large video card + overlay control card -->
		<div class="video-card-main">
			<div class="card">
				<div class="card-header">
					<h3>Primary Video Stream</h3>
					<span class="status-indicator connected">●</span>
				</div>
				<div class="card-content">
					<div class="video-placeholder">
						<p>Video stream placeholder</p>
						<small>WebCodecs integration coming in Phase 2</small>
					</div>
				</div>
			</div>
		</div>
		
		<div class="control-card-overlay">
			<div class="card">
				<div class="card-header">
					<h3>Robot Control</h3>
				</div>
				<div class="card-content">
					<div class="joystick-placeholder">
						<div class="joystick-circle">🎮</div>
						<small>Joystick integration coming in Phase 3</small>
					</div>
				</div>
			</div>
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
		grid-column: 1;
		grid-row: 1;
		z-index: 2;
		align-self: end;
		justify-self: end;
		width: 250px;
		height: 200px;
		margin: 1rem;
	}

	.card {
		background-color: #1e1e1e;
		border: 1px solid #444;
		border-radius: 8px;
		height: 100%;
		display: flex;
		flex-direction: column;
	}

	.card-header {
		display: flex;
		justify-content: space-between;
		align-items: center;
		padding: 0.75rem 1rem;
		border-bottom: 1px solid #444;
		background-color: #2a2a2a;
		border-radius: 8px 8px 0 0;
	}

	.card-header h3 {
		margin: 0;
		font-size: 1rem;
		color: #ffffff;
	}

	.status-indicator {
		font-size: 1.2rem;
	}

	.status-indicator.connected {
		color: #14a085;
	}

	.card-content {
		flex: 1;
		padding: 1rem;
		display: flex;
		align-items: center;
		justify-content: center;
		flex-direction: column;
	}

	.video-placeholder {
		text-align: center;
		color: #cccccc;
	}

	.joystick-placeholder {
		text-align: center;
		color: #cccccc;
	}

	.joystick-circle {
		font-size: 3rem;
		margin-bottom: 0.5rem;
	}
</style>