<script lang="ts">
	// TODO: Import actual config components when implemented in Phase 5
</script>

<div class="config-view">
	<div class="config-header">
		<h1>Configuration</h1>
		<div class="config-actions">
			<button class="load-btn">Load Config</button>
			<button class="save-btn">Save Config</button>
		</div>
	</div>
	
	<div class="config-editor">
		<div class="editor-placeholder">
			<h3>YAML Configuration Editor</h3>
			<p>Configuration editor integration coming in Phase 5</p>
			<div class="mock-editor">
				<pre><code># open_teleop_config.yaml
topics:
  cmd_vel:
    ott_id: "robot_control"
    priority: HIGH
    direction: OUTBOUND
    throttle_rate_hz: 30

  camera_feed:
    ott_id: "primary_video"
    priority: STANDARD  
    direction: INBOUND
    throttle_rate_hz: 30</code></pre>
			</div>
		</div>
	</div>
</div>

<style>
	.config-view {
		height: 100%;
		display: flex;
		flex-direction: column;
	}

	.config-header {
		display: flex;
		justify-content: space-between;
		align-items: center;
		padding: 1rem 0;
		border-bottom: 1px solid #444;
		margin-bottom: 1rem;
	}

	.config-header h1 {
		margin: 0;
		font-size: 1.5rem;
		color: #ffffff;
	}

	.config-actions {
		display: flex;
		gap: 0.5rem;
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

	.load-btn:hover, .save-btn:hover {
		background-color: #14a085;
	}

	.config-editor {
		flex: 1;
		background-color: #1e1e1e;
		border: 1px solid #444;
		border-radius: 8px;
		padding: 1rem;
	}

	.editor-placeholder {
		text-align: center;
		color: #cccccc;
		height: 100%;
		display: flex;
		flex-direction: column;
		align-items: center;
		justify-content: center;
	}

	.editor-placeholder h3 {
		margin-bottom: 0.5rem;
		color: #ffffff;
	}

	.mock-editor {
		margin-top: 2rem;
		background-color: #2a2a2a;
		border: 1px solid #555;
		border-radius: 4px;
		padding: 1rem;
		text-align: left;
		max-width: 500px;
	}

	.mock-editor pre {
		margin: 0;
		color: #cccccc;
		font-family: 'Courier New', monospace;
		font-size: 0.9rem;
		line-height: 1.4;
	}
</style>