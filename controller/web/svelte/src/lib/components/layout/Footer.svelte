<script lang="ts">
	// TODO: Replace with actual health metrics from backend
	let cpuUsage = 45;
	let memoryUsage = 62;
	let networkThroughput = 1.2; // MB/s
</script>

<footer class="footer">
	<div class="health-metrics">
		<div class="metric">
			<span class="metric-icon">💻</span>
			<span class="metric-label">CPU:</span>
			<div class="metric-value">
				<div class="gauge">
					<div class="gauge-fill" style="width: {cpuUsage}%"></div>
				</div>
				<span class="metric-text">{cpuUsage}%</span>
			</div>
		</div>

		<div class="metric">
			<span class="metric-icon">💾</span>
			<span class="metric-label">Memory:</span>
			<div class="metric-value">
				<div class="gauge">
					<div class="gauge-fill" style="width: {memoryUsage}%"></div>
				</div>
				<span class="metric-text">{memoryUsage}%</span>
			</div>
		</div>

		<div class="metric">
			<span class="metric-icon">🌐</span>
			<span class="metric-label">Network:</span>
			<div class="metric-value">
				<span class="metric-text">{networkThroughput} MB/s</span>
			</div>
		</div>
	</div>
</footer>

<style>
	.footer {
		background-color: #1e1e1e;
		border-top: 1px solid #444;
		padding: 0.75rem 1rem;
	}

	.health-metrics {
		display: flex;
		justify-content: space-between;
		align-items: center;
		max-width: 600px;
		margin: 0 auto;
	}

	.metric {
		display: flex;
		align-items: center;
		gap: 0.5rem;
		min-width: 120px;
	}

	.metric-icon {
		font-size: 1rem;
	}

	.metric-label {
		font-size: 0.9rem;
		color: #cccccc;
		min-width: 50px;
	}

	.metric-value {
		display: flex;
		align-items: center;
		gap: 0.5rem;
	}

	.gauge {
		width: 60px;
		height: 8px;
		background-color: #444;
		border-radius: 4px;
		overflow: hidden;
	}

	.gauge-fill {
		height: 100%;
		background: linear-gradient(90deg, #14a085, #0d7377);
		transition: width 0.3s ease;
	}

	.metric-text {
		font-size: 0.8rem;
		color: #ffffff;
		font-weight: 500;
		min-width: 40px;
	}
</style>