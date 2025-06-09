<script lang="ts">
	import ConfigEditor from '$lib/components/config/ConfigEditor.svelte';
</script>

<div class="config-page">
	<div class="page-header">
		<h1>System Configuration</h1>
		<p>Manage your Open-Teleop configuration. Load, edit, and save YAML configurations with real-time validation.</p>
	</div>
	
	<ConfigEditor />
</div>

<style>
	.config-page {
		padding: 1rem;
		max-width: 100%;
		margin: 0 auto;
	}

	.page-header {
		text-align: center;
		margin-bottom: 2rem;
		padding-bottom: 1rem;
		border-bottom: 1px solid #444;
	}

	.page-header h1 {
		color: #ffffff;
		margin: 0 0 0.5rem 0;
		font-size: 2rem;
	}

	.page-header p {
		color: #cccccc;
		margin: 0;
		font-size: 1rem;
		max-width: 60ch;
		margin: 0 auto;
	}

	@media (max-width: 768px) {
		.page-header h1 {
			font-size: 1.5rem;
		}
		
		.page-header p {
			font-size: 0.9rem;
		}
	}
</style>