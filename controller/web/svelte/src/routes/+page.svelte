<script lang="ts">
	import { goto } from '$app/navigation';
	import { onMount } from 'svelte';

	// Redirect to teleop as default view
	onMount(() => {
		goto('/teleop', { replaceState: true });
	});
</script>

<div class="loading">
	<p>Loading...</p>
</div>

<style>
	.loading {
		display: flex;
		justify-content: center;
		align-items: center;
		height: 100%;
		color: #cccccc;
	}
</style>