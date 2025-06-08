<script lang="ts">
	import MainLayout from '$lib/components/layout/MainLayout.svelte';
</script>

<MainLayout>
	<slot />
</MainLayout>

<style>
	:global(body) {
		margin: 0;
		padding: 0;
		font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
		background-color: #1a1a1a;
		color: #ffffff;
	}

	:global(*) {
		box-sizing: border-box;
	}
</style>