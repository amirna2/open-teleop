<script lang="ts">
	import Sidebar from './Sidebar.svelte';
	import Footer from './Footer.svelte';
</script>

<div class="app-layout">
	<Sidebar />
	<main class="main-content">
		<slot />
	</main>
	<Footer />
</div>

<style>
	.app-layout {
		display: grid;
		grid-template-columns: 200px 1fr;
		grid-template-rows: 1fr auto;
		grid-template-areas: 
			"sidebar main"
			"footer footer";
		height: 100vh;
		overflow: hidden;
	}

	.main-content {
		grid-area: main;
		background-color: #2a2a2a;
		overflow: auto;
		padding: 1rem;
	}

	:global(.sidebar) {
		grid-area: sidebar;
	}

	:global(.footer) {
		grid-area: footer;
	}
</style>