<script lang="ts">
	import { page } from '$app/stores';

	$: currentPath = $page.url.pathname;
</script>

<nav class="sidebar">
	<div class="sidebar-header">
		<h2>Open-Teleop</h2>
	</div>
	
	<ul class="nav-menu">
		<li>
			<a href="/" class:active={currentPath === '/' || currentPath === '/teleop'}>
				<span class="nav-icon">🎮</span>
				Teleop
			</a>
		</li>
		<li>
			<a href="/config" class:active={currentPath === '/config'}>
				<span class="nav-icon">⚙️</span>
				Configuration
			</a>
		</li>
	</ul>
</nav>

<style>
	.sidebar {
		background-color: #1e1e1e;
		border-right: 1px solid #444;
		display: flex;
		flex-direction: column;
		padding: 0;
	}

	.sidebar-header {
		padding: 1rem;
		border-bottom: 1px solid #444;
		text-align: center;
	}

	.sidebar-header h2 {
		margin: 0;
		font-size: 1.2rem;
		color: #ffffff;
	}

	.nav-menu {
		list-style: none;
		padding: 0;
		margin: 0;
		flex: 1;
	}

	.nav-menu li {
		margin: 0;
	}

	.nav-menu a {
		display: flex;
		align-items: center;
		padding: 1rem;
		color: #cccccc;
		text-decoration: none;
		transition: background-color 0.2s, color 0.2s;
		border-left: 3px solid transparent;
	}

	.nav-menu a:hover {
		background-color: #333;
		color: #ffffff;
	}

	.nav-menu a.active {
		background-color: #0d7377;
		color: #ffffff;
		border-left-color: #14a085;
	}

	.nav-icon {
		margin-right: 0.75rem;
		font-size: 1.1rem;
	}
</style>