<script lang="ts">
	import { createEventDispatcher } from 'svelte';
	import { 
		CARD_REGISTRY, 
		selectedCardType, 
		showAddDialog, 
		addCardActions,
		getAllCategories,
		getCardsByCategory,
		type CardDefinition 
	} from '$lib/stores/cards';
	
	// Props
	export let open = false;
	
	// Events
	const dispatch = createEventDispatcher<{
		cardSelected: { cardType: string };
		dialogClosed: {};
	}>();
	
	// Local state
	let selectedCategory = 'control';
	let selectedCard: string | null = null;
	
	// Reactive data
	$: categories = getAllCategories();
	$: cardsInCategory = getCardsByCategory(selectedCategory);
	$: cardDefinition = selectedCard ? CARD_REGISTRY[selectedCard] : null;
	
	// Handle category selection
	function selectCategory(category: string) {
		selectedCategory = category;
		selectedCard = null;
	}
	
	// Handle card selection
	function selectCard(cardType: string) {
		selectedCard = cardType;
		selectedCardType.set(cardType);
	}
	
	// Add selected card
	function addCard() {
		if (selectedCard) {
			dispatch('cardSelected', { cardType: selectedCard });
			closeDialog();
		}
	}
	
	// Close dialog
	function closeDialog() {
		open = false;
		selectedCard = null;
		selectedCardType.set(null);
		dispatch('dialogClosed');
	}
	
	// Handle escape key
	function handleKeydown(e: KeyboardEvent) {
		if (e.key === 'Escape') {
			closeDialog();
		}
	}
	
	// Category styling
	function getCategoryIcon(category: string): string {
		const icons: Record<string, string> = {
			control: '🕹️',
			video: '📹',
			monitoring: '📊',
			data: '📈'
		};
		return icons[category] || '📦';
	}
	
	function getCategoryColor(category: string): string {
		const colors: Record<string, string> = {
			control: '#14a085',
			video: '#3498db',
			monitoring: '#e67e22',
			data: '#9b59b6'
		};
		return colors[category] || '#0d7377';
	}
</script>

<!-- Backdrop -->
{#if open}
	<div 
		class="dialog-backdrop" 
		on:click={closeDialog}
		on:keydown={handleKeydown}
		tabindex="-1"
		role="presentation"
	>
		<!-- Dialog -->
		<div 
			class="dialog"
			on:click|stopPropagation
			role="dialog"
			aria-labelledby="dialog-title"
			aria-modal="true"
		>
			<!-- Header -->
			<div class="dialog-header">
				<h2 id="dialog-title">Add Component</h2>
				<button class="close-btn" on:click={closeDialog} title="Close">
					✕
				</button>
			</div>
			
			<!-- Content -->
			<div class="dialog-content">
				<!-- Categories -->
				<div class="categories">
					<h3>Categories</h3>
					<div class="category-list">
						{#each categories as category}
							<button
								class="category-item"
								class:active={selectedCategory === category}
								style="--category-color: {getCategoryColor(category)}"
								on:click={() => selectCategory(category)}
							>
								<span class="category-icon">{getCategoryIcon(category)}</span>
								<span class="category-name">{category.charAt(0).toUpperCase() + category.slice(1)}</span>
							</button>
						{/each}
					</div>
				</div>
				
				<!-- Card Grid -->
				<div class="cards">
					<h3>Available Components</h3>
					<div class="card-grid">
						{#each cardsInCategory as card (card.id)}
							<button
								class="card-item"
								class:selected={selectedCard === card.id}
								on:click={() => selectCard(card.id)}
							>
								<div class="card-icon">{card.icon}</div>
								<div class="card-info">
									<h4>{card.name}</h4>
									<p>{card.description}</p>
									<div class="card-meta">
										<span class="size-info">
											{card.defaultSize.w}×{card.defaultSize.h} grid
										</span>
										{#if card.requiresConnection}
											<span class="connection-required" title="Requires connection">
												🔗
											</span>
										{/if}
									</div>
								</div>
							</button>
						{/each}
					</div>
				</div>
				
				<!-- Preview -->
				{#if cardDefinition}
					<div class="preview">
						<h3>Preview</h3>
						<div class="preview-card">
							<div class="preview-icon">{cardDefinition.icon}</div>
							<h4>{cardDefinition.name}</h4>
							<p>{cardDefinition.description}</p>
							<div class="preview-details">
								<div class="detail">
									<strong>Size:</strong> {cardDefinition.defaultSize.w}×{cardDefinition.defaultSize.h}
								</div>
								<div class="detail">
									<strong>Min:</strong> {cardDefinition.minSize.w}×{cardDefinition.minSize.h}
								</div>
								{#if cardDefinition.maxSize}
									<div class="detail">
										<strong>Max:</strong> {cardDefinition.maxSize.w}×{cardDefinition.maxSize.h}
									</div>
								{/if}
								<div class="detail">
									<strong>Category:</strong> {cardDefinition.category}
								</div>
							</div>
						</div>
					</div>
				{/if}
			</div>
			
			<!-- Footer -->
			<div class="dialog-footer">
				<button class="cancel-btn" on:click={closeDialog}>
					Cancel
				</button>
				<button 
					class="add-btn" 
					disabled={!selectedCard}
					on:click={addCard}
				>
					Add Component
				</button>
			</div>
		</div>
	</div>
{/if}

<style>
	.dialog-backdrop {
		position: fixed;
		top: 0;
		left: 0;
		right: 0;
		bottom: 0;
		background: rgba(0, 0, 0, 0.7);
		backdrop-filter: blur(4px);
		display: flex;
		align-items: center;
		justify-content: center;
		z-index: 1000;
		padding: 1rem;
	}

	.dialog {
		background: #1e1e1e;
		border: 1px solid #444;
		border-radius: 12px;
		width: 100%;
		max-width: 900px;
		max-height: 80vh;
		display: flex;
		flex-direction: column;
		box-shadow: 0 20px 40px rgba(0, 0, 0, 0.5);
	}

	.dialog-header {
		display: flex;
		justify-content: space-between;
		align-items: center;
		padding: 1.5rem;
		border-bottom: 1px solid #444;
	}

	.dialog-header h2 {
		margin: 0;
		color: #ffffff;
		font-size: 1.5rem;
	}

	.close-btn {
		background: none;
		border: none;
		color: #999;
		font-size: 1.5rem;
		cursor: pointer;
		padding: 0.5rem;
		border-radius: 50%;
		transition: background-color 0.2s, color 0.2s;
	}

	.close-btn:hover {
		background: #444;
		color: #fff;
	}

	.dialog-content {
		flex: 1;
		display: grid;
		grid-template-columns: 200px 1fr 250px;
		gap: 1.5rem;
		padding: 1.5rem;
		overflow: hidden;
	}

	.categories h3,
	.cards h3,
	.preview h3 {
		margin: 0 0 1rem 0;
		color: #ffffff;
		font-size: 1rem;
		font-weight: 600;
	}

	.category-list {
		display: flex;
		flex-direction: column;
		gap: 0.5rem;
	}

	.category-item {
		display: flex;
		align-items: center;
		gap: 0.75rem;
		padding: 0.75rem;
		background: none;
		border: 1px solid #444;
		border-radius: 8px;
		color: #cccccc;
		cursor: pointer;
		transition: all 0.2s;
		text-align: left;
	}

	.category-item:hover {
		border-color: var(--category-color);
		background: rgba(255, 255, 255, 0.05);
	}

	.category-item.active {
		border-color: var(--category-color);
		background: var(--category-color);
		color: white;
	}

	.category-icon {
		font-size: 1.2rem;
	}

	.category-name {
		font-size: 0.9rem;
		font-weight: 500;
	}

	.card-grid {
		display: flex;
		flex-direction: column;
		gap: 0.75rem;
		overflow-y: auto;
		max-height: 400px;
		padding-right: 0.5rem;
	}

	.card-item {
		display: flex;
		align-items: flex-start;
		gap: 1rem;
		padding: 1rem;
		background: none;
		border: 1px solid #444;
		border-radius: 8px;
		color: #cccccc;
		cursor: pointer;
		transition: all 0.2s;
		text-align: left;
	}

	.card-item:hover {
		border-color: #0d7377;
		background: rgba(255, 255, 255, 0.05);
	}

	.card-item.selected {
		border-color: #14a085;
		background: rgba(20, 160, 133, 0.1);
		color: #ffffff;
	}

	.card-icon {
		font-size: 2rem;
		flex-shrink: 0;
	}

	.card-info {
		flex: 1;
		min-width: 0;
	}

	.card-info h4 {
		margin: 0 0 0.5rem 0;
		font-size: 1rem;
		font-weight: 600;
		color: inherit;
	}

	.card-info p {
		margin: 0 0 0.75rem 0;
		font-size: 0.875rem;
		color: #999;
		line-height: 1.4;
	}

	.card-meta {
		display: flex;
		align-items: center;
		justify-content: space-between;
		font-size: 0.75rem;
		color: #777;
	}

	.size-info {
		background: rgba(255, 255, 255, 0.1);
		padding: 0.25rem 0.5rem;
		border-radius: 4px;
	}

	.connection-required {
		font-size: 1rem;
	}

	.preview {
		background: #2a2a2a;
		border-radius: 8px;
		padding: 1rem;
		height: fit-content;
	}

	.preview-card {
		text-align: center;
	}

	.preview-icon {
		font-size: 3rem;
		margin-bottom: 1rem;
	}

	.preview-card h4 {
		margin: 0 0 0.5rem 0;
		color: #ffffff;
		font-size: 1.125rem;
	}

	.preview-card p {
		margin: 0 0 1rem 0;
		color: #cccccc;
		font-size: 0.875rem;
		line-height: 1.4;
	}

	.preview-details {
		text-align: left;
		border-top: 1px solid #444;
		padding-top: 1rem;
	}

	.detail {
		margin-bottom: 0.5rem;
		font-size: 0.875rem;
		color: #cccccc;
	}

	.detail strong {
		color: #ffffff;
	}

	.dialog-footer {
		display: flex;
		justify-content: flex-end;
		gap: 1rem;
		padding: 1.5rem;
		border-top: 1px solid #444;
	}

	.cancel-btn,
	.add-btn {
		padding: 0.75rem 1.5rem;
		border: none;
		border-radius: 6px;
		font-size: 0.9rem;
		font-weight: 500;
		cursor: pointer;
		transition: all 0.2s;
	}

	.cancel-btn {
		background: #444;
		color: #ffffff;
	}

	.cancel-btn:hover {
		background: #555;
	}

	.add-btn {
		background: #14a085;
		color: white;
	}

	.add-btn:hover:not(:disabled) {
		background: #0d7377;
	}

	.add-btn:disabled {
		background: #666;
		color: #999;
		cursor: not-allowed;
	}

	/* Responsive adjustments */
	@media (max-width: 768px) {
		.dialog {
			margin: 1rem;
			max-width: none;
		}

		.dialog-content {
			grid-template-columns: 1fr;
			grid-template-rows: auto auto auto;
			gap: 1rem;
		}

		.card-grid {
			max-height: 300px;
		}
	}
</style>