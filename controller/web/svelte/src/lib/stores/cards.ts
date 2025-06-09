import { writable } from 'svelte/store';

// Card registry types
export interface CardDefinition {
	id: string;
	name: string;
	description: string;
	component: string; // Component import path
	category: 'control' | 'video' | 'monitoring' | 'data';
	icon: string;
	defaultSize: {
		w: number;
		h: number;
	};
	minSize: {
		w: number;
		h: number;
	};
	maxSize?: {
		w: number;
		h: number;
	};
	defaultProps?: Record<string, any>;
	requiresConnection?: boolean;
}

export interface GridItem {
	id: string;
	x: number;
	y: number;
	w: number;
	h: number;
	component: string;
	props?: Record<string, any>;
	minW?: number;
	minH?: number;
	maxW?: number;
	maxH?: number;
	locked?: boolean;
}

// Available card types registry
export const CARD_REGISTRY: Record<string, CardDefinition> = {
	video: {
		id: 'video',
		name: 'Video Stream',
		description: 'Live video stream with statistics overlay',
		component: 'VideoCard',
		category: 'video',
		icon: '📹',
		defaultSize: { w: 6, h: 4 },
		minSize: { w: 3, h: 2 },
		maxSize: { w: 12, h: 8 },
		defaultProps: {
			title: 'Video Stream',
			showStats: true,
			allowOverlayToggle: true
		},
		requiresConnection: true
	},
	control: {
		id: 'control',
		name: 'Robot Control',
		description: 'Virtual joystick for robot teleoperation',
		component: 'ControlCard',
		category: 'control',
		icon: '🕹️',
		defaultSize: { w: 3, h: 3 },
		minSize: { w: 2, h: 2 },
		maxSize: { w: 4, h: 4 },
		defaultProps: {
			title: 'Robot Control',
			autoConnect: true
		},
		requiresConnection: true
	},
	status: {
		id: 'status',
		name: 'Robot Status',
		description: 'System health and diagnostic information',
		component: 'StatusCard',
		category: 'monitoring',
		icon: '📊',
		defaultSize: { w: 4, h: 3 },
		minSize: { w: 3, h: 2 },
		maxSize: { w: 6, h: 4 },
		defaultProps: {
			title: 'Robot Status'
		}
	},
	map: {
		id: 'map',
		name: 'Robot Map',
		description: 'Navigation map and robot position',
		component: 'MapCard',
		category: 'data',
		icon: '🗺️',
		defaultSize: { w: 6, h: 4 },
		minSize: { w: 4, h: 3 },
		maxSize: { w: 12, h: 8 },
		defaultProps: {
			title: 'Navigation Map'
		}
	},
	sensor: {
		id: 'sensor',
		name: 'Sensor Data',
		description: 'Real-time sensor readings and plots',
		component: 'SensorCard',
		category: 'data',
		icon: '📈',
		defaultSize: { w: 4, h: 3 },
		minSize: { w: 3, h: 2 },
		maxSize: { w: 8, h: 6 },
		defaultProps: {
			title: 'Sensor Data'
		}
	},
	logs: {
		id: 'logs',
		name: 'System Logs',
		description: 'Real-time system and error logs',
		component: 'LogsCard',
		category: 'monitoring',
		icon: '📋',
		defaultSize: { w: 6, h: 3 },
		minSize: { w: 4, h: 2 },
		maxSize: { w: 12, h: 6 },
		defaultProps: {
			title: 'System Logs'
		}
	}
};

// Grid layout store
export const gridItems = writable<GridItem[]>([]);
export const selectedCardType = writable<string | null>(null);
export const showAddDialog = writable<boolean>(false);

// Grid layout actions
export const gridActions = {
	addCard: (cardType: string, position?: { x: number; y: number }) => {
		const definition = CARD_REGISTRY[cardType];
		if (!definition) return;
		
		gridItems.update(items => {
			// Find available position if not specified
			let x = position?.x ?? 0;
			let y = position?.y ?? 0;
			
			if (!position) {
				// Find first available position
				const occupied = new Set(
					items.flatMap(item => {
						const positions = [];
						for (let dx = 0; dx < item.w; dx++) {
							for (let dy = 0; dy < item.h; dy++) {
								positions.push(`${item.x + dx},${item.y + dy}`);
							}
						}
						return positions;
					})
				);
				
				// Find first free position
				for (let row = 0; row < 20; row++) {
					for (let col = 0; col <= 12 - definition.defaultSize.w; col++) {
						let canPlace = true;
						for (let dx = 0; dx < definition.defaultSize.w; dx++) {
							for (let dy = 0; dy < definition.defaultSize.h; dy++) {
								if (occupied.has(`${col + dx},${row + dy}`)) {
									canPlace = false;
									break;
								}
							}
							if (!canPlace) break;
						}
						if (canPlace) {
							x = col;
							y = row;
							break;
						}
					}
					if (x !== 0 || y !== 0) break;
				}
			}
			
			const newItem: GridItem = {
				id: `${cardType}-${Date.now()}`,
				x,
				y,
				w: definition.defaultSize.w,
				h: definition.defaultSize.h,
				component: definition.component,
				props: { ...definition.defaultProps },
				minW: definition.minSize.w,
				minH: definition.minSize.h,
				maxW: definition.maxSize?.w,
				maxH: definition.maxSize?.h
			};
			
			return [...items, newItem];
		});
	},
	
	removeCard: (id: string) => {
		gridItems.update(items => items.filter(item => item.id !== id));
	},
	
	updateCard: (id: string, updates: Partial<GridItem>) => {
		gridItems.update(items => 
			items.map(item => item.id === id ? { ...item, ...updates } : item)
		);
	},
	
	moveCard: (id: string, x: number, y: number) => {
		gridItems.update(items => 
			items.map(item => item.id === id ? { ...item, x, y } : item)
		);
	},
	
	resizeCard: (id: string, w: number, h: number) => {
		gridItems.update(items => 
			items.map(item => item.id === id ? { ...item, w, h } : item)
		);
	},
	
	loadLayout: (layout: GridItem[]) => {
		gridItems.set(layout);
	},
	
	clearLayout: () => {
		gridItems.set([]);
	},
	
	// Preset layouts
	loadDefaultLayout: () => {
		const defaultItems: GridItem[] = [
			{
				id: 'video-main',
				x: 0,
				y: 0,
				w: 8,
				h: 5,
				component: 'VideoCard',
				props: {
					title: 'Primary Video Stream',
					streamId: 'primary',
					showStats: true,
					allowOverlayToggle: true
				},
				minW: 3,
				minH: 2,
				maxW: 12,
				maxH: 8
			},
			{
				id: 'control-main',
				x: 8,
				y: 0,
				w: 4,
				h: 3,
				component: 'ControlCard',
				props: {
					title: 'Robot Control',
					controlId: 'primary',
					autoConnect: true
				},
				minW: 2,
				minH: 2,
				maxW: 4,
				maxH: 4
			}
		];
		gridItems.set(defaultItems);
	}
};

// Add card dialog actions
export const addCardActions = {
	openDialog: () => {
		showAddDialog.set(true);
		selectedCardType.set(null);
	},
	
	closeDialog: () => {
		showAddDialog.set(false);
		selectedCardType.set(null);
	},
	
	selectCardType: (cardType: string) => {
		selectedCardType.set(cardType);
	},
	
	addSelectedCard: () => {
		const cardType = selectedCardType.get();
		if (cardType) {
			gridActions.addCard(cardType);
			addCardActions.closeDialog();
		}
	}
};

// Utility functions
export function getCardDefinition(cardType: string): CardDefinition | undefined {
	return CARD_REGISTRY[cardType];
}

export function getCardsByCategory(category: string): CardDefinition[] {
	return Object.values(CARD_REGISTRY).filter(card => card.category === category);
}

export function getAllCategories(): string[] {
	return [...new Set(Object.values(CARD_REGISTRY).map(card => card.category))];
}

// Store subscriptions helper
function createStoreSubscription<T>(store: ReturnType<typeof writable<T>>) {
	return {
		subscribe: store.subscribe,
		get: () => {
			let value: T;
			store.subscribe(v => value = v)();
			return value!;
		}
	};
}

// Enhanced store with get method
export const selectedCardTypeStore = createStoreSubscription(selectedCardType);
export const gridItemsStore = createStoreSubscription(gridItems);