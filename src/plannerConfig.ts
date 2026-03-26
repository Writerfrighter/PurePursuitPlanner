import type { Obstacle, PlannerSettings } from './types';

export const FIELD = {
  width: 651.2,
  height: 317.69,
};

export const DEFAULT_SETTINGS: PlannerSettings = {
  robotW: 28,
  robotL: 28,
  showGrid: true,
  showGhost: true,
  snap: 0,
  maxVel: 150,
  maxAccel: 120,
  maxDecel: 140,
  maxTurnRate: 360,
};

export const DEFAULT_OBSTACLES: Obstacle[] = [
  { id: 'hub_blue', cx: FIELD.width - 182.11, cy: FIELD.height / 2, w: 47, h: 47, label: 'Blue HUB', color: '#4a9eff', blocked: true, category: 'hub' },
  { id: 'hub_red', cx: 182.11, cy: FIELD.height / 2, w: 47, h: 47, label: 'Red HUB', color: '#ff5a5a', blocked: true, category: 'hub' },
  { id: 'bump_bl', cx: FIELD.width - 182.11, cy: 99.17, w: 44.4, h: 73, label: 'BUMP', color: '#4a6eff', blocked: false, category: 'bump' },
  { id: 'bump_br', cx: FIELD.width - 182.11, cy: FIELD.height - 99.17, w: 44.4, h: 73, label: 'BUMP', color: '#4a6eff', blocked: false, category: 'bump' },
  { id: 'bump_rl', cx: 182.11, cy: 99.17, w: 44.4, h: 73, label: 'BUMP', color: '#ff6a3a', blocked: false, category: 'bump' },
  { id: 'bump_rr', cx: 182.11, cy: FIELD.height - 99.17, w: 44.4, h: 73, label: 'BUMP', color: '#ff6a3a', blocked: false, category: 'bump' },
  { id: 'trench_bl', cx: FIELD.width - 182.11, cy: 23.5, w: 4, h: 50.67, label: 'TRENCH', color: '#4a6eff', blocked: false, category: 'trench' },
  { id: 'trench_br', cx: FIELD.width - 182.11, cy: FIELD.height - 23.5, w: 4, h: 50.67, label: 'TRENCH', color: '#4a6eff', blocked: false, category: 'trench' },
  { id: 'trench_rl', cx: 182.11, cy: 23.5, w: 4, h: 50.67, label: 'TRENCH', color: '#ff6a3a', blocked: false, category: 'trench' },
  { id: 'trench_rr', cx: 182.11, cy: FIELD.height - 23.5, w: 4, h: 50.67, label: 'TRENCH', color:'#ff6a3a', blocked: false, category: 'trench' },
  { id: 'trench_block_bl', cx: FIELD.width - 182.11, cy: 12/2 + 50.67, w: 47, h: 12, label: 'TRENCH BLOCK', color: '#4a9eff', blocked: true, category: 'trench' },
  { id: 'trench_block_br', cx: FIELD.width - 182.11, cy: FIELD.height - 12/2 - 50.67, w: 47, h: 12, label: 'TRENCH BLOCK', color: '#4a9eff', blocked: true, category: 'trench' },
  { id: 'trench_block_rl', cx: 182.11, cy: 12/2 + 50.67, w: 47, h: 12, label: 'TRENCH BLOCK', color: '#ff5a5a', blocked: true, category: 'trench' },
  { id: 'trench_block_rr', cx: 182.11, cy: FIELD.height - 12/2 - 50.67, w: 47, h: 12, label: 'TRENCH BLOCK', color:'#ff5a5a', blocked: true, category: 'trench' },
  { id: 'depot_b', cx: FIELD.width - 27/2, cy: FIELD.height/2 + 75.93, w: 27, h: 42, label: 'Blue DEPOT', color: 'rgba(74,158,255,.45)', blocked: false, category: 'depot' },
  { id: 'depot_r', cx: 27/2, cy: FIELD.height/2 - 75.93, w: 27, h: 42, label: 'Red DEPOT', color: 'rgba(255,90,90,.45)', blocked: false, category: 'depot' },
  { id: 'fuel', cx: FIELD.width / 2, cy: FIELD.height / 2, w: 35.95*2, h: 90.95*2, label: 'FUEL', color: 'rgba(235, 199, 42, 0.58)', blocked: false, category: 'fuel' },
  { id: 'tower_b', cx: FIELD.width - 41.06, cy: FIELD.height / 2 - 11.38, w: 2, h: 47, label: 'Blue TOWER', color: 'rgba(74,158,255,.45)', blocked: true, category: 'other' },
  { id: 'tower_r', cx: 41.06, cy: FIELD.height / 2 + 11.38, w: 2, h: 47, label: 'Red TOWER', color: 'rgba(255,90,90,.45)', blocked: true, category: 'other' },
];
