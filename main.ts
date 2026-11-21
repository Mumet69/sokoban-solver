// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
// Push-optimal A* over pushes, room by room: the generator's levels are a hallway
// with 1-4 rooms off it, each room 3-4 boxes, and a box never leaves its room, so
// the level optimum is the sum of the rooms' optima. Each room is solved with an
// A* whose state is the boxes plus the player's reachable region, pruned by dead
// squares (a box that can never reach a goal) and frozen 2x2 blocks, and guided by
// an admissible min-cost matching heuristic (boxes to goals by shortest-path
// distance). Deterministic: no clock, no randomness; the search budget is counted
// in states, never in time, so two runs write the same bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

/** The most states one room search keeps before it gives up (work, never the clock). */
const MAX_STATES = 1_000_000;
/** The most states the whole run spends across all levels, so the 60 s wall is never near. */
const GLOBAL_STATES = 4_000_000;

const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/** The squares from which a box alone can be pushed onto some goal: pull back from the goals over open floor. */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** Walking distance from `from` to every square, around walls only (boxes ignored). */
function bfsDist(w: number, wall: Uint8Array, from: number): Int32Array {
  const dist = new Int32Array(wall.length).fill(-1);
  const queue = [from];
  dist[from] = 0;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    for (const [dx, dy] of DIRS) {
      const q = p + dy * w + dx;
      if (!wall[q] && dist[q] < 0) {
        dist[q] = dist[p] + 1;
        queue.push(q);
      }
    }
  }
  return dist;
}

/**
 * The min-cost assignment of boxes to goals, by shortest-path distance.
 * A lower bound on the pushes still needed (each push moves one box one square),
 * so the A* heuristic is admissible; Infinity when no assignment exists, which
 * means the state is dead: a box's reachable goals only shrink as it moves, so
 * no later push can open an assignment. Brute force for up to 7 boxes, else the
 * sum of each box's nearest goal (a weaker but still admissible bound).
 */
function matchingCost(boxes: number[], distGoal: Int32Array[]): number {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n <= 7) {
    const perm = [...Array(n).keys()];
    let best = Infinity;
    const rec = (k: number, acc: number): void => {
      if (acc >= best) return;
      if (k === n) {
        if (acc < best) best = acc;
        return;
      }
      for (let j = k; j < n; j++) {
        [perm[k], perm[j]] = [perm[j], perm[k]];
        const d = distGoal[perm[k]][boxes[k]];
        if (d >= 0) rec(k + 1, acc + d);
        [perm[k], perm[j]] = [perm[j], perm[k]];
      }
    };
    rec(0, 0);
    return best;
  }
  let sum = 0;
  for (const b of boxes) {
    let m = Infinity;
    for (const dg of distGoal) if (dg[b] >= 0 && dg[b] < m) m = dg[b];
    if (m === Infinity) return Infinity;
    sum += m;
  }
  return sum;
}

/** The lowest square of the region the player reaches from `from`, marking it in `seen` with `stamp`. */
function regionLow(
  level: Level,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (const [dx, dy] of DIRS) {
      const q = p + dy * w + dx;
      if (!wall[q] && !occupied[q] && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block of walls and boxes with a box off its goal. */
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b])) continue;
    if (!(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b])) return true;
    if ((occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** The shortest walk from `from` to `to` around walls and boxes, in lower LURD letters. */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: the push square is out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

/** The square the player stands on after `moves` (LURD, case aside) from `from` on a board `w` wide. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

interface SState {
  boxes: number[];
  player: number;
  parent: number;
  from: number;
  to: number;
  g: number;
  key: string;
}

/** States spent across the whole run, so the 60 s wall is never near. */
let globalSpent = 0;

/**
 * The push-optimal solution of `level` (A* over pushes, first goal popped is
 * optimal: the matching heuristic is admissible and consistent), or null when the
 * search gives up or the level has no solution. `maxStates` caps the states kept.
 */
function solvePushOptimal(level: Level, maxStates: number): { pushes: number; moves: string } | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  if (solved(level.boxes)) return { pushes: 0, moves: '' };

  const distGoal = goals.map((g) => bfsDist(w, wall, g));
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const keyOf = (boxes: number[], low: number): string => low + ':' + boxes.join(',');

  const states: SState[] = [];
  const fScore: number[] = [];
  const best = new Map<string, number>();

  for (const b of level.boxes) occupied[b] = 1;
  const low0 = regionLow(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;
  const h0 = matchingCost(level.boxes, distGoal);
  if (h0 === Infinity) return null;
  const key0 = keyOf(level.boxes, low0);
  states.push({ boxes: level.boxes.slice(), player: level.player, parent: -1, from: -1, to: -1, g: 0, key: key0 });
  fScore.push(h0);
  best.set(key0, 0);
  globalSpent++;

  const heap: number[] = [];
  const less = (a: number, b: number): boolean => {
    if (fScore[a] !== fScore[b]) return fScore[a] < fScore[b];
    return states[a].g > states[b].g;
  };
  const heapPush = (s: number): void => {
    let i = heap.length;
    heap.push(s);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };
  heapPush(0);

  let found = -1;
  while (heap.length > 0 && found < 0) {
    const s = heapPop();
    const st = states[s];
    if ((best.get(st.key) ?? Infinity) < st.g) continue;
    if (solved(st.boxes)) {
      found = s;
      break;
    }
    if (states.length >= maxStates || globalSpent >= GLOBAL_STATES) return null;
    for (const b of st.boxes) occupied[b] = 1;
    const here = ++stamp;
    regionLow(level, occupied, st.player, seen, here, stack);
    for (let bi = 0; bi < st.boxes.length && found < 0; bi++) {
      const b = st.boxes[bi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const from = b - d;
        const t = b + d;
        if (seen[from] !== here) continue;
        if (wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = '';
        let next: number[] | null = null;
        let h = Infinity;
        if (!frozen) {
          next = st.boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          const low = regionLow(level, occupied, b, mark, ++stamp, stack);
          key = keyOf(next, low);
          h = matchingCost(next, distGoal);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || h === Infinity || next === null) continue;
        const g2 = st.g + 1;
        const prevBest = best.get(key);
        if (prevBest !== undefined && prevBest <= g2) continue;
        if (states.length >= maxStates || globalSpent >= GLOBAL_STATES) return null;
        best.set(key, g2);
        globalSpent++;
        const idx = states.length;
        states.push({ boxes: next, player: b, parent: s, from: b, to: t, g: g2, key });
        fScore.push(g2 + h);
        heapPush(idx);
      }
    }
    for (const b of st.boxes) occupied[b] = 0;
  }
  if (found < 0) return null;
  const path: number[] = [];
  for (let s = found; s > 0; s = states[s].parent) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes.slice();
  for (const s of path) {
    const nextBoxes = states[s].boxes;
    const from = states[s].from;
    const to = states[s].to;
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * w + dx === d)!;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += dir[3];
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

interface Room {
  squares: number[];
  boxes: number[];
  goals: number[];
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway: a
 * box there reaches no goal) out of the floor, and each part left that holds a
 * box is a room. Null when some part holds more or fewer boxes than goals, or a
 * box starts in the hallway: then the level is no set of rooms.
 */
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i] + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) if (part[g] >= 0) rooms[part[g]].goals.push(g);
  if (goals.some((g) => part[g] < 0)) return null;
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/**
 * The level solved room by room: each room's push-optimal solution in turn, the
 * other rooms' boxes standing as walls, the player walking on from where the last
 * room left it. A box never leaves its room, so the rooms' optima add up to the
 * level's. Null when a room's search gives up, or the level is no set of rooms
 * and the search over the whole level gives up.
 */
function solveByRooms(level: Level, maxStates: number): { pushes: number; moves: string } | null {
  const rooms = roomsOf(level);
  if (rooms === null) return solvePushOptimal(level, maxStates);
  let player = level.player;
  let pushes = 0;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of other.boxes) wall[b] = 1;
    }
    const sub: Level = {
      w: level.w,
      h: level.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const solved = solvePushOptimal(sub, maxStates);
    if (solved === null) return null;
    pushes += solved.pushes;
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    room.boxes = room.goals.slice();
  }
  return { pushes, moves };
}

/** The moves for a level, or '' when it cannot be solved (an empty answer is valid, worth no points). */
function solve(text: string): string {
  try {
    const level = parseLevel(text);
    const solved = solveByRooms(level, MAX_STATES);
    return solved === null ? '' : solved.moves;
  } catch {
    return '';
  }
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
