import type { Obstacle, PlannerSettings, SimPath, Waypoint } from './types';

export interface CollisionInfo {
  collides: boolean;
  sampleIndex: number;
  t: number;
  x: number;
  y: number;
  heading: number;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

export function normAngle(deg: number): number {
  let out = deg;
  while (out > 180) out -= 360;
  while (out < -180) out += 360;
  return out;
}

export function catmullRom(p0: Waypoint, p1: Waypoint, p2: Waypoint, p3: Waypoint, t: number): { x: number; y: number } {
  const t2 = t * t;
  const t3 = t2 * t;
  return {
    x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
    y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
  };
}

function travelTimeForDistance(dist: number, maxVel: number, maxAccel: number, maxDecel: number): number {
  if (dist <= 0) return 0;
  const dA = (maxVel * maxVel) / (2 * maxAccel);
  const dD = (maxVel * maxVel) / (2 * maxDecel);

  if (dist > dA + dD) {
    const cruise = dist - dA - dD;
    return maxVel / maxAccel + cruise / maxVel + maxVel / maxDecel;
  }

  const vPeak = Math.sqrt((2 * dist * maxAccel * maxDecel) / (maxAccel + maxDecel));
  return vPeak / maxAccel + vPeak / maxDecel;
}

function timeAtDistance(distance: number, totalDist: number, maxVel: number, maxAccel: number, maxDecel: number): number {
  if (totalDist <= 0) return 0;

  const dA = (maxVel * maxVel) / (2 * maxAccel);
  const dD = (maxVel * maxVel) / (2 * maxDecel);

  if (totalDist > dA + dD) {
    const tA = maxVel / maxAccel;
    const cruiseDist = totalDist - dA - dD;
    const total = travelTimeForDistance(totalDist, maxVel, maxAccel, maxDecel);

    if (distance <= dA) return Math.sqrt((2 * distance) / maxAccel);
    if (distance <= dA + cruiseDist) return tA + (distance - dA) / maxVel;

    const remaining = totalDist - distance;
    return total - Math.sqrt((2 * remaining) / maxDecel);
  }

  const vPeak = Math.sqrt((2 * totalDist * maxAccel * maxDecel) / (maxAccel + maxDecel));
  const dAccel = (vPeak * vPeak) / (2 * maxAccel);
  const total = vPeak / maxAccel + vPeak / maxDecel;

  if (distance <= dAccel) return Math.sqrt((2 * distance) / maxAccel);

  const remaining = totalDist - distance;
  return total - Math.sqrt((2 * remaining) / maxDecel);
}

function applyTurnRateLimit(samples: Array<{ x: number; y: number; heading: number; t: number }>, maxTurnRateDegPerSec: number): void {
  if (samples.length < 2 || maxTurnRateDegPerSec <= 0) return;

  for (let i = 1; i < samples.length; i++) {
    const dt = Math.max(0.001, samples[i].t - samples[i - 1].t);
    const maxDelta = maxTurnRateDegPerSec * dt;
    const desiredDelta = normAngle(samples[i].heading - samples[i - 1].heading);
    const limitedDelta = clamp(desiredDelta, -maxDelta, maxDelta);
    samples[i].heading = normAngle(samples[i - 1].heading + limitedDelta);
  }
}

export function buildSimPath(waypoints: Waypoint[], settings: PlannerSettings): SimPath | null {
  if (waypoints.length < 2) return null;

  const samples: Array<{ x: number; y: number; heading: number; t: number }> = [];
  const steps = 80;

  for (let i = 0; i < waypoints.length - 1; i++) {
    const p0 = waypoints[Math.max(0, i - 1)];
    const p1 = waypoints[i];
    const p2 = waypoints[i + 1];
    const p3 = waypoints[Math.min(waypoints.length - 1, i + 2)];

    for (let s = 0; s < steps; s++) {
      const t = s / steps;
      const pos = catmullRom(p0, p1, p2, p3, t);
      const h0 = normAngle(p1.heading);
      const h1 = normAngle(p2.heading);
      let dh = h1 - h0;
      if (dh > 180) dh -= 360;
      if (dh < -180) dh += 360;
      samples.push({ x: pos.x, y: pos.y, heading: h0 + dh * t, t: 0 });
    }
  }

  const last = waypoints[waypoints.length - 1];
  samples.push({ x: last.x, y: last.y, heading: last.heading, t: 0 });

  let totalDist = 0;
  const dists = [0];
  for (let i = 1; i < samples.length; i++) {
    totalDist += Math.hypot(samples[i].x - samples[i - 1].x, samples[i].y - samples[i - 1].y);
    dists.push(totalDist);
  }

  const maxVel = Math.max(1, settings.maxVel);
  const maxAccel = Math.max(1, settings.maxAccel);
  const maxDecel = Math.max(1, settings.maxDecel);
  const totalTime = Math.max(0.01, travelTimeForDistance(totalDist, maxVel, maxAccel, maxDecel));

  for (let i = 0; i < samples.length; i++) {
    samples[i].t = timeAtDistance(dists[i], totalDist, maxVel, maxAccel, maxDecel);
  }

  applyTurnRateLimit(samples, settings.maxTurnRate);

  return { samples, totalTime, totalDist };
}

export function sampleAtTime(path: SimPath, t: number): { x: number; y: number; heading: number; t: number } | null {
  const s = path.samples;
  if (!s.length) return null;
  if (t <= s[0].t) return s[0];
  if (t >= s[s.length - 1].t) return s[s.length - 1];

  let lo = 0;
  let hi = s.length - 1;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (s[m].t <= t) lo = m;
    else hi = m;
  }

  const a = s[lo];
  const b = s[hi];
  const f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
  let dh = b.heading - a.heading;
  if (dh > 180) dh -= 360;
  if (dh < -180) dh += 360;

  return {
    x: a.x + (b.x - a.x) * f,
    y: a.y + (b.y - a.y) * f,
    heading: normAngle(a.heading + dh * f),
    t,
  };
}

export function checkPathCollision(
  path: SimPath | null,
  obstacles: Obstacle[],
  robotWidth: number,
  robotLength: number,
  globalZeroAngleRad: number,
  userToImg: (x: number, y: number) => { imgX: number; imgY: number },
): boolean {
  return getFirstPathCollision(path, obstacles, robotWidth, robotLength, globalZeroAngleRad, userToImg).collides;
}

export function getFirstPathCollision(
  path: SimPath | null,
  obstacles: Obstacle[],
  robotWidth: number,
  robotLength: number,
  globalZeroAngleRad: number,
  userToImg: (x: number, y: number) => { imgX: number; imgY: number },
): CollisionInfo {
  if (!path || path.samples.length === 0) {
    return { collides: false, sampleIndex: -1, t: 0, x: 0, y: 0, heading: 0 };
  }

  const blocked = obstacles.filter((o) => o.blocked);
  if (!blocked.length) {
    return { collides: false, sampleIndex: -1, t: 0, x: 0, y: 0, heading: 0 };
  }

  const halfW = robotWidth / 2;
  const halfL = robotLength / 2;

  // Subsample points to keep this cheap while still respecting curved spline motion.
  const stride = Math.max(1, Math.floor(path.samples.length / 250));

  for (let i = 0; i < path.samples.length; i += stride) {
    const s = path.samples[i];
    const center = userToImg(s.x, s.y);
    const angle = globalZeroAngleRad - (s.heading * Math.PI) / 180 + Math.PI / 2;
    const robotPoly = makeRobotPolygon(center.imgX, center.imgY, halfW, halfL, angle);

    for (const obs of blocked) {
      const rx1 = obs.cx - obs.w / 2;
      const ry1 = obs.cy - obs.h / 2;
      const rx2 = obs.cx + obs.w / 2;
      const ry2 = obs.cy + obs.h / 2;
      const rectPoly = [
        { x: rx1, y: ry1 },
        { x: rx2, y: ry1 },
        { x: rx2, y: ry2 },
        { x: rx1, y: ry2 },
      ];
      if (polygonsIntersect(robotPoly, rectPoly)) {
        return { collides: true, sampleIndex: i, t: s.t, x: s.x, y: s.y, heading: s.heading };
      }
    }
  }

  // Ensure last sample is also checked even if stride skipped it.
  const last = path.samples[path.samples.length - 1];
  const lp = userToImg(last.x, last.y);
  const lastAngle = globalZeroAngleRad - (last.heading * Math.PI) / 180 + Math.PI / 2;
  const lastRobotPoly = makeRobotPolygon(lp.imgX, lp.imgY, halfW, halfL, lastAngle);
  for (const obs of blocked) {
    const rx1 = obs.cx - obs.w / 2;
    const ry1 = obs.cy - obs.h / 2;
    const rx2 = obs.cx + obs.w / 2;
    const ry2 = obs.cy + obs.h / 2;
    const rectPoly = [
      { x: rx1, y: ry1 },
      { x: rx2, y: ry1 },
      { x: rx2, y: ry2 },
      { x: rx1, y: ry2 },
    ];
    if (polygonsIntersect(lastRobotPoly, rectPoly)) {
      return {
        collides: true,
        sampleIndex: path.samples.length - 1,
        t: last.t,
        x: last.x,
        y: last.y,
        heading: last.heading,
      };
    }
  }

  return { collides: false, sampleIndex: -1, t: 0, x: 0, y: 0, heading: 0 };
}

type Point = { x: number; y: number };

function makeRobotPolygon(cx: number, cy: number, halfW: number, halfL: number, angle: number): Point[] {
  const local: Point[] = [
    { x: -halfW, y: -halfL },
    { x: halfW, y: -halfL },
    { x: halfW, y: halfL },
    { x: -halfW, y: halfL },
  ];
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return local.map((p) => ({
    x: cx + p.x * c - p.y * s,
    y: cy + p.x * s + p.y * c,
  }));
}

function polygonsIntersect(a: Point[], b: Point[]): boolean {
  const axes = [...polygonAxes(a), ...polygonAxes(b)];
  for (const axis of axes) {
    const projA = projectPolygon(a, axis);
    const projB = projectPolygon(b, axis);
    if (projA.max < projB.min || projB.max < projA.min) return false;
  }
  return true;
}

function polygonAxes(poly: Point[]): Point[] {
  const axes: Point[] = [];
  for (let i = 0; i < poly.length; i++) {
    const p1 = poly[i];
    const p2 = poly[(i + 1) % poly.length];
    const edge = { x: p2.x - p1.x, y: p2.y - p1.y };
    const normal = normalize({ x: -edge.y, y: edge.x });
    axes.push(normal);
  }
  return axes;
}

function normalize(v: Point): Point {
  const m = Math.hypot(v.x, v.y) || 1;
  return { x: v.x / m, y: v.y / m };
}

function projectPolygon(poly: Point[], axis: Point): { min: number; max: number } {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (const p of poly) {
    const d = p.x * axis.x + p.y * axis.y;
    if (d < min) min = d;
    if (d > max) max = d;
  }
  return { min, max };
}

export function checkSegBlocked(a: Waypoint, b: Waypoint, obstacles: Obstacle[], userToImg: (x: number, y: number) => { imgX: number; imgY: number }): boolean {
  const p1 = userToImg(a.x, a.y);
  const p2 = userToImg(b.x, b.y);

  for (const obs of obstacles) {
    if (!obs.blocked) continue;
    const rx1 = obs.cx - obs.w / 2;
    const ry1 = obs.cy - obs.h / 2;
    const rx2 = obs.cx + obs.w / 2;
    const ry2 = obs.cy + obs.h / 2;
    if (segRect(p1.imgX, p1.imgY, p2.imgX, p2.imgY, rx1, ry1, rx2, ry2)) return true;
  }

  return false;
}

function segRect(x1: number, y1: number, x2: number, y2: number, rx1: number, ry1: number, rx2: number, ry2: number): boolean {
  const dx = x2 - x1;
  const dy = y2 - y1;
  let t0 = 0;
  let t1 = 1;

  const edges: Array<[number, number]> = [
    [-dx, x1 - rx1],
    [dx, rx2 - x1],
    [-dy, y1 - ry1],
    [dy, ry2 - y1],
  ];

  for (const [p, q] of edges) {
    if (p === 0) {
      if (q < 0) return false;
      continue;
    }
    const r = q / p;
    if (p < 0) {
      if (r > t1) return false;
      if (r > t0) t0 = r;
    } else {
      if (r < t0) return false;
      if (r < t1) t1 = r;
    }
  }

  return t0 <= t1;
}
