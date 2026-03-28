import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { DEFAULT_OBSTACLES, DEFAULT_SETTINGS, FIELD } from './plannerConfig';
import { buildPurePursuitPreviewPath, buildSimPath, getFirstPathCollision, normAngle, sampleAtTime } from './plannerMath';
import type { Alliance, Mode, Obstacle, PlannerSettings, TabKey, Waypoint } from './types';

type PathDisplayMode = 'catmull' | 'purePursuit';

const modeLabel: Record<Mode, string> = {
  add: '+ ADD WAYPOINT',
  drag: '↕ DRAG WAYPOINT',
  rotate: '↻ ROTATE HEADING',
  delete: '✕ DELETE WAYPOINT',
};

function fmt(n: number): string {
  return (Math.round(n * 100) / 100).toFixed(2);
}

function hDir(h: number): string {
  const n = ((h % 360) + 360) % 360;
  if (n < 22.5 || n >= 337.5) return '+X →';
  if (n < 67.5) return '↗';
  if (n < 112.5) return '+Y ↑';
  if (n < 157.5) return '↖';
  if (n < 202.5) return '-X ←';
  if (n < 247.5) return '↙';
  if (n < 292.5) return '-Y ↓';
  return '↘';
}

export default function App() {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [selectedWp, setSelectedWp] = useState(-1);
  const [mode, setMode] = useState<Mode>('add');
  const [tab, setTab] = useState<TabKey>('waypoints');
  const [alliance, setAlliance] = useState<Alliance>('blue');
  const [settings, setSettings] = useState<PlannerSettings>(DEFAULT_SETTINGS);
  const [obstacles, setObstacles] = useState<Obstacle[]>(DEFAULT_OBSTACLES);
  const [simT, setSimT] = useState(0);
  const [simPlaying, setSimPlaying] = useState(false);
  const [simSpeed, setSimSpeed] = useState(1);
  const [pathDisplayMode, setPathDisplayMode] = useState<PathDisplayMode>('purePursuit');
  const [purePursuitLookahead, setPurePursuitLookahead] = useState(10);
  const [mouseCoord, setMouseCoord] = useState({ x: 0, y: 0 });
  const [tooltip, setTooltip] = useState<{ show: boolean; text: string; x: number; y: number }>({ show: false, text: '', x: 0, y: 0 });

  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const tooltipRef = useRef<HTMLDivElement>(null);
  const scaleRef = useRef(1);
  const canvasSizeRef = useRef({ w: 0, h: 0 });
  const dragIndexRef = useRef(-1);
  const rotateIndexRef = useRef(-1);
  const simRafRef = useRef<number | null>(null);
  const simLastTsRef = useRef<number | null>(null);

  const simPath = useMemo(() => buildSimPath(waypoints, settings), [waypoints, settings]);
  const purePursuitPreviewPath = useMemo(
    () => buildPurePursuitPreviewPath(simPath, {
      lookahead: purePursuitLookahead,
      maxTurnRateDegPerSec: settings.maxTurnRate,
      maxVel: settings.maxVel,
      maxAccel: settings.maxAccel,
      maxDecel: settings.maxDecel,
    }),
    [purePursuitLookahead, settings.maxAccel, settings.maxDecel, settings.maxTurnRate, settings.maxVel, simPath],
  );
  const activePath = useMemo(
    () => (pathDisplayMode === 'purePursuit' ? purePursuitPreviewPath : simPath),
    [pathDisplayMode, purePursuitPreviewPath, simPath],
  );

  const userToImg = useCallback((ux: number, uy: number): { imgX: number; imgY: number } => ({ imgX: FIELD.width - uy, imgY: ux }), []);
  const imgToUser = useCallback((ix: number, iy: number): { x: number; y: number } => ({ x: iy, y: FIELD.width - ix }), []);

  const userToCanvas = useCallback((ux: number, uy: number): { cx: number; cy: number } => {
    const { imgX, imgY } = userToImg(ux, uy);
    return { cx: imgX * scaleRef.current, cy: imgY * scaleRef.current };
  }, [userToImg]);

  const getGlobalZeroAngle = useCallback((): number => {
    const hub = obstacles.find((o) => o.id === 'hub_red') ?? obstacles[0];
    const s = scaleRef.current;
    const { w, h } = canvasSizeRef.current;
    return Math.atan2(hub.cy * s - h / 2, hub.cx * s - w / 2);
  }, [obstacles]);

  const collisionInfo = useMemo(
    () => getFirstPathCollision(activePath, obstacles, settings.robotW, settings.robotL, getGlobalZeroAngle(), userToImg),
    [activePath, getGlobalZeroAngle, obstacles, settings.robotL, settings.robotW, userToImg],
  );
  const collision = collisionInfo.collides;

  const canvasToUser = useCallback((px: number, py: number): { x: number; y: number } => {
    let { x, y } = imgToUser(px / scaleRef.current, py / scaleRef.current);
    if (settings.snap > 0) {
      x = Math.round(x / settings.snap) * settings.snap;
      y = Math.round(y / settings.snap) * settings.snap;
    }
    return { x, y };
  }, [imgToUser, settings.snap]);

  const getCanvasEventCoord = (event: React.MouseEvent<HTMLCanvasElement>): { x: number; y: number } => {
    const canvas = canvasRef.current;
    if (!canvas) return { x: 0, y: 0 };
    const r = canvas.getBoundingClientRect();
    return canvasToUser((event.clientX - r.left) * (canvas.width / r.width), (event.clientY - r.top) * (canvas.height / r.height));
  };

  const nearestWp = (ux: number, uy: number, threshold = 20 / scaleRef.current): number => {
    let best = -1;
    let bestDist = threshold;
    for (let i = 0; i < waypoints.length; i++) {
      const d = Math.hypot(waypoints[i].x - ux, waypoints[i].y - uy);
      if (d < bestDist) {
        best = i;
        bestDist = d;
      }
    }
    return best;
  };

  const calcHeadingFromTo = (from: Waypoint, to: { x: number; y: number }): number => {
    const a = userToCanvas(from.x, from.y);
    const b = userToCanvas(to.x, to.y);
    const vec = Math.atan2(b.cy - a.cy, b.cx - a.cx);
    return normAngle((vec - getGlobalZeroAngle()) * 180 / Math.PI);
  };

  const parseAndImportWaypoints = (text: string): Waypoint[] => {
    const parsed: Waypoint[] = [];
    const lines = text.trim().split('\n').map(l => l.trim()).filter(l => l);
    
    for (const line of lines) {
      // Skip comments and empty lines
      if (line.startsWith('//') || line.startsWith('#') || !line) continue;
      
      // Try JSON format first [{\"x\": 1, \"y\": 2, \"heading\": 0}, ...]
      if (line.startsWith('{') || line.startsWith('[')) {
        try {
          const obj = JSON.parse(line);
          if (Array.isArray(obj)) {
            for (const item of obj) {
              if (item.x !== undefined && item.y !== undefined) {
                parsed.push({ x: -item.x, y: item.y, heading: item.heading ?? 0 });
              }
            }
          } else if (obj.x !== undefined && obj.y !== undefined) {
            parsed.push({ x: -obj.x, y: obj.y, heading: obj.heading ?? 0 });
          }
          continue;
        } catch (e) {}
      }
      
      // Try TrcPose2D format: new TrcPose2D(x, y, heading)
      const trcMatch = line.match(/TrcPose2D\s*\(\s*([\d.-]+)\s*,\s*([\d.-]+)\s*,\s*([\d.-]+)\s*\)/);
      if (trcMatch) {
        const [, xStr, yStr, hStr] = trcMatch;
        parsed.push({ x: -Number(xStr), y: Number(yStr), heading: Number(hStr) });
        continue;
      }
      
      // Try CSV/space/comma separated: x,y,heading or x y heading
      const parts = line.replace(/[,\s]+/g, ' ').split(' ').filter(p => p);
      if (parts.length >= 2) {
        const x = Number(parts[0]);
        const y = Number(parts[1]);
        const h = parts.length >= 3 ? Number(parts[2]) : 0;
        if (!Number.isNaN(x) && !Number.isNaN(y)) {
          parsed.push({ x: -x, y, heading: h });
          continue;
        }
      }
    }
    
    return parsed;
  };

  const generateJava = (): string => {
    if (!waypoints.length) return '// No waypoints yet';
    const s = waypoints[0];
    const e = waypoints[waypoints.length - 1];
    const lines: string[] = [
      '// FRC 2026 REBUILT - Auto Path',
      `// Alliance: ${alliance.toUpperCase()} | WPs: ${waypoints.length} | theta CW+, 0°=toward RED HUB`,
      `// Limits: v=${fmt(settings.maxVel)} in/s, a=${fmt(settings.maxAccel)} in/s^2, decel=${fmt(settings.maxDecel)} in/s^2, turn=${fmt(settings.maxTurnRate)} deg/s`,
      '',
      `TrcPose2D startPose = new TrcPose2D(${fmt(-s.x)}, ${fmt(s.y)}, ${fmt(s.heading)});`,
      `TrcPose2D endPose   = new TrcPose2D(${fmt(-e.x)}, ${fmt(e.y)}, ${fmt(e.heading)});`,
    ];
    if (waypoints.length > 2) {
      lines.push('');
      lines.push('// Intermediate waypoints');
      waypoints.slice(1, -1).forEach((wp, i) => {
        lines.push(`TrcPose2D wp${i + 1} = new TrcPose2D(${fmt(-wp.x)}, ${fmt(wp.y)}, ${fmt(wp.heading)});`);
      });
    }
    lines.push('');
    lines.push('// Pure Pursuit path array');
    if (waypoints.length === 2) {
      lines.push('TrcPose2D[] path = new TrcPose2D[] { startPose, endPose };');
    } else {
      const mids = waypoints.slice(1, -1).map((_, i) => `wp${i + 1}`).join(', ');
      lines.push('TrcPose2D[] path = new TrcPose2D[] {');
      lines.push(`    startPose, ${mids}, endPose`);
      lines.push('};');
    }
    return lines.join('\n');
  };

  useEffect(() => {
    const resizeCanvas = (): void => {
      const wrap = wrapRef.current;
      const canvas = canvasRef.current;
      if (!wrap || !canvas) return;
      const maxW = wrap.clientWidth - 16;
      const maxH = wrap.clientHeight - 8;
      const scale = Math.min(maxW / FIELD.width, maxH / FIELD.height);
      scaleRef.current = scale;
      const w = Math.round(FIELD.width * scale);
      const h = Math.round(FIELD.height * scale);
      canvas.width = w;
      canvas.height = h;
      canvas.style.width = `${w}px`;
      canvas.style.height = `${h}px`;
      canvasSizeRef.current = { w, h };
    };

    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);
    return () => window.removeEventListener('resize', resizeCanvas);
  }, []);

  useEffect(() => {
    if (!tooltipRef.current) return;
    tooltipRef.current.style.left = `${tooltip.x}px`;
    tooltipRef.current.style.top = `${tooltip.y}px`;
  }, [tooltip.x, tooltip.y]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const { w: canvasW, h: canvasH } = canvasSizeRef.current;
    const scale = scaleRef.current;

    const drawGrid = (): void => {
      const g = 24 * scale;
      ctx.strokeStyle = 'rgba(255,255,255,.035)';
      ctx.lineWidth = 0.5;
      for (let x = 0; x < canvasW; x += g) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvasH);
        ctx.stroke();
      }
      for (let y = 0; y < canvasH; y += g) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvasW, y);
        ctx.stroke();
      }
    };

    const rrect = (x: number, y: number, ww: number, hh: number, r: number): void => {
      ctx.moveTo(x + r, y);
      ctx.lineTo(x + ww - r, y);
      ctx.quadraticCurveTo(x + ww, y, x + ww, y + r);
      ctx.lineTo(x + ww, y + hh - r);
      ctx.quadraticCurveTo(x + ww, y + hh, x + ww - r, y + hh);
      ctx.lineTo(x + r, y + hh);
      ctx.quadraticCurveTo(x, y + hh, x, y + hh - r);
      ctx.lineTo(x, y + r);
      ctx.quadraticCurveTo(x, y, x + r, y);
      ctx.closePath();
    };

    const drawHex = (cx: number, cy: number, r: number): void => {
      ctx.beginPath();
      for (let i = 0; i < 6; i++) {
        const a = (Math.PI / 3) * i - Math.PI / 6;
        const x = cx + r * Math.cos(a);
        const y = cy + r * Math.sin(a);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.closePath();
      ctx.stroke();
    };

    const ellipsizeToWidth = (text: string, maxWidth: number): string => {
      if (ctx.measureText(text).width <= maxWidth) return text;
      const ellipsis = '...';
      let out = text;
      while (out.length > 1 && ctx.measureText(`${out}${ellipsis}`).width > maxWidth) {
        out = out.slice(0, -1);
      }
      return `${out}${ellipsis}`;
    };

    const drawObstacleLabel = (label: string, cx: number, cy: number, ow: number, oh: number): boolean => {
      const maxWidth = Math.max(10, ow - 8);
      const minFont = 8.5;
      let fontPx = Math.max(minFont, Math.min(11, oh * 0.34));
      const maxLines = 3;

      // Extremely thin/tiny obstacles should use external labels to stay readable.
      if (maxWidth < 22 || oh < 12) return false;

      const words = label.split(/\s+/).filter(Boolean);

      const wrapLines = (): string[] => {
        const lines: string[] = [];
        if (!words.length) return lines;
        let line = words[0];
        for (let i = 1; i < words.length; i++) {
          const candidate = `${line} ${words[i]}`;
          if (ctx.measureText(candidate).width <= maxWidth) line = candidate;
          else {
            lines.push(line);
            line = words[i];
          }
        }
        lines.push(line);
        return lines;
      };

      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';

      let lines: string[] = [];
      while (fontPx >= minFont) {
        ctx.font = `500 ${fontPx}px 'JetBrains Mono', monospace`;
        lines = wrapLines();
        if (lines.length > maxLines) {
          fontPx -= 0.5;
          continue;
        }
        const lineHeight = fontPx + 1;
        const totalHeight = lines.length * lineHeight;
        const widest = lines.reduce((w, line) => Math.max(w, ctx.measureText(line).width), 0);
        if (widest <= maxWidth && totalHeight <= oh - 4) break;
        fontPx -= 0.5;
      }

      if (!lines.length) return false;

      // If still not fitting at minimum font, keep readability by truncating within max lines.
      ctx.font = `500 ${fontPx}px 'JetBrains Mono', monospace`;
      if (lines.length > maxLines) {
        lines = lines.slice(0, maxLines);
      }
      for (let i = 0; i < lines.length; i++) {
        lines[i] = ellipsizeToWidth(lines[i], maxWidth);
      }

      const lineHeight = fontPx + 1;
      const startY = cy - ((lines.length - 1) * lineHeight) / 2;
      for (let i = 0; i < lines.length; i++) {
        ctx.fillText(lines[i], cx, startY + i * lineHeight);
      }
      return true;
    };

    const drawExternalObstacleLabel = (obs: Obstacle, cx: number, cy: number, ow: number): void => {
      const padX = 6;
      const padY = 3;
      const fontPx = 9;
      const maxTextW = 96;
      ctx.font = `600 ${fontPx}px 'JetBrains Mono', monospace`;
      const text = ellipsizeToWidth(obs.label, maxTextW);
      const tw = ctx.measureText(text).width;
      const bw = tw + padX * 2;
      const bh = fontPx + padY * 2;

      let left = cx + ow / 2 + 6;
      if (left + bw > canvasW - 4) {
        left = cx - ow / 2 - 6 - bw;
      }
      const top = Math.max(4, Math.min(canvasH - bh - 4, cy - bh / 2));

      ctx.save();
      ctx.fillStyle = 'rgba(8,12,24,.88)';
      ctx.strokeStyle = 'rgba(255,255,255,.35)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      rrect(left, top, bw, bh, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = 'rgba(255,255,255,.85)';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(text, left + padX, top + bh / 2 + 0.5);
      ctx.restore();
    };

    const drawObstacle = (obs: Obstacle): void => {
      const cx = obs.cx * scale;
      const cy = obs.cy * scale;
      const ow = obs.w * scale;
      const oh = obs.h * scale;
      const colorWithAlpha = (color: string, alpha: number): string => {
        const a = Math.max(0, Math.min(1, alpha));
        const raw = color.trim();

        if (raw.startsWith('#')) {
          let hex = raw.slice(1);
          if (hex.length === 3 || hex.length === 4) {
            hex = hex.split('').map((ch) => ch + ch).join('');
          }
          if (hex.length === 6 || hex.length === 8) {
            const r = parseInt(hex.slice(0, 2), 16);
            const g = parseInt(hex.slice(2, 4), 16);
            const b = parseInt(hex.slice(4, 6), 16);
            if (![r, g, b].some(Number.isNaN)) {
              return `rgba(${r},${g},${b},${a})`;
            }
          }
        }

        const rgbMatch = raw.match(/^rgba?\(([^)]+)\)$/i);
        if (rgbMatch) {
          const parts = rgbMatch[1].split(',').map((v) => Number(v.trim()));
          const r = Math.max(0, Math.min(255, Math.round(parts[0] ?? 0)));
          const g = Math.max(0, Math.min(255, Math.round(parts[1] ?? 0)));
          const b = Math.max(0, Math.min(255, Math.round(parts[2] ?? 0)));
          return `rgba(${r},${g},${b},${a})`;
        }

        return color;
      };

      const drawRoundedRect = (radius: number): void => {
        ctx.beginPath();
        rrect(cx - ow / 2, cy - oh / 2, ow, oh, radius);
      };

      if (obs.category === 'hub') {
        ctx.save();
        ctx.globalAlpha = 0.15;
        ctx.fillStyle = obs.color;
        drawRoundedRect(4);
        ctx.fill();
        ctx.restore();

        ctx.strokeStyle = obs.color;
        ctx.lineWidth = 1.5;
        drawRoundedRect(4);
        ctx.stroke();

        ctx.save();
        ctx.globalAlpha = 0.55;
        ctx.strokeStyle = obs.color;
        ctx.lineWidth = 1;
        drawHex(cx, cy, ow * 0.37);
        ctx.restore();
      } else if (obs.category === 'bump') {
        ctx.save();
        ctx.globalAlpha = 0.16;
        ctx.fillStyle = obs.color;
        drawRoundedRect(3);
        ctx.fill();
        ctx.restore();

        ctx.save();
        ctx.globalAlpha = 0.45;
        ctx.strokeStyle = obs.color;
        ctx.lineWidth = 1;
        drawRoundedRect(3);
        ctx.stroke();
        ctx.restore();
      } else if (obs.category === 'trench') {
        ctx.save();
        ctx.globalAlpha = obs.blocked ? 0.32 : 0.18;
        ctx.fillStyle = obs.color;
        drawRoundedRect(2);
        ctx.fill();
        ctx.restore();

        ctx.save();
        ctx.globalAlpha = obs.blocked ? 0.85 : 0.6;
        ctx.strokeStyle = obs.color;
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        drawRoundedRect(2);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
      } else if (obs.category === 'depot') {
        ctx.save();
        ctx.globalAlpha = 0.8;
        ctx.fillStyle = obs.color;
        drawRoundedRect(3);
        ctx.fill();
        ctx.restore();

        ctx.strokeStyle = obs.color;
        ctx.lineWidth = 1.5;
        drawRoundedRect(3);
        ctx.stroke();
      } else if (obs.category === 'fuel') {
        ctx.fillStyle = colorWithAlpha(obs.color, 0.22);
        drawRoundedRect(3);
        ctx.fill();

        ctx.strokeStyle = colorWithAlpha(obs.color, 0.58);
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 5]);
        drawRoundedRect(3);
        ctx.stroke();
        ctx.setLineDash([]);

        const cols = 12;
        const rows = 20;
        const ballR = Math.max(1.5, scale * 0.9);
        const padX = 10;
        const padY = 8;
        const xStart = cx - ow / 2 + padX;
        const yStart = cy - oh / 2 + padY;
        const xSpacing = (ow - padX * 2) / Math.max(1, cols - 1);
        const ySpacing = (oh - padY * 2) / Math.max(1, rows - 1);
        for (let rr = 0; rr < rows; rr++) {
          for (let cc = 0; cc < cols; cc++) {
            const fx = xStart + cc * xSpacing;
            const fy = yStart + rr * ySpacing;
            ctx.fillStyle = colorWithAlpha(obs.color, 0.72);
            ctx.beginPath();
            ctx.arc(fx, fy, ballR, 0, Math.PI * 2);
            ctx.fill();
          }
        }
      } else {
        ctx.save();
        ctx.globalAlpha = obs.blocked ? 0.28 : 0.18;
        ctx.fillStyle = obs.color;
        drawRoundedRect(3);
        ctx.fill();
        ctx.restore();

        ctx.strokeStyle = obs.color;
        ctx.lineWidth = obs.blocked ? 1.5 : 1;
        ctx.globalAlpha = obs.blocked ? 0.8 : 0.55;
        drawRoundedRect(3);
        ctx.stroke();
        ctx.globalAlpha = 1;
      }

      ctx.save();
      ctx.beginPath();
      rrect(cx - ow / 2, cy - oh / 2, ow, oh, 3);
      ctx.clip();
      ctx.fillStyle = 'rgba(255,255,255,.65)';
      const drewInside = drawObstacleLabel(obs.label, cx, cy + 1, ow, oh);
      ctx.restore();

      if (!drewInside) {
        drawExternalObstacleLabel(obs, cx, cy, ow);
      }
    };

    const drawRobot = (x: number, y: number, headingDeg: number, alpha: number): void => {
      const p = userToCanvas(x, y);
      const rw = settings.robotW * scale;
      const rl = settings.robotL * scale;
      const angle = getGlobalZeroAngle() + headingDeg * Math.PI / 180;
      const c = alliance === 'blue' ? '74,158,255' : '255,90,90';
      ctx.save();
      ctx.translate(p.cx, p.cy);
      ctx.rotate(angle);
      ctx.fillStyle = `rgba(${c},${0.24 * alpha})`;
      ctx.strokeStyle = `rgba(${c},${0.8 * alpha})`;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      rrect(-rw / 2, -rl / 2, rw, rl, 3);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    };

    const drawCollisionRobot = (x: number, y: number, headingDeg: number): void => {
      const p = userToCanvas(x, y);
      const rw = settings.robotW * scale;
      const rl = settings.robotL * scale;
      const angle = getGlobalZeroAngle() + headingDeg * Math.PI / 180;

      ctx.save();
      ctx.translate(p.cx, p.cy);
      ctx.rotate(angle);
      ctx.shadowColor = 'rgba(255,90,90,.9)';
      ctx.shadowBlur = 16;
      ctx.fillStyle = 'rgba(255,90,90,.32)';
      ctx.strokeStyle = 'rgba(255,130,130,.95)';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      rrect(-rw / 2, -rl / 2, rw, rl, 3);
      ctx.fill();
      ctx.stroke();

      ctx.strokeStyle = 'rgba(255,255,255,.72)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(0, -rl * 0.45);
      ctx.stroke();
      ctx.restore();
    };

    const drawPath = (): void => {
      const renderPath = activePath;
      if (!renderPath || !renderPath.samples.length) return;

      ctx.beginPath();
      const first = userToCanvas(renderPath.samples[0].x, renderPath.samples[0].y);
      ctx.moveTo(first.cx, first.cy);
      for (let i = 1; i < renderPath.samples.length; i++) {
        const p = userToCanvas(renderPath.samples[i].x, renderPath.samples[i].y);
        ctx.lineTo(p.cx, p.cy);
      }
      const okColor = pathDisplayMode === 'purePursuit' ? 'rgba(74,158,255,.68)' : 'rgba(62,230,132,.55)';
      ctx.strokeStyle = collision ? 'rgba(255,80,80,.75)' : okColor;
      ctx.lineWidth = 2.5;
      ctx.setLineDash(collision ? [7, 4] : []);
      ctx.stroke();
      ctx.setLineDash([]);
    };

    ctx.clearRect(0, 0, canvasW, canvasH);
    ctx.fillStyle = '#0d1018';
    ctx.fillRect(0, 0, canvasW, canvasH);
    if (settings.showGrid) drawGrid();

    ctx.strokeStyle = 'rgba(255,255,255,.28)';
    ctx.lineWidth = 2;
    ctx.strokeRect(1, 1, canvasW - 2, canvasH - 2);

    obstacles.forEach(drawObstacle);

    drawPath();

    if (settings.showGhost && activePath && simT > 0.05) {
      for (let g = 6; g >= 1; g--) {
        const pose = sampleAtTime(activePath, simT * (g / 6));
        if (pose) drawRobot(pose.x, pose.y, pose.heading, (g / 6) * 0.8);
      }
    }

    if (activePath && simT > 0) {
      const pose = sampleAtTime(activePath, simT);
      if (pose) drawRobot(pose.x, pose.y, pose.heading, 1);
    }

    // Always render robot footprint previews at each waypoint.
    waypoints.forEach((wp) => drawRobot(wp.x, wp.y, wp.heading, 0.45));

    waypoints.forEach((wp, i) => {
      const p = userToCanvas(wp.x, wp.y);
      const isFirst = i === 0;
      const isLast = i === waypoints.length - 1;
      const isSelected = i === selectedWp;
      const color = isFirst ? '#3ee684' : isLast ? '#ff5a5a' : '#f7b731';
      const r = Math.max(7, scale * 4.2);
      const ang = getGlobalZeroAngle() + wp.heading * Math.PI / 180;

      ctx.strokeStyle = isSelected ? 'rgba(255,255,255,.65)' : 'rgba(255,255,255,.28)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(p.cx, p.cy);
      ctx.lineTo(p.cx + Math.cos(ang) * (r + 16), p.cy + Math.sin(ang) * (r + 16));
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.beginPath();
      ctx.arc(p.cx, p.cy, r, 0, Math.PI * 2);
      ctx.fillStyle = isSelected ? color : `${color}cc`;
      ctx.fill();
      ctx.strokeStyle = 'rgba(255,255,255,.4)';
      ctx.stroke();
    });

    if (collision) {
      drawCollisionRobot(collisionInfo.x, collisionInfo.y, collisionInfo.heading);
      const cp = userToCanvas(collisionInfo.x, collisionInfo.y);
      ctx.fillStyle = 'rgba(255,170,170,.95)';
      ctx.font = "600 10px 'JetBrains Mono', monospace";
      ctx.textAlign = 'left';
      ctx.fillText(`collision @ t=${fmt(collisionInfo.t)}s`, cp.cx + 10, cp.cy - 10);
    }

    if (collision) {
      // draw tiny collision badge indicator on canvas corner
      ctx.fillStyle = 'rgba(255,90,90,.82)';
      ctx.font = "600 11px 'JetBrains Mono', monospace";
      ctx.textAlign = 'left';
      ctx.fillText('COLLISION', 8, 14);
    }
  }, [activePath, alliance, collision, collisionInfo.heading, collisionInfo.t, collisionInfo.x, collisionInfo.y, getGlobalZeroAngle, obstacles, pathDisplayMode, selectedWp, settings, simT, userToCanvas, waypoints]);

  useEffect(() => {
    if (!simPlaying || !activePath) return;

    const frame = (ts: number): void => {
      if (!activePath) return;
      const prev = simLastTsRef.current;
      if (prev !== null) {
        const dt = (ts - prev) / 1000 * simSpeed;
        setSimT((old) => {
          const next = Math.min(old + dt, activePath.totalTime);
          if (next >= activePath.totalTime) setSimPlaying(false);
          return next;
        });
      }
      simLastTsRef.current = ts;
      simRafRef.current = requestAnimationFrame(frame);
    };

    simRafRef.current = requestAnimationFrame(frame);
    return () => {
      if (simRafRef.current) cancelAnimationFrame(simRafRef.current);
      simRafRef.current = null;
      simLastTsRef.current = null;
    };
  }, [activePath, simPlaying, simSpeed]);

  useEffect(() => {
    const onKey = (event: KeyboardEvent): void => {
      const target = event.target as HTMLElement | null;
      if (target?.tagName === 'INPUT') return;

      if (event.key === 'a' || event.key === 'A') setMode('add');
      if (event.key === 'd' || event.key === 'D') setMode('drag');
      if (event.key === 'r' || event.key === 'R') setMode('rotate');
      if (event.key === 'x' || event.key === 'X') setMode('delete');
      if (event.key === 'Escape') setSelectedWp(-1);
      if (event.key === 'Delete' && selectedWp >= 0) {
        setWaypoints((prev) => prev.filter((_, i) => i !== selectedWp));
        setSelectedWp(-1);
      }
      if (event.key === ' ' && activePath) {
        event.preventDefault();
        setSimPlaying((p) => !p);
      }
    };

    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [activePath, selectedWp]);

  const simPose = activePath ? sampleAtTime(activePath, simT) : null;

  const onCanvasDown = (event: React.MouseEvent<HTMLCanvasElement>): void => {
    const pos = getCanvasEventCoord(event);
    if (mode === 'add') {
      setWaypoints((prev) => {
        const heading = prev.length ? calcHeadingFromTo(prev[prev.length - 1], pos) : 0;
        return [...prev, { ...pos, heading, speed: 1.0, delay: 0.0 }];
      });
      setSelectedWp(waypoints.length);
      return;
    }
    const idx = nearestWp(pos.x, pos.y);
    if (idx < 0) return;
    setSelectedWp(idx);
    if (mode === 'drag') dragIndexRef.current = idx;
    if (mode === 'rotate') rotateIndexRef.current = idx;
    if (mode === 'delete') {
      setWaypoints((prev) => prev.filter((_, i) => i !== idx));
      setSelectedWp(-1);
    }
  };

  const onCanvasMove = (event: React.MouseEvent<HTMLCanvasElement>): void => {
    const pos = getCanvasEventCoord(event);
    setMouseCoord(pos);

    if (dragIndexRef.current >= 0 && mode === 'drag') {
      const idx = dragIndexRef.current;
      setWaypoints((prev) => prev.map((wp, i) => (i === idx ? { ...wp, x: pos.x, y: pos.y } : wp)));
    }

    if (rotateIndexRef.current >= 0 && mode === 'rotate') {
      const idx = rotateIndexRef.current;
      const wp = waypoints[idx];
      if (!wp) return;
      const p = userToCanvas(wp.x, wp.y);
      const canvas = canvasRef.current;
      if (!canvas) return;
      const r = canvas.getBoundingClientRect();
      const mpx = (event.clientX - r.left) * (canvas.width / r.width);
      const mpy = (event.clientY - r.top) * (canvas.height / r.height);
      const ca = Math.atan2(mpy - p.cy, mpx - p.cx);
      const h = normAngle((ca - getGlobalZeroAngle()) * 180 / Math.PI);
      setWaypoints((prev) => prev.map((item, i) => (i === idx ? { ...item, heading: h } : item)));
    }

    const idx = nearestWp(pos.x, pos.y, 28 / scaleRef.current);
    if (idx >= 0 && dragIndexRef.current < 0 && rotateIndexRef.current < 0) {
      const wp = waypoints[idx];
      setTooltip({
        show: true,
        x: event.clientX + 14,
        y: event.clientY - 10,
        text: `WP${idx}: (${fmt(wp.x)}", ${fmt(wp.y)}") theta=${Math.round(-wp.heading)}°`,
      });
    } else {
      setTooltip((t) => ({ ...t, show: false }));
    }
  };

  const onCanvasUp = (): void => {
    dragIndexRef.current = -1;
    rotateIndexRef.current = -1;
  };

  const deleteWp = (index: number): void => {
    setWaypoints((prev) => prev.filter((_, i) => i !== index));
    if (selectedWp === index) setSelectedWp(-1);
  };

  return (
    <div className="app-shell">
      <header>
        <div className="logo">FRC <span>2026 //</span> REBUILT Path Planner</div>
        <div className="alliance-toggle">
          <button
            className={alliance === 'blue' ? 'active blue' : ''}
            onClick={() => {
              if (alliance !== 'blue') {
                setWaypoints((prev) => prev.map((wp) => {
                  // Mirror in image space across field center, then convert back to user coords.
                  const img = userToImg(wp.x, wp.y);
                  const ix = FIELD.width - img.imgX;
                  const iy = FIELD.height - img.imgY;
                  const u = imgToUser(ix, iy);
                  return { ...wp, x: u.x, y: u.y, heading: normAngle(wp.heading + 180) };
                }));
              }
              setAlliance('blue');
            }}
          >
            BLUE
          </button>
          <button
            className={alliance === 'red' ? 'active red' : ''}
            onClick={() => {
              if (alliance !== 'red') {
                setWaypoints((prev) => prev.map((wp) => {
                  const img = userToImg(wp.x, wp.y);
                  const ix = FIELD.width - img.imgX;
                  const iy = FIELD.height - img.imgY;
                  const u = imgToUser(ix, iy);
                  return { ...wp, x: u.x, y: u.y, heading: normAngle(wp.heading + 180) };
                }));
              }
              setAlliance('red');
            }}
          >
            RED
          </button>
        </div>
        <div className={`mode-pill ${mode}`}>{modeLabel[mode]}</div>
        <div className="header-btns">
          {(['add', 'drag', 'rotate', 'delete'] as Mode[]).map((m) => (
            <button key={m} className={`btn ${mode === m ? m === 'drag' ? 'active-btn-drag' : m === 'rotate' ? 'active-btn-r' : m === 'delete' ? 'active-btn-del' : 'active-btn' : ''} ${m === 'delete' ? 'danger' : ''}`} onClick={() => setMode(m)}>
              {m[0].toUpperCase() + m.slice(1)}
            </button>
          ))}
          <button className="btn" onClick={() => { setWaypoints([]); setSelectedWp(-1); setSimT(0); }}>Clear</button>
          <button className="btn primary" onClick={() => navigator.clipboard.writeText(generateJava())}>Copy Array</button>
          <button className="btn" onClick={() => {
            setWaypoints((prev) => prev.map((wp) => {
              const img = userToImg(wp.x, wp.y);
              const ix = img.imgX; // keep horizontal image coord
              const iy = FIELD.height - img.imgY; // reflect vertically in image space
              const u = imgToUser(ix, iy);
              return { x: u.x, y: u.y, heading: normAngle(wp.heading + 180) };
            }));
            setSelectedWp(-1);
          }}>Flip Side</button>
        </div>
      </header>

      <div className="main">
        <div className="field-col">
          <div className="field-wrap" ref={wrapRef}>
            <canvas
              id="field-canvas"
              ref={canvasRef}
              onMouseDown={onCanvasDown}
              onMouseMove={onCanvasMove}
              onMouseUp={onCanvasUp}
              onMouseLeave={() => { onCanvasUp(); setTooltip((t) => ({ ...t, show: false })); }}
            />
          </div>

          <div className="sim-bar">
            <div className="sim-controls">
              <button className="sim-btn" onClick={() => { setSimPlaying(false); setSimT(0); }}>⏮</button>
              <button className={`sim-btn ${simPlaying ? 'playing' : ''}`} onClick={() => setSimPlaying((p) => !p)}>
                {simPlaying ? '⏸' : '▶'}
              </button>
              <button className="sim-btn" onClick={() => setSimT((t) => Math.min((activePath?.totalTime ?? 0), t + 0.1))}>⏭</button>
            </div>
            <input
              id="sim-scrubber"
              type="range"
              min={0}
              max={1000}
              step={1}
              title="Simulation timeline scrubber"
              value={activePath?.totalTime ? Math.round((simT / activePath.totalTime) * 1000) : 0}
              onChange={(event) => {
                if (!activePath?.totalTime) return;
                setSimT((Number(event.target.value) / 1000) * activePath.totalTime);
              }}
            />
            <div className="sim-time">t = <span>{fmt(simT)}</span>s / <span>{fmt(activePath?.totalTime ?? 0)}</span>s</div>
            <div className="sim-speed">
              ×
              <select title="Simulation speed" value={simSpeed} onChange={(event) => setSimSpeed(Number(event.target.value))}>
                <option value={0.25}>0.25</option>
                <option value={0.5}>0.5</option>
                <option value={1}>1</option>
                <option value={2}>2</option>
              </select>
            </div>
            <div className="sim-info">
              <span>{simPose ? `X=${fmt(simPose.x)}" Y=${fmt(simPose.y)}" θ=${fmt(-simPose.heading)}°` : 'place waypoints to simulate'}</span>
            </div>
          </div>

          <div className="coord-bar">
            <div>X=<span>{fmt(mouseCoord.x)}</span>" Y=<span>{fmt(mouseCoord.y)}</span>"</div>
            <div>WPs: <span>{waypoints.length}</span></div>
            <div>Dist: <span>{fmt(activePath?.totalDist ?? 0)}</span>"</div>
            {collision && <div>At: <span>{fmt(collisionInfo.t)}</span>s</div>}
            {collision && <div className="collision-badge visible">⚠ Collision</div>}
          </div>
        </div>

        <div className="sidebar">
          <div className="sidebar-tabs">
              <button title="Waypoints" className={tab === 'waypoints' ? 'active' : ''} onClick={() => setTab('waypoints')}>Wpts</button>
              <button title="Import" className={tab === 'import' ? 'active' : ''} onClick={() => setTab('import')}>Imp</button>
              <button title="Java" className={tab === 'output' ? 'active' : ''} onClick={() => setTab('output')}>Java</button>
              <button title="Settings" className={tab === 'settings' ? 'active' : ''} onClick={() => setTab('settings')}>Set</button>
              <button title="Obstacles" className={tab === 'obstacles' ? 'active' : ''} onClick={() => setTab('obstacles')}>Obs</button>
          </div>

          {tab === 'waypoints' && (
            <div className="sidebar-content">
              <div className="info-box">
                <strong>Origin:</strong> Blue outpost corner (top-right) - (0,0)<br />
                <strong>+X</strong> right and <strong>+Y</strong> forward<br />
                <strong>θ</strong> heading with max turn-rate limiting in simulation
              </div>
              {collision && <div className="collision-warn visible">⚠ Path crosses a blocked obstacle.</div>}
              <div className="section-hdr">Waypoints</div>
              <div id="wp-list">
                {!waypoints.length && <div className="empty-note">Click the field to place waypoints.</div>}
                {waypoints.map((wp, i) => (
                  <div key={i} className={`wp-item ${i === 0 ? 'start' : i === waypoints.length - 1 ? 'end' : 'mid'} ${selectedWp === i ? 'selected' : ''}`} onClick={() => setSelectedWp(i)}>
                    <div className="wp-label">{i === 0 ? 'Start' : i === waypoints.length - 1 ? 'End' : `Waypoint ${i}`}</div>
                    <div className="wp-coords">X=<span>{fmt(-wp.x)}"</span> Y=<span>{fmt(wp.y)}"</span></div>
                    <div className="heading-row">
                      <canvas
                        ref={(el) => {
                          if (!el) return;
                          el.width = 36;
                          el.height = 36;
                          const ctx2 = el.getContext('2d');
                          if (!ctx2) return;
                          const cx2 = el.width / 2, cy2 = el.height / 2, r2 = cx2 - 1.5;
                          ctx2.clearRect(0, 0, el.width, el.height);
                          ctx2.fillStyle = '#1f2333';
                          ctx2.strokeStyle = 'rgba(255,255,255,.14)';
                          ctx2.lineWidth = 1;
                          ctx2.beginPath();
                          ctx2.arc(cx2, cy2, r2, 0, Math.PI * 2);
                          ctx2.fill();
                          ctx2.stroke();
                          // Tick marks
                          for (let i = 0; i < 8; i++) {
                            const a = i * Math.PI / 4;
                            ctx2.strokeStyle = 'rgba(255,255,255,.18)';
                            ctx2.lineWidth = 0.8;
                            ctx2.beginPath();
                            ctx2.moveTo(cx2 + (r2 - 4) * Math.cos(a), cy2 + (r2 - 4) * Math.sin(a));
                            ctx2.lineTo(cx2 + (r2 - 1) * Math.cos(a), cy2 + (r2 - 1) * Math.sin(a));
                            ctx2.stroke();
                          }
                          // N tick
                          const nA = Math.PI;
                          ctx2.strokeStyle = 'rgba(62,230,132,.5)';
                          ctx2.lineWidth = 1;
                          ctx2.beginPath();
                          ctx2.moveTo(cx2 + (r2 - 5) * Math.cos(nA), cy2 + (r2 - 5) * Math.sin(nA));
                          ctx2.lineTo(cx2 + (r2 - 1) * Math.cos(nA), cy2 + (r2 - 1) * Math.sin(nA));
                          ctx2.stroke();
                          // Needle
                          const na = getGlobalZeroAngle() + wp.heading * Math.PI / 180;
                          ctx2.strokeStyle = '#a78bfa';
                          ctx2.lineWidth = 2;
                          ctx2.beginPath();
                          ctx2.moveTo(cx2, cy2);
                          ctx2.lineTo(cx2 + (r2 - 4) * Math.cos(na), cy2 + (r2 - 4) * Math.sin(na));
                          ctx2.stroke();
                          ctx2.fillStyle = '#a78bfa';
                          ctx2.beginPath();
                          ctx2.arc(cx2 + (r2 - 4) * Math.cos(na), cy2 + (r2 - 4) * Math.sin(na), 2.5, 0, Math.PI * 2);
                          ctx2.fill();
                          ctx2.fillStyle = 'rgba(255,255,255,.45)';
                          ctx2.beginPath();
                          ctx2.arc(cx2, cy2, 2, 0, Math.PI * 2);
                          ctx2.fill();
                        }}
                        className="dial-canvas"
                        title="Drag to rotate heading"
                        style={{ cursor: 'grab', borderRadius: '50%', display: 'block', flexShrink: 0, width: '36px', height: '36px' }}
                        onMouseDown={(event) => {
                          const el = event.currentTarget;
                          const r = el.getBoundingClientRect();
                          const startAngle = Math.atan2(event.clientY - (r.top + r.height / 2), event.clientX - (r.left + r.width / 2)) * 180 / Math.PI;
                          const startH = wp.heading;
                          const handleMouseMove = (e: MouseEvent) => {
                            const r2 = el.getBoundingClientRect();
                            const currentAngle = Math.atan2(e.clientY - (r2.top + r2.height / 2), e.clientX - (r2.left + r2.width / 2)) * 180 / Math.PI;
                            const delta = currentAngle - startAngle;
                            const newH = normAngle(startH + delta);
                            setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, heading: newH } : item)));
                          };
                          const handleMouseUp = () => {
                            window.removeEventListener('mousemove', handleMouseMove);
                            window.removeEventListener('mouseup', handleMouseUp);
                            el.style.cursor = 'grab';
                          };
                          el.style.cursor = 'grabbing';
                          window.addEventListener('mousemove', handleMouseMove);
                          window.addEventListener('mouseup', handleMouseUp);
                          event.stopPropagation();
                        }}
                      />
                      <input
                        key={`heading-${i}-${Math.round(wp.heading)}`}
                        className="heading-inp"
                        type="text"
                        title="Waypoint heading in degrees (e.g. -90, 0, 45)"
                        defaultValue={Math.round(wp.heading)}
                        onKeyDown={(event) => {
                          if (event.key === 'Enter') {
                            const text = (event.target as HTMLInputElement).value.trim();
                            if (text === '') {
                              setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, heading: 0 } : item)));
                              (event.target as HTMLInputElement).value = '0';
                            } else {
                              const next = Number(text);
                              if (!Number.isNaN(next)) {
                                const normalized = normAngle(next);
                                setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, heading: normalized } : item)));
                                (event.target as HTMLInputElement).value = String(Math.round(normalized));
                              } else {
                                (event.target as HTMLInputElement).value = String(Math.round(wp.heading));
                              }
                            }
                            (event.target as HTMLInputElement).blur();
                          }
                        }}
                        onBlur={(event) => {
                          const text = event.target.value.trim();
                          if (text === '') {
                            setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, heading: 0 } : item)));
                            event.target.value = '0';
                          } else {
                            const next = Number(text);
                            if (!Number.isNaN(next)) {
                              const normalized = normAngle(next);
                              setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, heading: normalized } : item)));
                              event.target.value = String(Math.round(normalized));
                            } else {
                              event.target.value = String(Math.round(wp.heading));
                            }
                          }
                        }}
                        onClick={(event) => event.stopPropagation()}
                      />
                      <div className="heading-meta">
                        <div>{Math.round(wp.heading)}°</div>
                        <div className="heading-meta-dir">{hDir(wp.heading)}</div>
                      </div>
                    </div>
                    <div className="form-row wp-extra" style={{ display: 'flex', gap: '8px', alignItems: 'center', marginTop: '6px' }}>
                      <div style={{ flex: 1 }}>
                        <label style={{ display: 'block', fontSize: '11px' }}>Power (0-1)</label>
                        <input
                          title="Per-waypoint power scalar (0.0 - 1.0). Multiplied by global max velocity."
                          type="number"
                          step={0.01}
                          min={0}
                          max={1}
                          defaultValue={wp.speed ?? 1.0}
                          onKeyDown={(event) => {
                            if (event.key !== 'Enter') return;
                            const txt = (event.target as HTMLInputElement).value.trim();
                            const raw = txt === '' ? undefined : Number(txt);
                            const val = raw === undefined || Number.isNaN(raw) ? 1.0 : Math.max(0, Math.min(1, raw));
                            setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, speed: val } : item)));
                            (event.target as HTMLInputElement).blur();
                          }}
                          onBlur={(event) => {
                            const txt = event.target.value.trim();
                            const raw = txt === '' ? undefined : Number(txt);
                            const val = raw === undefined || Number.isNaN(raw) ? 1.0 : Math.max(0, Math.min(1, raw));
                            setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, speed: val } : item)));
                          }}
                          onClick={(e) => e.stopPropagation()}
                        />
                      </div>
                      <div style={{ width: 86 }}>
                        <label style={{ display: 'block', fontSize: '11px' }}>Delay (s)</label>
                        <input
                          title="Delay to wait at this waypoint in seconds"
                          type="number"
                          step={0.1}
                          defaultValue={wp.delay ?? 0.0}
                          onKeyDown={(event) => {
                            if (event.key !== 'Enter') return;
                            const txt = (event.target as HTMLInputElement).value.trim();
                            const raw = txt === '' ? undefined : Number(txt);
                            const val = raw === undefined || Number.isNaN(raw) ? 0.0 : Math.max(0, raw);
                            setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, delay: val } : item)));
                            (event.target as HTMLInputElement).blur();
                          }}
                          onBlur={(event) => {
                            const txt = event.target.value.trim();
                            const raw = txt === '' ? undefined : Number(txt);
                            const val = raw === undefined || Number.isNaN(raw) ? 0.0 : Math.max(0, raw);
                            setWaypoints((prev) => prev.map((item, idx) => (idx === i ? { ...item, delay: val } : item)));
                          }}
                          onClick={(e) => e.stopPropagation()}
                        />
                      </div>
                    </div>
                    <button className="wp-del" onClick={(event) => { event.stopPropagation(); deleteWp(i); }}>×</button>
                  </div>
                ))}
              </div>
            </div>
          )}

          {tab === 'import' && (
            <div className="sidebar-content">
              <div className="section-hdr">Import Waypoints</div>
              <div className="info-box">
                Supports: JSON, TrcPose2D arrays, CSV, or space-separated values.
              </div>
              <textarea
                id="import-textarea"
                placeholder={`Paste waypoints here...\n\nFormats:\n[{"x":100,"y":200,"heading":45}]\nTrcPose2D(100,200,45)\n100 200 45`}
                style={{
                  width: '100%',
                  minHeight: '200px',
                  padding: '8px',
                  fontFamily: 'monospace',
                  fontSize: '10px',
                  border: '1px solid var(--border)',
                  borderRadius: '4px',
                  backgroundColor: 'var(--surface2)',
                  color: 'var(--text)',
                  resize: 'vertical',
                  marginBottom: '8px'
                }}
              />
              <div style={{ display: 'flex', gap: '6px', marginBottom: '8px' }}>
                <button
                  className="btn primary"
                  style={{ flex: 1 }}
                  onClick={() => {
                    const textarea = document.getElementById('import-textarea') as HTMLTextAreaElement;
                    if (textarea?.value.trim()) {
                      const imported = parseAndImportWaypoints(textarea.value);
                      if (imported.length > 0) {
                        setWaypoints(imported);
                        setSelectedWp(-1);
                        textarea.value = '';
                        setTab('waypoints');
                      }
                    }
                  }}
                >
                  Import
                </button>
                <button
                  className="btn"
                  style={{ flex: 1 }}
                  onClick={() => {
                    const textarea = document.getElementById('import-textarea') as HTMLTextAreaElement;
                    if (textarea) textarea.value = '';
                  }}
                >
                  Clear
                </button>
              </div>
              <div className="info-box">
                <strong>Format Examples:</strong><br />JSON:<br />[{"{"}x":-100,"y":200,"heading":45{"}"}]<br /><br />TrcPose2D:<br />TrcPose2D(-100,200,45)<br /><br />CSV/Space-separated:<br />-100 200 45
              </div>
            </div>
          )}

          {tab === 'output' && (
            <div className="sidebar-content">
              <div className="section-hdr">Java Array</div>
              <pre className="code-block">{generateJava()}</pre>
            </div>
          )}

          {tab === 'settings' && (
            <div className="sidebar-content">
              <div className="section-hdr">Robot</div>
              <div className="form-row"><label>Width (in)</label><input title="Robot width in inches" type="number" value={settings.robotW} onChange={(e) => setSettings((s) => ({ ...s, robotW: Number(e.target.value) }))} /></div>
              <div className="form-row"><label>Length (in)</label><input title="Robot length in inches" type="number" value={settings.robotL} onChange={(e) => setSettings((s) => ({ ...s, robotL: Number(e.target.value) }))} /></div>

              <div className="section-hdr">Motion Limits</div>
              <div className="form-row"><label>Max vel</label><input title="Max velocity in inches per second" type="number" value={settings.maxVel} onChange={(e) => setSettings((s) => ({ ...s, maxVel: Number(e.target.value) }))} /></div>
              <div className="form-row"><label>Max accel</label><input title="Max acceleration in inches per second squared" type="number" value={settings.maxAccel} onChange={(e) => setSettings((s) => ({ ...s, maxAccel: Number(e.target.value) }))} /></div>
              <div className="form-row"><label>Max decel</label><input title="Max deceleration in inches per second squared" type="number" value={settings.maxDecel} onChange={(e) => setSettings((s) => ({ ...s, maxDecel: Number(e.target.value) }))} /></div>
              <div className="form-row"><label>Turn rate °/s</label><input title="Max turn rate in degrees per second" type="number" value={settings.maxTurnRate} onChange={(e) => setSettings((s) => ({ ...s, maxTurnRate: Number(e.target.value) }))} /></div>

              <div className="section-hdr">Display</div>
              <div className="form-row"><label>Show grid</label><input title="Toggle field grid" type="checkbox" checked={settings.showGrid} onChange={(e) => setSettings((s) => ({ ...s, showGrid: e.target.checked }))} /></div>
              <div className="form-row"><label>Ghost trail</label><input title="Toggle robot ghost trail" type="checkbox" checked={settings.showGhost} onChange={(e) => setSettings((s) => ({ ...s, showGhost: e.target.checked }))} /></div>
              <div className="form-row">
                <label>Path line</label>
                <select title="Select displayed path line type" value={pathDisplayMode} onChange={(e) => setPathDisplayMode(e.target.value as PathDisplayMode)}>
                  <option value="catmull">Catmull reference</option>
                  <option value="purePursuit">Pure pursuit preview</option>
                </select>
              </div>
              <div className="form-row"><label>PP lookahead (in)</label><input title="Pure pursuit lookahead distance in inches" type="number" min={1} value={purePursuitLookahead} onChange={(e) => setPurePursuitLookahead(Math.max(1, Number(e.target.value) || 1))} /></div>
              <div className="form-row"><label>Snap (in)</label><input title="Waypoint snap size in inches" type="number" value={settings.snap} onChange={(e) => setSettings((s) => ({ ...s, snap: Number(e.target.value) }))} /></div>
            </div>
          )}

          {tab === 'obstacles' && (
            <div className="sidebar-content">
              <div className="section-hdr">Obstacle Config</div>
              <div className="info-box">Obstacle coordinates are now editable. Values are in field inches.</div>
              {obstacles.map((obs, i) => (
                <div key={obs.id} className="wp-item">
                  <div className="wp-label">{obs.label} ({obs.id})</div>
                  <div className="form-row"><label>X</label><input title="Obstacle center X" type="number" value={obs.cx} onChange={(e) => setObstacles((prev) => prev.map((o, idx) => idx === i ? { ...o, cx: Number(e.target.value) } : o))} /></div>
                  <div className="form-row"><label>Y</label><input title="Obstacle center Y" type="number" value={obs.cy} onChange={(e) => setObstacles((prev) => prev.map((o, idx) => idx === i ? { ...o, cy: Number(e.target.value) } : o))} /></div>
                  <div className="form-row"><label>W</label><input title="Obstacle width" type="number" value={obs.w} onChange={(e) => setObstacles((prev) => prev.map((o, idx) => idx === i ? { ...o, w: Number(e.target.value) } : o))} /></div>
                  <div className="form-row"><label>H</label><input title="Obstacle height" type="number" value={obs.h} onChange={(e) => setObstacles((prev) => prev.map((o, idx) => idx === i ? { ...o, h: Number(e.target.value) } : o))} /></div>
                  <div className="form-row"><label>Blocked</label><input title="Obstacle is blocking" type="checkbox" checked={obs.blocked} onChange={(e) => setObstacles((prev) => prev.map((o, idx) => idx === i ? { ...o, blocked: e.target.checked } : o))} /></div>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>

      {tooltip.show && (
        <div id="tooltip" ref={tooltipRef} className="visible">
          {tooltip.text}
        </div>
      )}
    </div>
  );
}
