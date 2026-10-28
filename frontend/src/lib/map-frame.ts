export type MapGeometry = {
  width_px: number;
  height_px: number;
  resolution: number;
  origin: { x: number; y: number; yaw: number };
};

export type GridPoint = { col: number; row: number };

/** Nav2 occupancy grid: row 0 is the image bottom, at the YAML origin. */
export function worldToGrid(geom: MapGeometry, x: number, y: number): GridPoint {
  const theta = geom.origin.yaw;
  const dx = x - geom.origin.x;
  const dy = y - geom.origin.y;
  const localX = dx * Math.cos(theta) + dy * Math.sin(theta);
  const localY = -dx * Math.sin(theta) + dy * Math.cos(theta);
  return { col: localX / geom.resolution, row: localY / geom.resolution };
}

export function gridToWorld(geom: MapGeometry, col: number, row: number): { x: number; y: number } {
  const theta = geom.origin.yaw;
  const localX = col * geom.resolution;
  const localY = row * geom.resolution;
  return {
    x: geom.origin.x + localX * Math.cos(theta) - localY * Math.sin(theta),
    y: geom.origin.y + localX * Math.sin(theta) + localY * Math.cos(theta),
  };
}

export function gridToPixel(geom: MapGeometry, col: number, row: number): { px: number; py: number } {
  return { px: col, py: geom.height_px - row };
}

export function worldToPixel(geom: MapGeometry, x: number, y: number): { px: number; py: number } {
  const grid = worldToGrid(geom, x, y);
  return gridToPixel(geom, grid.col, grid.row);
}

export function pixelToWorld(geom: MapGeometry, px: number, py: number): { x: number; y: number } {
  return gridToWorld(geom, px, geom.height_px - py);
}

export function insidePixel(geom: MapGeometry, px: number, py: number): boolean {
  return px >= 0 && py >= 0 && px <= geom.width_px && py <= geom.height_px;
}
