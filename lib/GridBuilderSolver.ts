import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject, Rect } from "graphics-debug"
import type {
  BusObstacle,
  BusRouterInput,
  BusTerminalObstacleDetectionOutput,
  XYPoint,
} from "./IdentifyBusTerminalObstaclesSolver"

const DEFAULT_TRACE_WIDTH = 0.1
const ADJACENT_TRACE_SPACING = 0.15
const GRID_PADDING_IN_BUS_WIDTHS = 1

const GRID_OBSTACLE = 1 << 0
const GRID_START_AREA = 1 << 1
const GRID_END_AREA = 1 << 2

const GRID_OUTLINE_STROKE = "rgba(15, 23, 42, 0.1)"
const GRID_BORDER_STROKE = "rgba(15, 23, 42, 0.35)"
const GRID_OBSTACLE_FILL = "rgba(220, 38, 38, 0.38)"
const BUS_AREA_STROKE = "#2563eb"
const BUS_AREA_FILL = "rgba(37, 99, 235, 0.24)"
const SUMMARY_TEXT_COLOR = "#0f172a"

export const GridCellFlags = {
  obstacle: GRID_OBSTACLE,
  startArea: GRID_START_AREA,
  endArea: GRID_END_AREA,
} as const

export interface GridBuilderSolverInput {
  inputProblem: BusRouterInput
  terminalObstacles: BusTerminalObstacleDetectionOutput
}

export interface GridCellAddress {
  column: number
  row: number
  index: number
  center: XYPoint
}

export interface GridTerminalArea {
  obstacleIndices: number[]
  centroid: XYPoint
  centerCell: GridCellAddress
}

export interface GridBuilderOutput {
  traceCount: number
  traceWidth: number
  traceSpacing: number
  requiredBusWidth: number
  cellSize: number
  origin: XYPoint
  bounds: {
    minX: number
    maxX: number
    minY: number
    maxY: number
  }
  gridWidth: number
  gridHeight: number
  grid: Int32Array
  obstacleCellCount: number
  busConnectedObstacleIndices: number[]
  startArea: GridTerminalArea
  endArea: GridTerminalArea
}

const clamp = (value: number, min: number, max: number): number =>
  Math.min(max, Math.max(min, value))

const getTraceCount = (inputProblem: BusRouterInput): number =>
  Math.max(
    inputProblem.bus.connectionPatches.length,
    Math.floor(inputProblem.bus.pointIds.length / 2),
    1,
  )

const getObstacleBounds = (obstacles: BusObstacle[]) => {
  if (obstacles.length === 0) {
    return {
      minX: 0,
      maxX: 0,
      minY: 0,
      maxY: 0,
    }
  }

  let minX = Number.POSITIVE_INFINITY
  let maxX = Number.NEGATIVE_INFINITY
  let minY = Number.POSITIVE_INFINITY
  let maxY = Number.NEGATIVE_INFINITY

  for (const obstacle of obstacles) {
    const halfWidth = obstacle.width / 2
    const halfHeight = obstacle.height / 2

    minX = Math.min(minX, obstacle.center.x - halfWidth)
    maxX = Math.max(maxX, obstacle.center.x + halfWidth)
    minY = Math.min(minY, obstacle.center.y - halfHeight)
    maxY = Math.max(maxY, obstacle.center.y + halfHeight)
  }

  return { minX, maxX, minY, maxY }
}

export const getGridIndex = (
  column: number,
  row: number,
  gridWidth: number,
): number => row * gridWidth + column

export const getCellAddressFromPoint = (params: {
  point: XYPoint
  origin: XYPoint
  cellSize: number
  gridWidth: number
  gridHeight: number
}): GridCellAddress => {
  const column = clamp(
    Math.floor((params.point.x - params.origin.x) / params.cellSize),
    0,
    params.gridWidth - 1,
  )
  const row = clamp(
    Math.floor((params.point.y - params.origin.y) / params.cellSize),
    0,
    params.gridHeight - 1,
  )

  return {
    column,
    row,
    index: getGridIndex(column, row, params.gridWidth),
    center: {
      x: params.origin.x + (column + 0.5) * params.cellSize,
      y: params.origin.y + (row + 0.5) * params.cellSize,
    },
  }
}

const rasterizeObstacleIndices = (params: {
  flag: number
  grid: Int32Array
  obstacleIndices: number[]
  obstacles: BusObstacle[]
  origin: XYPoint
  cellSize: number
  gridWidth: number
  gridHeight: number
}) => {
  for (const obstacleIndex of params.obstacleIndices) {
    const obstacle = params.obstacles[obstacleIndex]

    if (!obstacle) {
      continue
    }

    const halfWidth = obstacle.width / 2
    const halfHeight = obstacle.height / 2
    const minColumn = clamp(
      Math.floor(
        (obstacle.center.x - halfWidth - params.origin.x) / params.cellSize,
      ),
      0,
      params.gridWidth - 1,
    )
    const maxColumn = clamp(
      Math.ceil(
        (obstacle.center.x + halfWidth - params.origin.x) / params.cellSize,
      ) - 1,
      0,
      params.gridWidth - 1,
    )
    const minRow = clamp(
      Math.floor(
        (obstacle.center.y - halfHeight - params.origin.y) / params.cellSize,
      ),
      0,
      params.gridHeight - 1,
    )
    const maxRow = clamp(
      Math.ceil(
        (obstacle.center.y + halfHeight - params.origin.y) / params.cellSize,
      ) - 1,
      0,
      params.gridHeight - 1,
    )

    for (let row = minRow; row <= maxRow; row += 1) {
      for (let column = minColumn; column <= maxColumn; column += 1) {
        const index = getGridIndex(column, row, params.gridWidth)
        params.grid[index] = params.grid[index]! | params.flag
      }
    }
  }
}

const createSummaryTexts = (
  bounds: GridBuilderOutput["bounds"],
  cellSize: number,
  lines: string[],
): NonNullable<GraphicsObject["texts"]> => {
  const texts: NonNullable<GraphicsObject["texts"]> = []

  for (let index = 0; index < lines.length; index += 1) {
    texts.push({
      x: bounds.minX,
      y: bounds.maxY + cellSize * (2 + index * 1.25),
      text: lines[index]!,
      fontSize: Math.max(4, cellSize * 0.55),
      color: SUMMARY_TEXT_COLOR,
    })
  }

  return texts
}

const createObstacleCellRects = (output: GridBuilderOutput): Rect[] => {
  const rects: Rect[] = []

  for (let row = 0; row < output.gridHeight; row += 1) {
    for (let column = 0; column < output.gridWidth; column += 1) {
      const cell = output.grid[getGridIndex(column, row, output.gridWidth)]!

      if ((cell & GridCellFlags.obstacle) === 0) {
        continue
      }

      rects.push({
        center: {
          x: output.origin.x + (column + 0.5) * output.cellSize,
          y: output.origin.y + (row + 0.5) * output.cellSize,
        },
        width: output.cellSize,
        height: output.cellSize,
        fill: GRID_OBSTACLE_FILL,
        stroke: GRID_OUTLINE_STROKE,
        label: "obstacle-cell",
      })
    }
  }

  return rects
}

const createAreaObstacleRects = (params: {
  obstacles: BusObstacle[]
  startAreaObstacleIndices: number[]
  endAreaObstacleIndices: number[]
}): Rect[] => {
  const rects: Rect[] = []

  for (const obstacleIndex of params.startAreaObstacleIndices) {
    const obstacle = params.obstacles[obstacleIndex]

    if (!obstacle) {
      continue
    }

    rects.push({
      center: obstacle.center,
      width: obstacle.width,
      height: obstacle.height,
      fill: BUS_AREA_FILL,
      stroke: BUS_AREA_STROKE,
      label: "bus-start-area-obstacle",
    })
  }

  for (const obstacleIndex of params.endAreaObstacleIndices) {
    const obstacle = params.obstacles[obstacleIndex]

    if (!obstacle) {
      continue
    }

    rects.push({
      center: obstacle.center,
      width: obstacle.width,
      height: obstacle.height,
      fill: BUS_AREA_FILL,
      stroke: BUS_AREA_STROKE,
      label: "bus-end-area-obstacle",
    })
  }

  return rects
}

const createAreaTexts = (
  output: GridBuilderOutput,
): NonNullable<GraphicsObject["texts"]> => [
  {
    x: output.startArea.centroid.x,
    y: output.startArea.centroid.y + output.cellSize * 1.2,
    text: "Bus Start Area",
    fontSize: Math.max(4, output.cellSize * 0.7),
    color: BUS_AREA_STROKE,
  },
  {
    x: output.endArea.centroid.x,
    y: output.endArea.centroid.y + output.cellSize * 1.2,
    text: "Bus End Area",
    fontSize: Math.max(4, output.cellSize * 0.7),
    color: BUS_AREA_STROKE,
  },
]

const createGridBorderRect = (output: GridBuilderOutput): Rect => ({
  center: {
    x: (output.bounds.minX + output.bounds.maxX) / 2,
    y: (output.bounds.minY + output.bounds.maxY) / 2,
  },
  width: output.gridWidth * output.cellSize,
  height: output.gridHeight * output.cellSize,
  stroke: GRID_BORDER_STROKE,
  label: "grid-border",
})

export const createGridBuilderVisualization = (params: {
  inputProblem: BusRouterInput
  output: GridBuilderOutput
  title: string
  extraLines?: NonNullable<GraphicsObject["lines"]>
  extraRects?: NonNullable<GraphicsObject["rects"]>
  extraCircles?: NonNullable<GraphicsObject["circles"]>
  extraTexts?: NonNullable<GraphicsObject["texts"]>
  extraPoints?: NonNullable<GraphicsObject["points"]>
}): GraphicsObject => {
  const summaryLines = [
    `Trace count: ${params.output.traceCount}`,
    `Required bus width: ${params.output.requiredBusWidth.toFixed(3)}`,
    `Grid cell size: ${params.output.cellSize.toFixed(3)}`,
    `Grid size: ${params.output.gridWidth} x ${params.output.gridHeight}`,
    `Obstacle cells: ${params.output.obstacleCellCount}`,
    `Bus-connected obstacles: ${params.output.busConnectedObstacleIndices.length}`,
  ]
  const texts = [
    ...createSummaryTexts(
      params.output.bounds,
      params.output.cellSize,
      summaryLines,
    ),
    ...createAreaTexts(params.output),
    ...(params.extraTexts ?? []),
  ]

  return {
    title: params.title,
    coordinateSystem: "cartesian",
    points: params.extraPoints ?? [],
    lines: params.extraLines ?? [],
    rects: [
      ...createObstacleCellRects(params.output),
      ...createAreaObstacleRects({
        obstacles: params.inputProblem.obstacles,
        startAreaObstacleIndices: params.output.startArea.obstacleIndices,
        endAreaObstacleIndices: params.output.endArea.obstacleIndices,
      }),
      createGridBorderRect(params.output),
      ...(params.extraRects ?? []),
    ],
    circles: params.extraCircles ?? [],
    texts,
  }
}

const computeRequiredBusWidth = (
  traceCount: number,
  traceWidth: number,
): number =>
  traceCount * traceWidth + Math.max(0, traceCount - 1) * ADJACENT_TRACE_SPACING

export class GridBuilderSolver extends BaseSolver {
  private output: GridBuilderOutput | null = null

  constructor(private readonly params: GridBuilderSolverInput) {
    super()
    const traceCount = getTraceCount(params.inputProblem)
    const traceWidth = params.inputProblem.traceWidth ?? DEFAULT_TRACE_WIDTH
    const requiredBusWidth = computeRequiredBusWidth(traceCount, traceWidth)

    this.MAX_ITERATIONS = 1
    this.stats = {
      phase: "build-grid",
      traceCount,
      traceWidth,
      traceSpacing: ADJACENT_TRACE_SPACING,
      requiredBusWidth,
    }
  }

  override _step() {
    const traceCount = getTraceCount(this.params.inputProblem)
    const traceWidth =
      this.params.inputProblem.traceWidth ?? DEFAULT_TRACE_WIDTH
    const requiredBusWidth = computeRequiredBusWidth(traceCount, traceWidth)
    const cellSize = requiredBusWidth / 2
    const obstacleBounds = getObstacleBounds(this.params.inputProblem.obstacles)
    const padding = requiredBusWidth * GRID_PADDING_IN_BUS_WIDTHS
    const origin = {
      x: obstacleBounds.minX - padding,
      y: obstacleBounds.minY - padding,
    }
    const gridWidth = Math.max(
      1,
      Math.ceil(
        (obstacleBounds.maxX - obstacleBounds.minX + padding * 2) / cellSize,
      ),
    )
    const gridHeight = Math.max(
      1,
      Math.ceil(
        (obstacleBounds.maxY - obstacleBounds.minY + padding * 2) / cellSize,
      ),
    )
    const bounds = {
      minX: origin.x,
      maxX: origin.x + gridWidth * cellSize,
      minY: origin.y,
      maxY: origin.y + gridHeight * cellSize,
    }
    const grid = new Int32Array(gridWidth * gridHeight)
    const busConnectedObstacleIndices =
      this.params.terminalObstacles.candidateObstacleIndices
    const busConnectedObstacleIndexSet = new Set(busConnectedObstacleIndices)
    let obstacleCellCount = 0

    // Rasterize only non-bus obstacles into the blocking occupancy grid.
    for (
      let obstacleIndex = 0;
      obstacleIndex < this.params.inputProblem.obstacles.length;
      obstacleIndex += 1
    ) {
      if (busConnectedObstacleIndexSet.has(obstacleIndex)) {
        continue
      }

      const obstacle = this.params.inputProblem.obstacles[obstacleIndex]!
      const halfWidth = obstacle.width / 2
      const halfHeight = obstacle.height / 2
      const minColumn = clamp(
        Math.floor((obstacle.center.x - halfWidth - origin.x) / cellSize),
        0,
        gridWidth - 1,
      )
      const maxColumn = clamp(
        Math.ceil((obstacle.center.x + halfWidth - origin.x) / cellSize) - 1,
        0,
        gridWidth - 1,
      )
      const minRow = clamp(
        Math.floor((obstacle.center.y - halfHeight - origin.y) / cellSize),
        0,
        gridHeight - 1,
      )
      const maxRow = clamp(
        Math.ceil((obstacle.center.y + halfHeight - origin.y) / cellSize) - 1,
        0,
        gridHeight - 1,
      )

      for (let row = minRow; row <= maxRow; row += 1) {
        for (let column = minColumn; column <= maxColumn; column += 1) {
          const index = getGridIndex(column, row, gridWidth)

          if ((grid[index]! & GridCellFlags.obstacle) === 0) {
            obstacleCellCount += 1
          }

          grid[index] = grid[index]! | GridCellFlags.obstacle
        }
      }
    }

    const startArea: GridTerminalArea = {
      obstacleIndices: this.params.terminalObstacles.busStart.obstacleIndices,
      centroid: this.params.terminalObstacles.busStart.centroid,
      centerCell: getCellAddressFromPoint({
        point: this.params.terminalObstacles.busStart.centroid,
        origin,
        cellSize,
        gridWidth,
        gridHeight,
      }),
    }
    const endArea: GridTerminalArea = {
      obstacleIndices: this.params.terminalObstacles.busEnd.obstacleIndices,
      centroid: this.params.terminalObstacles.busEnd.centroid,
      centerCell: getCellAddressFromPoint({
        point: this.params.terminalObstacles.busEnd.centroid,
        origin,
        cellSize,
        gridWidth,
        gridHeight,
      }),
    }

    rasterizeObstacleIndices({
      flag: GridCellFlags.startArea,
      grid,
      obstacleIndices: startArea.obstacleIndices,
      obstacles: this.params.inputProblem.obstacles,
      origin,
      cellSize,
      gridWidth,
      gridHeight,
    })
    rasterizeObstacleIndices({
      flag: GridCellFlags.endArea,
      grid,
      obstacleIndices: endArea.obstacleIndices,
      obstacles: this.params.inputProblem.obstacles,
      origin,
      cellSize,
      gridWidth,
      gridHeight,
    })

    for (let index = 0; index < grid.length; index += 1) {
      if (
        (grid[index]! & (GridCellFlags.startArea | GridCellFlags.endArea)) ===
        0
      ) {
        continue
      }

      grid[index] = grid[index]! & ~GridCellFlags.obstacle
    }

    obstacleCellCount = 0

    for (let index = 0; index < grid.length; index += 1) {
      if ((grid[index]! & GridCellFlags.obstacle) !== 0) {
        obstacleCellCount += 1
      }
    }

    this.output = {
      traceCount,
      traceWidth,
      traceSpacing: ADJACENT_TRACE_SPACING,
      requiredBusWidth,
      cellSize,
      origin,
      bounds,
      gridWidth,
      gridHeight,
      grid,
      obstacleCellCount,
      busConnectedObstacleIndices,
      startArea,
      endArea,
    }
    this.stats = {
      phase: "done",
      traceCount,
      traceWidth,
      traceSpacing: ADJACENT_TRACE_SPACING,
      requiredBusWidth,
      cellSize,
      gridWidth,
      gridHeight,
      obstacleCellCount,
      busConnectedObstacleCount: busConnectedObstacleIndices.length,
      startAreaCell: `${startArea.centerCell.column},${startArea.centerCell.row}`,
      endAreaCell: `${endArea.centerCell.column},${endArea.centerCell.row}`,
    }
    this.solved = true
  }

  override visualize(): GraphicsObject {
    if (!this.output) {
      return {
        title: "Stage 2 - Build Grid",
        coordinateSystem: "cartesian",
        points: [],
        lines: [],
        rects: [],
        circles: [],
        texts: [],
      }
    }

    return createGridBuilderVisualization({
      inputProblem: this.params.inputProblem,
      output: this.output,
      title: "Stage 2 - Build Grid",
    })
  }

  override getOutput(): GridBuilderOutput | null {
    return this.output
  }

  override getConstructorParams() {
    return [this.params]
  }
}

export { GridBuilderSolver as GridBuilder }
