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
const GRID_START = 1 << 1
const GRID_END = 1 << 2

const GRID_OUTLINE_STROKE = "rgba(15, 23, 42, 0.1)"
const GRID_BORDER_STROKE = "rgba(220, 38, 38, 0.7)"
const GRID_OBSTACLE_FILL = "rgba(220, 38, 38, 0.38)"
const GRID_START_FILL = "rgba(37, 99, 235, 0.5)"
const GRID_START_STROKE = "#2563eb"
const GRID_END_FILL = "rgba(22, 163, 74, 0.5)"
const GRID_END_STROKE = "#16a34a"
const SUMMARY_TEXT_COLOR = "#0f172a"

export const GridCellFlags = {
  obstacle: GRID_OBSTACLE,
  start: GRID_START,
  end: GRID_END,
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
  startCell: GridCellAddress
  endCell: GridCellAddress
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

const getGridIndex = (column: number, row: number, gridWidth: number): number =>
  row * gridWidth + column

const getCellAddressFromPoint = (params: {
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

const createTerminalCellRects = (output: GridBuilderOutput): Rect[] => [
  {
    center: output.startCell.center,
    width: output.cellSize,
    height: output.cellSize,
    fill: GRID_START_FILL,
    stroke: GRID_START_STROKE,
    label: "bus-start-cell",
  },
  {
    center: output.endCell.center,
    width: output.cellSize,
    height: output.cellSize,
    fill: GRID_END_FILL,
    stroke: GRID_END_STROKE,
    label: "bus-end-cell",
  },
]

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
    const cellSize = requiredBusWidth / 4
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
    let obstacleCellCount = 0

    // Rasterize each obstacle's bounding box into the occupancy grid.
    for (const obstacle of this.params.inputProblem.obstacles) {
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

    const startCell = getCellAddressFromPoint({
      point: this.params.terminalObstacles.busStart.centroid,
      origin,
      cellSize,
      gridWidth,
      gridHeight,
    })
    const endCell = getCellAddressFromPoint({
      point: this.params.terminalObstacles.busEnd.centroid,
      origin,
      cellSize,
      gridWidth,
      gridHeight,
    })

    grid[startCell.index] = grid[startCell.index]! | GridCellFlags.start
    grid[endCell.index] = grid[endCell.index]! | GridCellFlags.end

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
      startCell,
      endCell,
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
      startCell: `${startCell.column},${startCell.row}`,
      endCell: `${endCell.column},${endCell.row}`,
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

    const summaryLines = [
      `Trace count: ${this.output.traceCount}`,
      `Required bus width: ${this.output.requiredBusWidth.toFixed(3)}`,
      `Grid cell size: ${this.output.cellSize.toFixed(3)}`,
      `Grid size: ${this.output.gridWidth} x ${this.output.gridHeight}`,
      `Obstacle cells: ${this.output.obstacleCellCount}`,
    ]
    const texts = createSummaryTexts(
      this.output.bounds,
      this.output.cellSize,
      summaryLines,
    )

    texts.push({
      x: this.output.startCell.center.x,
      y: this.output.startCell.center.y + this.output.cellSize * 0.7,
      text: "Bus Start",
      fontSize: Math.max(4, this.output.cellSize * 0.65),
      color: GRID_START_STROKE,
    })
    texts.push({
      x: this.output.endCell.center.x,
      y: this.output.endCell.center.y + this.output.cellSize * 0.7,
      text: "Bus End",
      fontSize: Math.max(4, this.output.cellSize * 0.65),
      color: GRID_END_STROKE,
    })

    return {
      title: "Stage 2 - Build Grid",
      coordinateSystem: "cartesian",
      points: [],
      lines: [],
      rects: [
        ...createObstacleCellRects(this.output),
        ...createTerminalCellRects(this.output),
        {
          center: {
            x: (this.output.bounds.minX + this.output.bounds.maxX) / 2,
            y: (this.output.bounds.minY + this.output.bounds.maxY) / 2,
          },
          width: this.output.gridWidth * this.output.cellSize,
          height: this.output.gridHeight * this.output.cellSize,
          stroke: GRID_BORDER_STROKE,
        },
      ],
      circles: [],
      texts,
    }
  }

  override getOutput(): GridBuilderOutput | null {
    return this.output
  }

  override getConstructorParams() {
    return [this.params]
  }
}

export { GridBuilderSolver as GridBuilder }
