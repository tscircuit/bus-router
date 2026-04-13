import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject, Line } from "graphics-debug"
import type {
  BusObstacle,
  BusRouterInput,
  XYPoint,
} from "./IdentifyBusTerminalObstaclesSolver"
import {
  GridCellFlags,
  type GridBuilderOutput,
  type GridCellAddress,
  createGridBuilderVisualization,
  getGridIndex,
} from "./GridBuilderSolver"

const FANOUT_COLOR = "#2563eb"
const FANIN_COLOR = "#f59e0b"
const SELECTED_COLOR = "#16a34a"

type AxisOrientation = "horizontal" | "vertical"
type SearchDirection = "negative" | "positive"

export interface RegionLine {
  orientation: AxisOrientation
  start: XYPoint
  end: XYPoint
  midpoint: XYPoint
}

export interface RegionCandidate {
  direction: SearchDirection
  cell: GridCellAddress
}

export interface FindFanoutStartEndSolverInput {
  inputProblem: BusRouterInput
  grid: GridBuilderOutput
}

export interface FindFanoutStartEndOutput {
  fanoutRegionLine: RegionLine
  faninRegionLine: RegionLine
  fanoutCandidates: RegionCandidate[]
  faninCandidates: RegionCandidate[]
  selectedFanoutCandidate: RegionCandidate
  selectedFaninCandidate: RegionCandidate
  selectedDistance: number
}

const isInBounds = (
  column: number,
  row: number,
  gridWidth: number,
  gridHeight: number,
): boolean => column >= 0 && column < gridWidth && row >= 0 && row < gridHeight

const isFreeCell = (
  grid: GridBuilderOutput,
  column: number,
  row: number,
): boolean => {
  if (!isInBounds(column, row, grid.gridWidth, grid.gridHeight)) {
    return false
  }

  const flags = grid.grid[getGridIndex(column, row, grid.gridWidth)]!
  return (
    (flags & GridCellFlags.obstacle) === 0 &&
    (flags & GridCellFlags.startArea) === 0 &&
    (flags & GridCellFlags.endArea) === 0
  )
}

const getCellAddress = (
  grid: GridBuilderOutput,
  column: number,
  row: number,
): GridCellAddress => ({
  column,
  row,
  index: getGridIndex(column, row, grid.gridWidth),
  center: {
    x: grid.origin.x + (column + 0.5) * grid.cellSize,
    y: grid.origin.y + (row + 0.5) * grid.cellSize,
  },
})

const computeRegionBounds = (
  obstacles: BusObstacle[],
  obstacleIndices: number[],
): {
  minX: number
  maxX: number
  minY: number
  maxY: number
} => {
  let minX = Number.POSITIVE_INFINITY
  let maxX = Number.NEGATIVE_INFINITY
  let minY = Number.POSITIVE_INFINITY
  let maxY = Number.NEGATIVE_INFINITY

  for (const obstacleIndex of obstacleIndices) {
    const obstacle = obstacles[obstacleIndex]

    if (!obstacle) {
      continue
    }

    const halfWidth = obstacle.width / 2
    const halfHeight = obstacle.height / 2
    minX = Math.min(minX, obstacle.center.x - halfWidth)
    maxX = Math.max(maxX, obstacle.center.x + halfWidth)
    minY = Math.min(minY, obstacle.center.y - halfHeight)
    maxY = Math.max(maxY, obstacle.center.y + halfHeight)
  }

  return { minX, maxX, minY, maxY }
}

const computeRegionLine = (params: {
  obstacles: BusObstacle[]
  obstacleIndices: number[]
  centroid: XYPoint
}): RegionLine => {
  const bounds = computeRegionBounds(params.obstacles, params.obstacleIndices)
  const spanX = bounds.maxX - bounds.minX
  const spanY = bounds.maxY - bounds.minY

  if (spanX >= spanY) {
    return {
      orientation: "horizontal",
      start: {
        x: bounds.minX,
        y: params.centroid.y,
      },
      end: {
        x: bounds.maxX,
        y: params.centroid.y,
      },
      midpoint: {
        x: (bounds.minX + bounds.maxX) / 2,
        y: params.centroid.y,
      },
    }
  }

  return {
    orientation: "vertical",
    start: {
      x: params.centroid.x,
      y: bounds.minY,
    },
    end: {
      x: params.centroid.x,
      y: bounds.maxY,
    },
    midpoint: {
      x: params.centroid.x,
      y: (bounds.minY + bounds.maxY) / 2,
    },
  }
}

const getTangentStep = (
  orientation: AxisOrientation,
): {
  columnStep: number
  rowStep: number
} =>
  orientation === "horizontal"
    ? { columnStep: 1, rowStep: 0 }
    : { columnStep: 0, rowStep: 1 }

const getOrthogonalSearchSteps = (
  orientation: AxisOrientation,
): Array<{
  direction: SearchDirection
  columnStep: number
  rowStep: number
}> =>
  orientation === "horizontal"
    ? [
        { direction: "negative", columnStep: 0, rowStep: -1 },
        { direction: "positive", columnStep: 0, rowStep: 1 },
      ]
    : [
        { direction: "negative", columnStep: -1, rowStep: 0 },
        { direction: "positive", columnStep: 1, rowStep: 0 },
      ]

const hasTwoFreeSpaceCellsSurrounding = (params: {
  grid: GridBuilderOutput
  column: number
  row: number
  tangentColumnStep: number
  tangentRowStep: number
  orthogonalColumnStep: number
  orthogonalRowStep: number
}): boolean => {
  if (!isFreeCell(params.grid, params.column, params.row)) {
    return false
  }

  const previousAlongLine = isFreeCell(
    params.grid,
    params.column - params.tangentColumnStep,
    params.row - params.tangentRowStep,
  )
  const nextAlongLine = isFreeCell(
    params.grid,
    params.column + params.tangentColumnStep,
    params.row + params.tangentRowStep,
  )
  const nextAwayFromRegion = isFreeCell(
    params.grid,
    params.column + params.orthogonalColumnStep,
    params.row + params.orthogonalRowStep,
  )

  return previousAlongLine && nextAlongLine && nextAwayFromRegion
}

const findCandidateInDirection = (params: {
  grid: GridBuilderOutput
  anchorCell: GridCellAddress
  orientation: AxisOrientation
  direction: SearchDirection
  columnStep: number
  rowStep: number
}): RegionCandidate | null => {
  const tangentStep = getTangentStep(params.orientation)
  const maxDistance = Math.max(params.grid.gridWidth, params.grid.gridHeight)

  for (let distance = 1; distance < maxDistance; distance += 1) {
    const column = params.anchorCell.column + params.columnStep * distance
    const row = params.anchorCell.row + params.rowStep * distance

    if (
      !isInBounds(column, row, params.grid.gridWidth, params.grid.gridHeight)
    ) {
      return null
    }

    if (
      !hasTwoFreeSpaceCellsSurrounding({
        grid: params.grid,
        column,
        row,
        tangentColumnStep: tangentStep.columnStep,
        tangentRowStep: tangentStep.rowStep,
        orthogonalColumnStep: params.columnStep,
        orthogonalRowStep: params.rowStep,
      })
    ) {
      continue
    }

    return {
      direction: params.direction,
      cell: getCellAddress(params.grid, column, row),
    }
  }

  return null
}

const getDistanceBetweenCells = (
  cellA: GridCellAddress,
  cellB: GridCellAddress,
): number =>
  Math.hypot(cellA.center.x - cellB.center.x, cellA.center.y - cellB.center.y)

const createRegionLineGraphic = (params: {
  line: RegionLine
  color: string
  label: string
}): Line => ({
  points: [params.line.start, params.line.end],
  strokeColor: params.color,
  label: params.label,
})

const createCandidateTexts = (params: {
  candidates: RegionCandidate[]
  color: string
  prefix: string
}): NonNullable<GraphicsObject["texts"]> =>
  params.candidates.map((candidate, index) => ({
    x: candidate.cell.center.x,
    y: candidate.cell.center.y + 0.7,
    text: `${params.prefix} ${index + 1}`,
    fontSize: 4.5,
    color: params.color,
  }))

export const createFindFanoutStartEndVisualization = (params: {
  inputProblem: BusRouterInput
  grid: GridBuilderOutput
  output: FindFanoutStartEndOutput
  title: string
  extraLines?: NonNullable<GraphicsObject["lines"]>
  extraRects?: NonNullable<GraphicsObject["rects"]>
  extraCircles?: NonNullable<GraphicsObject["circles"]>
  extraTexts?: NonNullable<GraphicsObject["texts"]>
  extraPoints?: NonNullable<GraphicsObject["points"]>
}): GraphicsObject => {
  const fanoutTexts = createCandidateTexts({
    candidates: params.output.fanoutCandidates,
    color: FANOUT_COLOR,
    prefix: "Fanout",
  })
  const faninTexts = createCandidateTexts({
    candidates: params.output.faninCandidates,
    color: FANIN_COLOR,
    prefix: "Fanin",
  })

  return createGridBuilderVisualization({
    inputProblem: params.inputProblem,
    output: params.grid,
    title: params.title,
    extraLines: [
      createRegionLineGraphic({
        line: params.output.fanoutRegionLine,
        color: FANOUT_COLOR,
        label: "fanout-area-line",
      }),
      createRegionLineGraphic({
        line: params.output.faninRegionLine,
        color: FANIN_COLOR,
        label: "fanin-area-line",
      }),
      {
        points: [
          params.output.selectedFanoutCandidate.cell.center,
          params.output.selectedFaninCandidate.cell.center,
        ],
        strokeColor: SELECTED_COLOR,
        label: "selected-fanout-fanin-pair",
      },
      ...(params.extraLines ?? []),
    ],
    extraRects: params.extraRects,
    extraCircles: [
      ...params.output.fanoutCandidates.map((candidate) => ({
        center: candidate.cell.center,
        radius: params.grid.cellSize * 0.22,
        fill: "rgba(37, 99, 235, 0.35)",
        stroke: FANOUT_COLOR,
        label: "fanout-candidate",
      })),
      ...params.output.faninCandidates.map((candidate) => ({
        center: candidate.cell.center,
        radius: params.grid.cellSize * 0.22,
        fill: "rgba(245, 158, 11, 0.35)",
        stroke: FANIN_COLOR,
        label: "fanin-candidate",
      })),
      {
        center: params.output.selectedFanoutCandidate.cell.center,
        radius: params.grid.cellSize * 0.3,
        fill: "rgba(22, 163, 74, 0.35)",
        stroke: SELECTED_COLOR,
        label: "selected-fanout",
      },
      {
        center: params.output.selectedFaninCandidate.cell.center,
        radius: params.grid.cellSize * 0.3,
        fill: "rgba(22, 163, 74, 0.35)",
        stroke: SELECTED_COLOR,
        label: "selected-fanin",
      },
      ...(params.extraCircles ?? []),
    ],
    extraTexts: [
      ...fanoutTexts,
      ...faninTexts,
      {
        x: params.output.selectedFanoutCandidate.cell.center.x,
        y:
          params.output.selectedFanoutCandidate.cell.center.y -
          params.grid.cellSize * 0.8,
        text: "Selected Fanout",
        fontSize: 4.5,
        color: SELECTED_COLOR,
      },
      {
        x: params.output.selectedFaninCandidate.cell.center.x,
        y:
          params.output.selectedFaninCandidate.cell.center.y -
          params.grid.cellSize * 0.8,
        text: "Selected Fanin",
        fontSize: 4.5,
        color: SELECTED_COLOR,
      },
      ...(params.extraTexts ?? []),
    ],
    extraPoints: params.extraPoints,
  })
}

export class FindFanoutStartEndSolver extends BaseSolver {
  private output: FindFanoutStartEndOutput | null = null

  constructor(private readonly params: FindFanoutStartEndSolverInput) {
    super()
    this.MAX_ITERATIONS = 1
    this.stats = {
      phase: "find-fanout-start-end",
      fanoutCandidateCount: 0,
      faninCandidateCount: 0,
    }
  }

  override _step() {
    const fanoutRegionLine = computeRegionLine({
      obstacles: this.params.inputProblem.obstacles,
      obstacleIndices: this.params.grid.startArea.obstacleIndices,
      centroid: this.params.grid.startArea.centroid,
    })
    const faninRegionLine = computeRegionLine({
      obstacles: this.params.inputProblem.obstacles,
      obstacleIndices: this.params.grid.endArea.obstacleIndices,
      centroid: this.params.grid.endArea.centroid,
    })
    const fanoutCandidates = getOrthogonalSearchSteps(
      fanoutRegionLine.orientation,
    )
      .map((searchStep) =>
        findCandidateInDirection({
          grid: this.params.grid,
          anchorCell: this.params.grid.startArea.centerCell,
          orientation: fanoutRegionLine.orientation,
          direction: searchStep.direction,
          columnStep: searchStep.columnStep,
          rowStep: searchStep.rowStep,
        }),
      )
      .filter((candidate): candidate is RegionCandidate => candidate !== null)
    const faninCandidates = getOrthogonalSearchSteps(
      faninRegionLine.orientation,
    )
      .map((searchStep) =>
        findCandidateInDirection({
          grid: this.params.grid,
          anchorCell: this.params.grid.endArea.centerCell,
          orientation: faninRegionLine.orientation,
          direction: searchStep.direction,
          columnStep: searchStep.columnStep,
          rowStep: searchStep.rowStep,
        }),
      )
      .filter((candidate): candidate is RegionCandidate => candidate !== null)

    if (fanoutCandidates.length === 0 || faninCandidates.length === 0) {
      this.failed = true
      this.error =
        "Unable to find free-space fanout/fanin candidates on both bus terminal areas."
      return
    }

    let selectedFanoutCandidate = fanoutCandidates[0]!
    let selectedFaninCandidate = faninCandidates[0]!
    let selectedDistance = getDistanceBetweenCells(
      selectedFanoutCandidate.cell,
      selectedFaninCandidate.cell,
    )

    for (const fanoutCandidate of fanoutCandidates) {
      for (const faninCandidate of faninCandidates) {
        const distance = getDistanceBetweenCells(
          fanoutCandidate.cell,
          faninCandidate.cell,
        )

        if (distance < selectedDistance) {
          selectedDistance = distance
          selectedFanoutCandidate = fanoutCandidate
          selectedFaninCandidate = faninCandidate
        }
      }
    }

    this.output = {
      fanoutRegionLine,
      faninRegionLine,
      fanoutCandidates,
      faninCandidates,
      selectedFanoutCandidate,
      selectedFaninCandidate,
      selectedDistance,
    }
    this.stats = {
      phase: "done",
      fanoutCandidateCount: fanoutCandidates.length,
      faninCandidateCount: faninCandidates.length,
      selectedDistance,
      selectedFanoutCell: `${selectedFanoutCandidate.cell.column},${selectedFanoutCandidate.cell.row}`,
      selectedFaninCell: `${selectedFaninCandidate.cell.column},${selectedFaninCandidate.cell.row}`,
    }
    this.solved = true
  }

  override visualize(): GraphicsObject {
    if (!this.output) {
      return {
        title: "Stage 3 - Find Fanout Start/End",
        coordinateSystem: "cartesian",
        points: [],
        lines: [],
        rects: [],
        circles: [],
        texts: [],
      }
    }

    return createFindFanoutStartEndVisualization({
      inputProblem: this.params.inputProblem,
      grid: this.params.grid,
      output: this.output,
      title: "Stage 3 - Find Fanout Start/End",
    })
  }

  override getOutput(): FindFanoutStartEndOutput | null {
    return this.output
  }

  override getConstructorParams() {
    return [this.params]
  }
}

export { FindFanoutStartEndSolver as FindFanoutStartEnd }
