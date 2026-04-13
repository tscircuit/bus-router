import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import type { BusRouterInput } from "./IdentifyBusTerminalObstaclesSolver"
import {
  type FindFanoutStartEndOutput,
  createFindFanoutStartEndVisualization,
} from "./FindFanoutStartEndSolver"
import {
  GridCellFlags,
  type GridBuilderOutput,
  type GridCellAddress,
  getGridIndex,
} from "./GridBuilderSolver"

const DEFAULT_GREEDY_MULTIPLIER = 1.5
const DEFAULT_OBSTACLE_SEARCH_CELLS = 10
const CURRENT_PATH_COLOR = "#14b8a6"
const CURRENT_CANDIDATE_COLOR = "#f97316"

type CandidateStatus = "open" | "closed"

interface PathCandidateRecord {
  cell: GridCellAddress
  parentIndex: number | null
  g: number
  h: number
  f: number
  status: CandidateStatus
}

export interface FindBusPathSolverInput {
  inputProblem: BusRouterInput
  grid: GridBuilderOutput
  fanoutStartEnd: FindFanoutStartEndOutput
  greedyMultiplier?: number
  obstacleSearchCells?: number
  obstacleProximityPenalty?: number
}

export interface FindBusPathOutput {
  faninCell: GridCellAddress
  fanoutCell: GridCellAddress
  path: GridCellAddress[]
  pathLength: number
  pathCost: number
  exploredCandidateCount: number
  visitedCellCount: number
  greedyMultiplier: number
  obstacleSearchCells: number
  obstacleProximityPenalty: number
}

const isInBounds = (
  column: number,
  row: number,
  gridWidth: number,
  gridHeight: number,
): boolean => column >= 0 && column < gridWidth && row >= 0 && row < gridHeight

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

const getManhattanDistance = (
  cellA: GridCellAddress,
  cellB: GridCellAddress,
): number =>
  Math.abs(cellA.column - cellB.column) + Math.abs(cellA.row - cellB.row)

export class FindBusPathSolver extends BaseSolver {
  private output: FindBusPathOutput | null = null
  private readonly records = new Map<number, PathCandidateRecord>()
  private readonly openQueue: number[] = []
  private currentCandidateIndex: number | null = null
  private currentPath: GridCellAddress[] = []
  private exploredCandidateCount = 0
  private readonly greedyMultiplier: number
  private readonly obstacleSearchCells: number
  private readonly obstacleProximityPenalty: number
  private readonly faninCell: GridCellAddress
  private readonly fanoutCell: GridCellAddress

  constructor(private readonly params: FindBusPathSolverInput) {
    super()
    this.greedyMultiplier = params.greedyMultiplier ?? DEFAULT_GREEDY_MULTIPLIER
    this.obstacleSearchCells = Math.max(
      1,
      params.obstacleSearchCells ?? DEFAULT_OBSTACLE_SEARCH_CELLS,
    )
    this.obstacleProximityPenalty =
      params.obstacleProximityPenalty ?? params.grid.cellSize * 5
    this.faninCell = params.fanoutStartEnd.selectedFaninCandidate.cell
    this.fanoutCell = params.fanoutStartEnd.selectedFanoutCandidate.cell
    this.MAX_ITERATIONS = params.grid.grid.length + 10

    const startRecord = this.createCandidateRecord(this.faninCell, null, 0)
    this.records.set(this.faninCell.index, startRecord)
    this.openQueue.push(this.faninCell.index)
    this.updateStats("search")
  }

  computeProgress(): number {
    if (this.solved) {
      return 1
    }

    return Math.min(
      0.95,
      this.exploredCandidateCount / Math.max(1, this.params.grid.grid.length),
    )
  }

  private isWalkableCell(column: number, row: number): boolean {
    if (
      !isInBounds(
        column,
        row,
        this.params.grid.gridWidth,
        this.params.grid.gridHeight,
      )
    ) {
      return false
    }

    const cellIndex = getGridIndex(column, row, this.params.grid.gridWidth)

    if (
      cellIndex === this.faninCell.index ||
      cellIndex === this.fanoutCell.index
    ) {
      return true
    }

    const flags = this.params.grid.grid[cellIndex]!
    return (
      (flags & GridCellFlags.obstacle) === 0 &&
      (flags & GridCellFlags.startArea) === 0 &&
      (flags & GridCellFlags.endArea) === 0
    )
  }

  private getObstacleDistance(column: number, row: number): number {
    let nearestObstacleDistance = this.obstacleSearchCells

    for (
      let columnOffset = -this.obstacleSearchCells;
      columnOffset <= this.obstacleSearchCells;
      columnOffset += 1
    ) {
      for (
        let rowOffset = -this.obstacleSearchCells;
        rowOffset <= this.obstacleSearchCells;
        rowOffset += 1
      ) {
        const manhattanDistance =
          Math.abs(columnOffset) + Math.abs(rowOffset)

        if (
          manhattanDistance === 0 ||
          manhattanDistance > this.obstacleSearchCells
        ) {
          continue
        }

        const candidateColumn = column + columnOffset
        const candidateRow = row + rowOffset

        if (
          !isInBounds(
            candidateColumn,
            candidateRow,
            this.params.grid.gridWidth,
            this.params.grid.gridHeight,
          )
        ) {
          continue
        }

        const flags =
          this.params.grid.grid[
            getGridIndex(
              candidateColumn,
              candidateRow,
              this.params.grid.gridWidth,
            )
          ]!

        if ((flags & GridCellFlags.obstacle) === 0) {
          continue
        }

        nearestObstacleDistance = Math.min(
          nearestObstacleDistance,
          manhattanDistance,
        )
      }
    }

    return nearestObstacleDistance
  }

  private getStepCost(nextCell: GridCellAddress): number {
    const cellDistToObstacle = this.getObstacleDistance(
      nextCell.column,
      nextCell.row,
    )
    // Treat proximity as added cost; subtracting a "penalty" would attract the path into obstacles.
    const obstacleProximityCost = Math.max(
      0,
      this.obstacleProximityPenalty -
        (cellDistToObstacle / this.obstacleSearchCells) *
          this.obstacleProximityPenalty,
    )

    return this.params.grid.cellSize + obstacleProximityCost
  }

  private createCandidateRecord(
    cell: GridCellAddress,
    parentIndex: number | null,
    g: number,
  ): PathCandidateRecord {
    const h =
      getManhattanDistance(cell, this.fanoutCell) * this.params.grid.cellSize
    return {
      cell,
      parentIndex,
      g,
      h,
      f: g + h * this.greedyMultiplier,
      status: "open",
    }
  }

  private dequeueBestCandidateIndex(): number | null {
    if (this.openQueue.length === 0) {
      return null
    }

    let bestQueueIndex = 0
    let bestCellIndex = this.openQueue[0]!
    let bestRecord = this.records.get(bestCellIndex)!

    for (let index = 1; index < this.openQueue.length; index += 1) {
      const cellIndex = this.openQueue[index]!
      const record = this.records.get(cellIndex)

      if (!record) {
        continue
      }

      if (
        record.f < bestRecord.f ||
        (record.f === bestRecord.f && record.h < bestRecord.h)
      ) {
        bestQueueIndex = index
        bestCellIndex = cellIndex
        bestRecord = record
      }
    }

    this.openQueue.splice(bestQueueIndex, 1)
    return bestCellIndex
  }

  private reconstructPath(cellIndex: number): GridCellAddress[] {
    const path: GridCellAddress[] = []
    let currentIndex: number | null = cellIndex

    while (currentIndex !== null) {
      const record = this.records.get(currentIndex)

      if (!record) {
        break
      }

      path.push(record.cell)
      currentIndex = record.parentIndex
    }

    return path.reverse()
  }

  private updateStats(phase: string) {
    const currentRecord =
      this.currentCandidateIndex !== null
        ? this.records.get(this.currentCandidateIndex)
        : null

    this.stats = {
      phase,
      queueSize: this.openQueue.length,
      visitedCellCount: this.records.size,
      exploredCandidateCount: this.exploredCandidateCount,
      currentCandidate: currentRecord
        ? `${currentRecord.cell.column},${currentRecord.cell.row}`
        : null,
      currentG: currentRecord?.g ?? null,
      currentH: currentRecord?.h ?? null,
      currentF: currentRecord?.f ?? null,
      currentPathLength: this.currentPath.length,
      greedyMultiplier: this.greedyMultiplier,
      obstacleSearchCells: this.obstacleSearchCells,
      obstacleProximityPenalty: this.obstacleProximityPenalty,
    }
  }

  override _step() {
    const nextCellIndex = this.dequeueBestCandidateIndex()

    if (nextCellIndex === null) {
      this.failed = true
      this.error =
        "Unable to find a walkable bus path between the selected fanin and fanout cells."
      this.updateStats("failed")
      return
    }

    const currentRecord = this.records.get(nextCellIndex)

    if (!currentRecord) {
      this.failed = true
      this.error = `Missing candidate record for cell index ${nextCellIndex}.`
      this.updateStats("failed")
      return
    }

    this.currentCandidateIndex = nextCellIndex
    this.currentPath = this.reconstructPath(nextCellIndex)
    this.exploredCandidateCount += 1

    if (nextCellIndex === this.fanoutCell.index) {
      this.output = {
        faninCell: this.faninCell,
        fanoutCell: this.fanoutCell,
        path: this.currentPath,
        pathLength: this.currentPath.length,
        pathCost: currentRecord.g,
        exploredCandidateCount: this.exploredCandidateCount,
        visitedCellCount: this.records.size,
        greedyMultiplier: this.greedyMultiplier,
        obstacleSearchCells: this.obstacleSearchCells,
        obstacleProximityPenalty: this.obstacleProximityPenalty,
      }
      this.solved = true
      this.updateStats("done")
      return
    }

    currentRecord.status = "closed"

    const neighborOffsets = [
      { columnStep: 0, rowStep: -1 },
      { columnStep: 1, rowStep: 0 },
      { columnStep: 0, rowStep: 1 },
      { columnStep: -1, rowStep: 0 },
    ]

    for (const neighborOffset of neighborOffsets) {
      const neighborColumn =
        currentRecord.cell.column + neighborOffset.columnStep
      const neighborRow = currentRecord.cell.row + neighborOffset.rowStep

      if (!this.isWalkableCell(neighborColumn, neighborRow)) {
        continue
      }

      const neighborCell = getCellAddress(
        this.params.grid,
        neighborColumn,
        neighborRow,
      )
      const tentativeG = currentRecord.g + this.getStepCost(neighborCell)
      const existingRecord = this.records.get(neighborCell.index)

      if (existingRecord && tentativeG >= existingRecord.g) {
        continue
      }

      const nextRecord = this.createCandidateRecord(
        neighborCell,
        currentRecord.cell.index,
        tentativeG,
      )

      this.records.set(neighborCell.index, nextRecord)

      if (!this.openQueue.includes(neighborCell.index)) {
        this.openQueue.push(neighborCell.index)
      }
    }

    this.updateStats("search")
  }

  override visualize(): GraphicsObject {
    const extraRects: NonNullable<GraphicsObject["rects"]> =
      this.currentPath.map((cell) => ({
        center: cell.center,
        width: this.params.grid.cellSize * 0.5,
        height: this.params.grid.cellSize * 0.5,
        fill: "rgba(20, 184, 166, 0.22)",
        stroke: CURRENT_PATH_COLOR,
        label: "current-path-cell",
      }))
    const extraLines: NonNullable<GraphicsObject["lines"]> =
      this.currentPath.length > 1
        ? [
            {
              points: this.currentPath.map((cell) => cell.center),
              strokeColor: CURRENT_PATH_COLOR,
              label: "current-path",
            },
          ]
        : []
    const extraCircles: NonNullable<GraphicsObject["circles"]> =
      this.currentCandidateIndex !== null
        ? [
            {
              center: this.records.get(this.currentCandidateIndex)!.cell.center,
              radius: this.params.grid.cellSize * 0.28,
              fill: "rgba(249, 115, 22, 0.35)",
              stroke: CURRENT_CANDIDATE_COLOR,
              label: "current-candidate",
            },
          ]
        : []
    const extraTexts: NonNullable<GraphicsObject["texts"]> =
      this.currentCandidateIndex !== null
        ? [
            {
              x: this.records.get(this.currentCandidateIndex)!.cell.center.x,
              y:
                this.records.get(this.currentCandidateIndex)!.cell.center.y +
                this.params.grid.cellSize * 0.8,
              text: "Current Candidate",
              fontSize: 4.5,
              color: CURRENT_CANDIDATE_COLOR,
            },
          ]
        : []

    return createFindFanoutStartEndVisualization({
      inputProblem: this.params.inputProblem,
      grid: this.params.grid,
      output: this.params.fanoutStartEnd,
      title: "Stage 4 - Find Bus Path",
      extraRects,
      extraLines,
      extraCircles,
      extraTexts,
    })
  }

  override getOutput(): FindBusPathOutput | null {
    return this.output
  }

  override getConstructorParams() {
    return [this.params]
  }
}

export { FindBusPathSolver as FindBusPath }
