import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import type { BusRouterInput } from "./IdentifyBusTerminalObstaclesSolver"
import {
  type FindFanoutStartEndOutput,
  type RegionCandidate,
  type RegionLine,
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
const MIN_DIRECTION_STREAK = 2
const FORCED_ENDPOINT_STEPS = 2
const DEFAULT_NINETY_DEGREE_TURN_PENALTY_MULTIPLIER = 2
const CURRENT_PATH_COLOR = "#14b8a6"
const CURRENT_CANDIDATE_COLOR = "#f97316"

type CandidateStatus = "open" | "closed"

interface MoveDirection {
  columnStep: number
  rowStep: number
}

interface TerminalCorridor {
  terminalCell: GridCellAddress
  outwardDirection: MoveDirection
  anchorCell: GridCellAddress
  pathFromTerminal: GridCellAddress[]
}

interface PathCandidateRecord {
  stateKey: string
  cell: GridCellAddress
  parentStateKey: string | null
  g: number
  h: number
  f: number
  status: CandidateStatus
  direction: MoveDirection
  directionRunLength: number
}

export interface FindBusPathSolverInput {
  inputProblem: BusRouterInput
  grid: GridBuilderOutput
  fanoutStartEnd: FindFanoutStartEndOutput
  greedyMultiplier?: number
  obstacleSearchCells?: number
  obstacleProximityPenalty?: number
  ninetyDegreeTurnPenalty?: number
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
  ninetyDegreeTurnPenalty: number
}

const ALL_MOVE_DIRECTIONS: MoveDirection[] = [
  { columnStep: 0, rowStep: -1 },
  { columnStep: 1, rowStep: -1 },
  { columnStep: 1, rowStep: 0 },
  { columnStep: 1, rowStep: 1 },
  { columnStep: 0, rowStep: 1 },
  { columnStep: -1, rowStep: 1 },
  { columnStep: -1, rowStep: 0 },
  { columnStep: -1, rowStep: -1 },
]

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

const isSameDirection = (
  directionA: MoveDirection,
  directionB: MoveDirection,
): boolean =>
  directionA.columnStep === directionB.columnStep &&
  directionA.rowStep === directionB.rowStep

const isNinetyDegreeTurn = (
  directionA: MoveDirection,
  directionB: MoveDirection,
): boolean =>
  directionA.columnStep * directionB.columnStep +
    directionA.rowStep * directionB.rowStep ===
  0

const invertDirection = (direction: MoveDirection): MoveDirection => ({
  columnStep: -direction.columnStep,
  rowStep: -direction.rowStep,
})

const getDirectionKey = (direction: MoveDirection): string =>
  `${direction.columnStep},${direction.rowStep}`

const getStateKey = (
  cellIndex: number,
  direction: MoveDirection,
  directionRunLength: number,
): string =>
  `${cellIndex}:${getDirectionKey(direction)}:${Math.min(
    MIN_DIRECTION_STREAK,
    directionRunLength,
  )}`

const getRegionCandidateDirectionVector = (
  line: RegionLine,
  candidate: RegionCandidate,
): MoveDirection =>
  line.orientation === "horizontal"
    ? {
        columnStep: 0,
        rowStep: candidate.direction === "negative" ? -1 : 1,
      }
    : {
        columnStep: candidate.direction === "negative" ? -1 : 1,
        rowStep: 0,
      }

const stitchPathSegments = (
  ...segments: GridCellAddress[][]
): GridCellAddress[] => {
  const stitchedPath: GridCellAddress[] = []

  for (const segment of segments) {
    for (const cell of segment) {
      if (
        stitchedPath.length > 0 &&
        stitchedPath[stitchedPath.length - 1]!.index === cell.index
      ) {
        continue
      }

      stitchedPath.push(cell)
    }
  }

  return stitchedPath
}

export class FindBusPathSolver extends BaseSolver {
  private output: FindBusPathOutput | null = null
  private readonly records = new Map<string, PathCandidateRecord>()
  private readonly openQueue: string[] = []
  private readonly visitedCellIndices = new Set<number>()
  private currentCandidateStateKey: string | null = null
  private currentPath: GridCellAddress[] = []
  private exploredCandidateCount = 0
  private readonly greedyMultiplier: number
  private readonly obstacleSearchCells: number
  private readonly obstacleProximityPenalty: number
  private readonly ninetyDegreeTurnPenalty: number
  private readonly faninCell: GridCellAddress
  private readonly fanoutCell: GridCellAddress
  private readonly faninCorridor: TerminalCorridor | null
  private readonly fanoutCorridor: TerminalCorridor | null
  private readonly searchStartCell: GridCellAddress | null
  private readonly searchGoalCell: GridCellAddress | null
  private readonly startPathPrefix: GridCellAddress[]
  private readonly goalPathSuffix: GridCellAddress[]
  private readonly goalApproachDirection: MoveDirection | null
  private readonly maxSearchStates: number
  private initializationError: string | null = null

  constructor(private readonly params: FindBusPathSolverInput) {
    super()
    this.greedyMultiplier = params.greedyMultiplier ?? DEFAULT_GREEDY_MULTIPLIER
    this.obstacleSearchCells = Math.max(
      1,
      params.obstacleSearchCells ?? DEFAULT_OBSTACLE_SEARCH_CELLS,
    )
    this.obstacleProximityPenalty =
      params.obstacleProximityPenalty ?? params.grid.cellSize * 5
    this.ninetyDegreeTurnPenalty =
      params.ninetyDegreeTurnPenalty ??
      params.grid.cellSize * DEFAULT_NINETY_DEGREE_TURN_PENALTY_MULTIPLIER
    this.faninCell = params.fanoutStartEnd.selectedFaninCandidate.cell
    this.fanoutCell = params.fanoutStartEnd.selectedFanoutCandidate.cell
    this.maxSearchStates =
      params.grid.grid.length *
      ALL_MOVE_DIRECTIONS.length *
      MIN_DIRECTION_STREAK
    this.MAX_ITERATIONS = this.maxSearchStates + 10

    const faninCorridor = this.buildTerminalCorridor(
      this.faninCell,
      getRegionCandidateDirectionVector(
        params.fanoutStartEnd.faninRegionLine,
        params.fanoutStartEnd.selectedFaninCandidate,
      ),
      "fanin",
    )
    const fanoutCorridor = this.buildTerminalCorridor(
      this.fanoutCell,
      getRegionCandidateDirectionVector(
        params.fanoutStartEnd.fanoutRegionLine,
        params.fanoutStartEnd.selectedFanoutCandidate,
      ),
      "fanout",
    )

    this.faninCorridor = faninCorridor
    this.fanoutCorridor = fanoutCorridor
    this.searchStartCell = faninCorridor?.anchorCell ?? null
    this.searchGoalCell = fanoutCorridor?.anchorCell ?? null
    this.startPathPrefix = faninCorridor?.pathFromTerminal.slice(0, -1) ?? []
    this.goalPathSuffix =
      fanoutCorridor?.pathFromTerminal.slice(0, -1).reverse() ?? []
    this.goalApproachDirection = fanoutCorridor
      ? invertDirection(fanoutCorridor.outwardDirection)
      : null

    if (!faninCorridor || !fanoutCorridor) {
      this.updateStats("initialization-failed")
      return
    }

    const startRecord = this.createCandidateRecord({
      cell: faninCorridor.anchorCell,
      parentStateKey: null,
      g: this.getForcedSegmentCost(faninCorridor.pathFromTerminal.slice(1)),
      direction: faninCorridor.outwardDirection,
      directionRunLength: MIN_DIRECTION_STREAK,
    })
    this.records.set(startRecord.stateKey, startRecord)
    this.openQueue.push(startRecord.stateKey)
    this.visitedCellIndices.add(this.faninCell.index)
    this.visitedCellIndices.add(startRecord.cell.index)
    this.updateStats("search")
  }

  computeProgress(): number {
    if (this.solved) {
      return 1
    }

    return Math.min(
      0.95,
      this.exploredCandidateCount / Math.max(1, this.maxSearchStates),
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

  private buildTerminalCorridor(
    terminalCell: GridCellAddress,
    outwardDirection: MoveDirection,
    label: "fanin" | "fanout",
  ): TerminalCorridor | null {
    const pathFromTerminal: GridCellAddress[] = [terminalCell]

    for (let step = 1; step <= FORCED_ENDPOINT_STEPS; step += 1) {
      const column = terminalCell.column + outwardDirection.columnStep * step
      const row = terminalCell.row + outwardDirection.rowStep * step

      if (!this.isWalkableCell(column, row)) {
        this.initializationError = `Unable to extend the selected ${label} corridor ${FORCED_ENDPOINT_STEPS} cells away from the bus.`
        return null
      }

      pathFromTerminal.push(getCellAddress(this.params.grid, column, row))
    }

    return {
      terminalCell,
      outwardDirection,
      anchorCell: pathFromTerminal[pathFromTerminal.length - 1]!,
      pathFromTerminal,
    }
  }

  private getObstacleDistance(column: number, row: number): number {
    let nearestObstacleDistance = Number.POSITIVE_INFINITY

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
        if (columnOffset === 0 && rowOffset === 0) {
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
          Math.abs(columnOffset) + Math.abs(rowOffset),
        )
      }
    }

    if (!Number.isFinite(nearestObstacleDistance)) {
      return this.obstacleSearchCells
    }

    return Math.min(this.obstacleSearchCells, nearestObstacleDistance)
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

  private getTransitionCost(
    nextCell: GridCellAddress,
    currentRecord: PathCandidateRecord,
    nextDirection: MoveDirection,
  ): number {
    const rightAngleTurnPenalty =
      !isSameDirection(currentRecord.direction, nextDirection) &&
      isNinetyDegreeTurn(currentRecord.direction, nextDirection)
        ? this.ninetyDegreeTurnPenalty
        : 0

    return this.getStepCost(nextCell) + rightAngleTurnPenalty
  }

  private getForcedSegmentCost(cells: GridCellAddress[]): number {
    let cost = 0

    for (const cell of cells) {
      cost += this.getStepCost(cell)
    }

    return cost
  }

  private createCandidateRecord(params: {
    cell: GridCellAddress
    parentStateKey: string | null
    g: number
    direction: MoveDirection
    directionRunLength: number
  }): PathCandidateRecord {
    const directionRunLength = Math.min(
      MIN_DIRECTION_STREAK,
      params.directionRunLength,
    )
    const h =
      getManhattanDistance(params.cell, this.fanoutCell) *
      this.params.grid.cellSize

    return {
      stateKey: getStateKey(
        params.cell.index,
        params.direction,
        directionRunLength,
      ),
      cell: params.cell,
      parentStateKey: params.parentStateKey,
      g: params.g,
      h,
      f: params.g + h * this.greedyMultiplier,
      status: "open",
      direction: params.direction,
      directionRunLength,
    }
  }

  private dequeueBestCandidateStateKey(): string | null {
    let bestQueueIndex = -1
    let bestStateKey: string | null = null
    let bestRecord: PathCandidateRecord | null = null

    for (let index = 0; index < this.openQueue.length; index += 1) {
      const stateKey = this.openQueue[index]!
      const record = this.records.get(stateKey)

      if (!record || record.status !== "open") {
        continue
      }

      if (
        !bestRecord ||
        record.f < bestRecord.f ||
        (record.f === bestRecord.f && record.h < bestRecord.h)
      ) {
        bestQueueIndex = index
        bestStateKey = stateKey
        bestRecord = record
      }
    }

    if (bestQueueIndex === -1 || bestStateKey === null) {
      this.openQueue.length = 0
      return null
    }

    this.openQueue.splice(bestQueueIndex, 1)
    return bestStateKey
  }

  private reconstructSearchPath(stateKey: string): GridCellAddress[] {
    const path: GridCellAddress[] = []
    let currentStateKey: string | null = stateKey

    while (currentStateKey !== null) {
      const record = this.records.get(currentStateKey)

      if (!record) {
        break
      }

      path.push(record.cell)
      currentStateKey = record.parentStateKey
    }

    return path.reverse()
  }

  private buildCurrentPath(
    record: PathCandidateRecord,
    includeGoalSuffix: boolean,
  ): GridCellAddress[] {
    return stitchPathSegments(
      this.startPathPrefix,
      this.reconstructSearchPath(record.stateKey),
      includeGoalSuffix ? this.goalPathSuffix : [],
    )
  }

  private canTransitionToDirection(
    record: PathCandidateRecord,
    nextDirection: MoveDirection,
  ): boolean {
    if (isSameDirection(record.direction, nextDirection)) {
      return true
    }

    return record.directionRunLength >= MIN_DIRECTION_STREAK
  }

  private getNextDirectionRunLength(
    record: PathCandidateRecord,
    nextDirection: MoveDirection,
  ): number {
    if (isSameDirection(record.direction, nextDirection)) {
      return Math.min(MIN_DIRECTION_STREAK, record.directionRunLength + 1)
    }

    return 1
  }

  private canFinishFromRecord(record: PathCandidateRecord): boolean {
    if (
      !this.searchGoalCell ||
      record.cell.index !== this.searchGoalCell.index
    ) {
      return false
    }

    if (!this.goalApproachDirection) {
      return false
    }

    return (
      isSameDirection(record.direction, this.goalApproachDirection) ||
      record.directionRunLength >= MIN_DIRECTION_STREAK
    )
  }

  private updateStats(phase: string) {
    const currentRecord =
      this.currentCandidateStateKey !== null
        ? this.records.get(this.currentCandidateStateKey)
        : null

    this.stats = {
      phase,
      queueSize: this.openQueue.length,
      visitedCellCount: this.visitedCellIndices.size,
      exploredCandidateCount: this.exploredCandidateCount,
      currentCandidate: currentRecord
        ? `${currentRecord.cell.column},${currentRecord.cell.row}`
        : null,
      currentDirection: currentRecord
        ? getDirectionKey(currentRecord.direction)
        : null,
      currentDirectionRunLength: currentRecord?.directionRunLength ?? null,
      currentG: currentRecord?.g ?? null,
      currentH: currentRecord?.h ?? null,
      currentF: currentRecord?.f ?? null,
      currentPathLength: this.currentPath.length,
      searchStartCell: this.searchStartCell
        ? `${this.searchStartCell.column},${this.searchStartCell.row}`
        : null,
      searchGoalCell: this.searchGoalCell
        ? `${this.searchGoalCell.column},${this.searchGoalCell.row}`
        : null,
      greedyMultiplier: this.greedyMultiplier,
      obstacleSearchCells: this.obstacleSearchCells,
      obstacleProximityPenalty: this.obstacleProximityPenalty,
      ninetyDegreeTurnPenalty: this.ninetyDegreeTurnPenalty,
      initializationError: this.initializationError,
    }
  }

  override _step() {
    if (this.initializationError) {
      this.failed = true
      this.error = this.initializationError
      this.updateStats("failed")
      return
    }

    const nextStateKey = this.dequeueBestCandidateStateKey()

    if (nextStateKey === null) {
      this.failed = true
      this.error =
        "Unable to find a walkable bus path between the selected fanin and fanout cells."
      this.updateStats("failed")
      return
    }

    const currentRecord = this.records.get(nextStateKey)

    if (!currentRecord) {
      this.failed = true
      this.error = `Missing candidate record for state ${nextStateKey}.`
      this.updateStats("failed")
      return
    }

    this.currentCandidateStateKey = nextStateKey
    this.currentPath = this.buildCurrentPath(currentRecord, false)
    this.exploredCandidateCount += 1

    if (this.canFinishFromRecord(currentRecord)) {
      const finalPath = this.buildCurrentPath(currentRecord, true)

      this.currentPath = finalPath
      this.output = {
        faninCell: this.faninCell,
        fanoutCell: this.fanoutCell,
        path: finalPath,
        pathLength: finalPath.length,
        pathCost:
          currentRecord.g + this.getForcedSegmentCost(this.goalPathSuffix),
        exploredCandidateCount: this.exploredCandidateCount,
        visitedCellCount: this.visitedCellIndices.size,
        greedyMultiplier: this.greedyMultiplier,
        obstacleSearchCells: this.obstacleSearchCells,
        obstacleProximityPenalty: this.obstacleProximityPenalty,
        ninetyDegreeTurnPenalty: this.ninetyDegreeTurnPenalty,
      }
      this.solved = true
      this.updateStats("done")
      return
    }

    currentRecord.status = "closed"

    for (const direction of ALL_MOVE_DIRECTIONS) {
      if (!this.canTransitionToDirection(currentRecord, direction)) {
        continue
      }

      const neighborColumn = currentRecord.cell.column + direction.columnStep
      const neighborRow = currentRecord.cell.row + direction.rowStep

      if (!this.isWalkableCell(neighborColumn, neighborRow)) {
        continue
      }

      const neighborCell = getCellAddress(
        this.params.grid,
        neighborColumn,
        neighborRow,
      )
      const tentativeG =
        currentRecord.g +
        this.getTransitionCost(neighborCell, currentRecord, direction)
      const nextRecord = this.createCandidateRecord({
        cell: neighborCell,
        parentStateKey: currentRecord.stateKey,
        g: tentativeG,
        direction,
        directionRunLength: this.getNextDirectionRunLength(
          currentRecord,
          direction,
        ),
      })
      const existingRecord = this.records.get(nextRecord.stateKey)

      if (existingRecord && tentativeG >= existingRecord.g) {
        continue
      }

      this.records.set(nextRecord.stateKey, nextRecord)
      this.visitedCellIndices.add(neighborCell.index)

      if (!this.openQueue.includes(nextRecord.stateKey)) {
        this.openQueue.push(nextRecord.stateKey)
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
      this.currentCandidateStateKey !== null &&
      this.records.has(this.currentCandidateStateKey)
        ? [
            {
              center: this.records.get(this.currentCandidateStateKey)!.cell
                .center,
              radius: this.params.grid.cellSize * 0.28,
              fill: "rgba(249, 115, 22, 0.35)",
              stroke: CURRENT_CANDIDATE_COLOR,
              label: "current-candidate",
            },
          ]
        : []
    const extraTexts: NonNullable<GraphicsObject["texts"]> =
      this.currentCandidateStateKey !== null &&
      this.records.has(this.currentCandidateStateKey)
        ? [
            {
              x: this.records.get(this.currentCandidateStateKey)!.cell.center.x,
              y:
                this.records.get(this.currentCandidateStateKey)!.cell.center.y +
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
