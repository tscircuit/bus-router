import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import type {
  BusRouterInput,
  XYPoint,
} from "./IdentifyBusTerminalObstaclesSolver"
import type { FindBusPathOutput } from "./FindBusPathSolver"
import {
  type FindFanoutStartEndOutput,
  createFindFanoutStartEndVisualization,
} from "./FindFanoutStartEndSolver"
import type { GridBuilderOutput, GridCellAddress } from "./GridBuilderSolver"

const DEFAULT_TURN_SEGMENTS_PER_QUARTER_TURN = 6
const CENTERLINE_COLOR = "rgba(15, 23, 42, 0.38)"
const TRACE_LABEL_COLOR = "#0f172a"

interface GridStep {
  columnStep: number
  rowStep: number
}

interface Vector2 {
  x: number
  y: number
}

interface CenterlineRun {
  startCell: GridCellAddress
  endCell: GridCellAddress
  start: XYPoint
  end: XYPoint
  step: GridStep
  directionVector: Vector2
  normalVector: Vector2
}

interface OffsetJoin {
  currentRunEndPoint: XYPoint
  nextRunStartPoint: XYPoint
  arcPoints: XYPoint[]
}

export interface SplitIntoTracePathsSolverInput {
  inputProblem: BusRouterInput
  grid: GridBuilderOutput
  fanoutStartEnd: FindFanoutStartEndOutput
  busPath: FindBusPathOutput
  turnSegmentsPerQuarterTurn?: number
}

export interface SplitTracePath {
  traceIndex: number
  offsetFromCenterline: number
  points: XYPoint[]
  approximateLength: number
}

export interface SplitIntoTracePathsOutput extends FindBusPathOutput {
  traceCount: number
  traceWidth: number
  traceSpacing: number
  tracePitch: number
  centerlinePath: XYPoint[]
  centerlineRunCount: number
  turnCount: number
  turnSegmentsPerQuarterTurn: number
  tracePaths: SplitTracePath[]
}

const EPSILON = 1e-6

const isSameGridStep = (stepA: GridStep, stepB: GridStep): boolean =>
  stepA.columnStep === stepB.columnStep && stepA.rowStep === stepB.rowStep

const getTraceColor = (traceIndex: number, traceCount: number): string => {
  if (traceCount <= 1) {
    return "hsl(205 75% 42%)"
  }

  const fraction = traceIndex / (traceCount - 1)
  const hue = 208 - 180 * fraction
  return `hsl(${hue.toFixed(1)} 72% 44%)`
}

const getStepVector = (fromCell: GridCellAddress, toCell: GridCellAddress): GridStep => ({
  columnStep: toCell.column - fromCell.column,
  rowStep: toCell.row - fromCell.row,
})

const getUnitDirectionVector = (step: GridStep): Vector2 => {
  const length = Math.hypot(step.columnStep, step.rowStep)

  return {
    x: step.columnStep / length,
    y: step.rowStep / length,
  }
}

const getNormalVector = (directionVector: Vector2): Vector2 => ({
  x: -directionVector.y,
  y: directionVector.x,
})

const dotProduct = (vectorA: Vector2, vectorB: Vector2): number =>
  vectorA.x * vectorB.x + vectorA.y * vectorB.y

const crossProduct = (vectorA: Vector2, vectorB: Vector2): number =>
  vectorA.x * vectorB.y - vectorA.y * vectorB.x

const offsetPoint = (
  point: XYPoint,
  normalVector: Vector2,
  offset: number,
): XYPoint => ({
  x: point.x + normalVector.x * offset,
  y: point.y + normalVector.y * offset,
})

const arePointsNear = (pointA: XYPoint, pointB: XYPoint): boolean =>
  Math.abs(pointA.x - pointB.x) < EPSILON &&
  Math.abs(pointA.y - pointB.y) < EPSILON

const pushPointIfDistinct = (points: XYPoint[], point: XYPoint) => {
  if (points.length === 0 || !arePointsNear(points[points.length - 1]!, point)) {
    points.push(point)
  }
}

const getPolylineLength = (points: XYPoint[]): number => {
  let totalLength = 0

  for (let index = 1; index < points.length; index += 1) {
    const previousPoint = points[index - 1]!
    const nextPoint = points[index]!
    totalLength += Math.hypot(
      nextPoint.x - previousPoint.x,
      nextPoint.y - previousPoint.y,
    )
  }

  return totalLength
}

const buildCenterlineRuns = (path: GridCellAddress[]): CenterlineRun[] => {
  if (path.length < 2) {
    return []
  }

  const runs: CenterlineRun[] = []
  let runStartIndex = 0
  let currentStep = getStepVector(path[0]!, path[1]!)

  for (let index = 1; index < path.length - 1; index += 1) {
    const nextStep = getStepVector(path[index]!, path[index + 1]!)

    if (isSameGridStep(currentStep, nextStep)) {
      continue
    }

    const directionVector = getUnitDirectionVector(currentStep)

    runs.push({
      startCell: path[runStartIndex]!,
      endCell: path[index]!,
      start: path[runStartIndex]!.center,
      end: path[index]!.center,
      step: currentStep,
      directionVector,
      normalVector: getNormalVector(directionVector),
    })

    runStartIndex = index
    currentStep = nextStep
  }

  const directionVector = getUnitDirectionVector(currentStep)

  runs.push({
    startCell: path[runStartIndex]!,
    endCell: path[path.length - 1]!,
    start: path[runStartIndex]!.center,
    end: path[path.length - 1]!.center,
    step: currentStep,
    directionVector,
    normalVector: getNormalVector(directionVector),
  })

  return runs
}

const intersectLines = (params: {
  lineAPoint: XYPoint
  lineADirection: Vector2
  lineBPoint: XYPoint
  lineBDirection: Vector2
}): XYPoint | null => {
  const denominator = crossProduct(
    params.lineADirection,
    params.lineBDirection,
  )

  if (Math.abs(denominator) < EPSILON) {
    return null
  }

  const delta = {
    x: params.lineBPoint.x - params.lineAPoint.x,
    y: params.lineBPoint.y - params.lineAPoint.y,
  }
  const lineAFactor = crossProduct(delta, params.lineBDirection) / denominator

  return {
    x: params.lineAPoint.x + params.lineADirection.x * lineAFactor,
    y: params.lineAPoint.y + params.lineADirection.y * lineAFactor,
  }
}

const createRoundJoinPoints = (params: {
  vertex: XYPoint
  previousNormal: Vector2
  nextNormal: Vector2
  offset: number
  turnAngle: number
  turnSegmentsPerQuarterTurn: number
}): XYPoint[] => {
  const previousJoinPoint = offsetPoint(
    params.vertex,
    params.previousNormal,
    params.offset,
  )
  const nextJoinPoint = offsetPoint(params.vertex, params.nextNormal, params.offset)

  if (Math.abs(params.offset) < EPSILON) {
    return [nextJoinPoint]
  }

  const radius = Math.abs(params.offset)
  const startAngle = Math.atan2(
    previousJoinPoint.y - params.vertex.y,
    previousJoinPoint.x - params.vertex.x,
  )
  const segmentCount = Math.max(
    2,
    Math.ceil(
      (Math.abs(params.turnAngle) / (Math.PI / 2)) *
        params.turnSegmentsPerQuarterTurn,
    ),
  )
  const points: XYPoint[] = []

  for (let segmentIndex = 1; segmentIndex <= segmentCount; segmentIndex += 1) {
    const fraction = segmentIndex / segmentCount
    const angle = startAngle + params.turnAngle * fraction

    pushPointIfDistinct(points, {
      x: params.vertex.x + Math.cos(angle) * radius,
      y: params.vertex.y + Math.sin(angle) * radius,
    })
  }

  pushPointIfDistinct(points, nextJoinPoint)
  return points
}

const buildOffsetJoin = (params: {
  previousRun: CenterlineRun
  nextRun: CenterlineRun
  offset: number
  turnSegmentsPerQuarterTurn: number
}): OffsetJoin => {
  const turnCross = crossProduct(
    params.previousRun.directionVector,
    params.nextRun.directionVector,
  )
  const turnAngle = Math.atan2(
    turnCross,
    dotProduct(params.previousRun.directionVector, params.nextRun.directionVector),
  )
  const previousJoinPoint = offsetPoint(
    params.nextRun.start,
    params.previousRun.normalVector,
    params.offset,
  )
  const nextJoinPoint = offsetPoint(
    params.nextRun.start,
    params.nextRun.normalVector,
    params.offset,
  )
  const isOuterCorner = params.offset * turnCross < 0

  if (!isOuterCorner || Math.abs(params.offset) < EPSILON) {
    const intersectionPoint = intersectLines({
      lineAPoint: previousJoinPoint,
      lineADirection: params.previousRun.directionVector,
      lineBPoint: nextJoinPoint,
      lineBDirection: params.nextRun.directionVector,
    })
    const sharedJoinPoint = intersectionPoint ?? previousJoinPoint

    return {
      currentRunEndPoint: sharedJoinPoint,
      nextRunStartPoint: sharedJoinPoint,
      arcPoints: [],
    }
  }

  return {
    currentRunEndPoint: previousJoinPoint,
    nextRunStartPoint: nextJoinPoint,
    arcPoints: createRoundJoinPoints({
      vertex: params.nextRun.start,
      previousNormal: params.previousRun.normalVector,
      nextNormal: params.nextRun.normalVector,
      offset: params.offset,
      turnAngle,
      turnSegmentsPerQuarterTurn: params.turnSegmentsPerQuarterTurn,
    }),
  }
}

const buildTracePathPoints = (params: {
  runs: CenterlineRun[]
  offset: number
  turnSegmentsPerQuarterTurn: number
}): XYPoint[] => {
  if (params.runs.length === 0) {
    return []
  }

  const points: XYPoint[] = []
  const joins: OffsetJoin[] = []

  for (let runIndex = 1; runIndex < params.runs.length; runIndex += 1) {
    joins.push(
      buildOffsetJoin({
        previousRun: params.runs[runIndex - 1]!,
        nextRun: params.runs[runIndex]!,
        offset: params.offset,
        turnSegmentsPerQuarterTurn: params.turnSegmentsPerQuarterTurn,
      }),
    )
  }

  for (let runIndex = 0; runIndex < params.runs.length; runIndex += 1) {
    const run = params.runs[runIndex]!
    const startPoint =
      runIndex === 0
        ? offsetPoint(run.start, run.normalVector, params.offset)
        : joins[runIndex - 1]!.nextRunStartPoint
    const endPoint =
      runIndex === params.runs.length - 1
        ? offsetPoint(run.end, run.normalVector, params.offset)
        : joins[runIndex]!.currentRunEndPoint

    pushPointIfDistinct(points, startPoint)
    pushPointIfDistinct(points, endPoint)

    if (runIndex < joins.length) {
      for (const arcPoint of joins[runIndex]!.arcPoints) {
        pushPointIfDistinct(points, arcPoint)
      }
    }
  }

  return points
}

export const createSplitIntoTracePathsVisualization = (params: {
  inputProblem: BusRouterInput
  grid: GridBuilderOutput
  fanoutStartEnd: FindFanoutStartEndOutput
  output: SplitIntoTracePathsOutput
  title: string
}): GraphicsObject => {
  const traceLines: NonNullable<GraphicsObject["lines"]> =
    params.output.tracePaths.map((tracePath) => ({
      points: tracePath.points,
      strokeColor: getTraceColor(tracePath.traceIndex, params.output.traceCount),
      strokeWidth: params.output.traceWidth,
      label: "trace-path",
    }))
  const traceTexts: NonNullable<GraphicsObject["texts"]> =
    params.output.tracePaths.map((tracePath) => ({
      x: tracePath.points[0]!.x,
      y: tracePath.points[0]!.y + params.grid.cellSize * 0.65,
      text: `Trace ${tracePath.traceIndex + 1}`,
      fontSize: Math.max(4, params.grid.cellSize * 0.45),
      color: TRACE_LABEL_COLOR,
    }))

  return createFindFanoutStartEndVisualization({
    inputProblem: params.inputProblem,
    grid: params.grid,
    output: params.fanoutStartEnd,
    title: params.title,
    extraLines: [
      {
        points: params.output.centerlinePath,
        strokeColor: CENTERLINE_COLOR,
        strokeWidth: Math.max(0.1, params.output.traceWidth * 0.35),
        label: "bus-centerline-path",
      },
      ...traceLines,
    ],
    extraTexts: traceTexts,
  })
}

export class SplitIntoTracePathsSolver extends BaseSolver {
  private output: SplitIntoTracePathsOutput | null = null

  constructor(private readonly params: SplitIntoTracePathsSolverInput) {
    super()
    this.MAX_ITERATIONS = 1
    this.stats = {
      phase: "split-into-trace-paths",
      traceCount: params.grid.traceCount,
      turnSegmentsPerQuarterTurn:
        params.turnSegmentsPerQuarterTurn ??
        DEFAULT_TURN_SEGMENTS_PER_QUARTER_TURN,
    }
  }

  override _step() {
    const centerlinePath = this.params.busPath.path.map((cell) => cell.center)
    const centerlineRuns = buildCenterlineRuns(this.params.busPath.path)

    if (centerlineRuns.length === 0) {
      this.failed = true
      this.error =
        "Unable to split the bus path into trace paths because the centerline path is too short."
      return
    }

    const tracePitch = this.params.grid.traceWidth + this.params.grid.traceSpacing
    const traceCenter = (this.params.grid.traceCount - 1) / 2
    const turnSegmentsPerQuarterTurn = Math.max(
      2,
      this.params.turnSegmentsPerQuarterTurn ??
        DEFAULT_TURN_SEGMENTS_PER_QUARTER_TURN,
    )
    const tracePaths: SplitTracePath[] = []

    for (
      let traceIndex = 0;
      traceIndex < this.params.grid.traceCount;
      traceIndex += 1
    ) {
      const offsetFromCenterline = (traceIndex - traceCenter) * tracePitch
      const points = buildTracePathPoints({
        runs: centerlineRuns,
        offset: offsetFromCenterline,
        turnSegmentsPerQuarterTurn,
      })

      tracePaths.push({
        traceIndex,
        offsetFromCenterline,
        points,
        approximateLength: getPolylineLength(points),
      })
    }

    this.output = {
      ...this.params.busPath,
      traceCount: this.params.grid.traceCount,
      traceWidth: this.params.grid.traceWidth,
      traceSpacing: this.params.grid.traceSpacing,
      tracePitch,
      centerlinePath,
      centerlineRunCount: centerlineRuns.length,
      turnCount: Math.max(0, centerlineRuns.length - 1),
      turnSegmentsPerQuarterTurn,
      tracePaths,
    }
    this.stats = {
      phase: "done",
      traceCount: tracePaths.length,
      tracePitch,
      centerlineRunCount: centerlineRuns.length,
      turnCount: Math.max(0, centerlineRuns.length - 1),
      turnSegmentsPerQuarterTurn,
      firstTraceLength: tracePaths[0]?.approximateLength ?? 0,
      lastTraceLength:
        tracePaths[tracePaths.length - 1]?.approximateLength ?? 0,
    }
    this.solved = true
  }

  override visualize(): GraphicsObject {
    if (!this.output) {
      return {
        title: "Stage 5 - Split Into Trace Paths",
        coordinateSystem: "cartesian",
        points: [],
        lines: [],
        rects: [],
        circles: [],
        texts: [],
      }
    }

    return createSplitIntoTracePathsVisualization({
      inputProblem: this.params.inputProblem,
      grid: this.params.grid,
      fanoutStartEnd: this.params.fanoutStartEnd,
      output: this.output,
      title: "Stage 5 - Split Into Trace Paths",
    })
  }

  override getOutput(): SplitIntoTracePathsOutput | null {
    return this.output
  }

  override getConstructorParams() {
    return [this.params]
  }
}

export { SplitIntoTracePathsSolver as SplitIntoTracePaths }
