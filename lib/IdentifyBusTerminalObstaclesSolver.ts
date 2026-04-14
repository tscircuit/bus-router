import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject, Rect } from "graphics-debug"

export type XYPoint = {
  x: number
  y: number
}

export interface BusObstacle {
  type: string
  center: XYPoint
  width: number
  height: number
  connectedTo?: string[]
  layers?: string[]
}

export interface BusConnectionPatch {
  connectionId: string
  pointIds: string[]
  _bus?: {
    id: string
  }
}

export interface BusDefinition {
  busId: string
  pointIds: string[]
  connectionPatches: BusConnectionPatch[]
}

export interface BusRouterInput {
  obstacles: BusObstacle[]
  bus: BusDefinition
  traceWidth?: number
  traceSpacing?: number
}

export interface BusObstacleGroup {
  obstacleIndices: number[]
  centroid: XYPoint
}

export interface BusTerminalObstacleDetectionOutput {
  candidateObstacleIndices: number[]
  busStart: BusObstacleGroup
  busEnd: BusObstacleGroup
}

const BUS_TERMINAL_STROKE = "#2563eb"
const BUS_TERMINAL_FILL = "rgba(37, 99, 235, 0.22)"
const OTHER_OBSTACLE_STROKE = "#dc2626"
const OTHER_OBSTACLE_FILL = "rgba(220, 38, 38, 0.18)"
const SUMMARY_TEXT_COLOR = "#0f172a"
const MAX_CLUSTERING_ITERATIONS = 12

const getSquaredDistance = (a: XYPoint, b: XYPoint): number => {
  const deltaX = a.x - b.x
  const deltaY = a.y - b.y
  return deltaX * deltaX + deltaY * deltaY
}

const computeCentroid = (
  obstacles: BusObstacle[],
  obstacleIndices: number[],
): XYPoint => {
  let totalX = 0
  let totalY = 0

  for (const obstacleIndex of obstacleIndices) {
    totalX += obstacles[obstacleIndex]?.center.x ?? 0
    totalY += obstacles[obstacleIndex]?.center.y ?? 0
  }

  return {
    x: totalX / obstacleIndices.length,
    y: totalY / obstacleIndices.length,
  }
}

const comparePointsLexicographically = (a: XYPoint, b: XYPoint): number => {
  if (a.x !== b.x) {
    return a.x - b.x
  }

  return a.y - b.y
}

const sortObstacleIndices = (obstacleIndices: number[]): number[] =>
  obstacleIndices.slice().sort((a, b) => a - b)

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

const getConnectedBusObstacleIndices = (
  obstacles: BusObstacle[],
  busPointIds: Set<string>,
): number[] => {
  const connectedObstacleIndices: number[] = []

  for (
    let obstacleIndex = 0;
    obstacleIndex < obstacles.length;
    obstacleIndex += 1
  ) {
    const obstacle = obstacles[obstacleIndex]

    if (
      obstacle?.connectedTo?.some((connectedId) => busPointIds.has(connectedId))
    ) {
      connectedObstacleIndices.push(obstacleIndex)
    }
  }

  return connectedObstacleIndices
}

const getFarthestPair = (
  obstacles: BusObstacle[],
  candidateObstacleIndices: number[],
): [number, number] => {
  let farthestPair: [number, number] = [
    candidateObstacleIndices[0]!,
    candidateObstacleIndices[1]!,
  ]
  let farthestDistance = Number.NEGATIVE_INFINITY

  for (let indexA = 0; indexA < candidateObstacleIndices.length; indexA += 1) {
    for (
      let indexB = indexA + 1;
      indexB < candidateObstacleIndices.length;
      indexB += 1
    ) {
      const obstacleIndexA = candidateObstacleIndices[indexA]!
      const obstacleIndexB = candidateObstacleIndices[indexB]!
      const distance = getSquaredDistance(
        obstacles[obstacleIndexA]!.center,
        obstacles[obstacleIndexB]!.center,
      )

      if (distance > farthestDistance) {
        farthestDistance = distance
        farthestPair = [obstacleIndexA, obstacleIndexB]
      }
    }
  }

  return farthestPair
}

const fallbackSplitCandidateObstacles = (
  obstacles: BusObstacle[],
  candidateObstacleIndices: number[],
): [number[], number[]] => {
  const sortedObstacleIndices = candidateObstacleIndices
    .slice()
    .sort((obstacleIndexA, obstacleIndexB) =>
      comparePointsLexicographically(
        obstacles[obstacleIndexA]!.center,
        obstacles[obstacleIndexB]!.center,
      ),
    )
  const midpoint = Math.floor(sortedObstacleIndices.length / 2)
  return [
    sortObstacleIndices(sortedObstacleIndices.slice(0, midpoint)),
    sortObstacleIndices(sortedObstacleIndices.slice(midpoint)),
  ]
}

const clusterCandidateObstacles = (
  obstacles: BusObstacle[],
  candidateObstacleIndices: number[],
): [number[], number[]] => {
  if (candidateObstacleIndices.length === 2) {
    return [[candidateObstacleIndices[0]!], [candidateObstacleIndices[1]!]]
  }

  const [seedA, seedB] = getFarthestPair(obstacles, candidateObstacleIndices)
  let centroidA = obstacles[seedA]!.center
  let centroidB = obstacles[seedB]!.center
  let clusterA: number[] = []
  let clusterB: number[] = []
  let previousSignature = ""

  for (
    let iteration = 0;
    iteration < MAX_CLUSTERING_ITERATIONS;
    iteration += 1
  ) {
    clusterA = []
    clusterB = []

    for (const obstacleIndex of candidateObstacleIndices) {
      const obstacle = obstacles[obstacleIndex]!
      const distanceToA = getSquaredDistance(obstacle.center, centroidA)
      const distanceToB = getSquaredDistance(obstacle.center, centroidB)

      if (distanceToA <= distanceToB) {
        clusterA.push(obstacleIndex)
      } else {
        clusterB.push(obstacleIndex)
      }
    }

    if (clusterA.length === 0 || clusterB.length === 0) {
      return fallbackSplitCandidateObstacles(
        obstacles,
        candidateObstacleIndices,
      )
    }

    const nextSignature = `${sortObstacleIndices(clusterA).join(",")}|${sortObstacleIndices(clusterB).join(",")}`
    centroidA = computeCentroid(obstacles, clusterA)
    centroidB = computeCentroid(obstacles, clusterB)

    if (nextSignature === previousSignature) {
      break
    }

    previousSignature = nextSignature
  }

  return [sortObstacleIndices(clusterA), sortObstacleIndices(clusterB)]
}

const createObstacleRects = (
  obstacles: BusObstacle[],
  highlightedObstacleIndices: Set<number>,
): Rect[] => {
  const rects: Rect[] = []

  for (
    let obstacleIndex = 0;
    obstacleIndex < obstacles.length;
    obstacleIndex += 1
  ) {
    const obstacle = obstacles[obstacleIndex]!
    const isHighlighted = highlightedObstacleIndices.has(obstacleIndex)

    rects.push({
      center: obstacle.center,
      width: obstacle.width,
      height: obstacle.height,
      stroke: isHighlighted ? BUS_TERMINAL_STROKE : OTHER_OBSTACLE_STROKE,
      fill: isHighlighted ? BUS_TERMINAL_FILL : OTHER_OBSTACLE_FILL,
    })
  }

  return rects
}

const createSummaryTexts = (
  obstacles: BusObstacle[],
  lines: string[],
): NonNullable<GraphicsObject["texts"]> => {
  if (obstacles.length === 0 || lines.length === 0) {
    return []
  }

  const bounds = getObstacleBounds(obstacles)
  const texts: NonNullable<GraphicsObject["texts"]> = []

  for (let index = 0; index < lines.length; index += 1) {
    const text = lines[index]!

    texts.push({
      x: bounds.minX,
      y: bounds.maxY + 4 + index * 3,
      text,
      fontSize: 8,
      color: SUMMARY_TEXT_COLOR,
    })
  }

  return texts
}

export const createBusTerminalObstacleVisualization = (params: {
  obstacles: BusObstacle[]
  highlightedObstacleIndices: Set<number>
  summaryLines: string[]
  busStartCentroid?: XYPoint
  busEndCentroid?: XYPoint
  title: string
}): GraphicsObject => {
  const texts = createSummaryTexts(params.obstacles, params.summaryLines)

  if (params.busStartCentroid) {
    texts.push({
      x: params.busStartCentroid.x,
      y: params.busStartCentroid.y + 3,
      text: "Bus Start",
      fontSize: 10,
      color: BUS_TERMINAL_STROKE,
    })
  }

  if (params.busEndCentroid) {
    texts.push({
      x: params.busEndCentroid.x,
      y: params.busEndCentroid.y + 3,
      text: "Bus End",
      fontSize: 10,
      color: BUS_TERMINAL_STROKE,
    })
  }

  return {
    title: params.title,
    coordinateSystem: "cartesian",
    points: [],
    lines: [],
    rects: createObstacleRects(
      params.obstacles,
      params.highlightedObstacleIndices,
    ),
    circles: [],
    texts,
  }
}

export class IdentifyBusTerminalObstaclesSolver extends BaseSolver {
  private candidateObstacleIndices: number[] = []
  private output: BusTerminalObstacleDetectionOutput | null = null

  constructor(private readonly inputProblem: BusRouterInput) {
    super()
    this.MAX_ITERATIONS = 2
    this.stats = {
      totalObstacles: inputProblem.obstacles.length,
      busPointCount: inputProblem.bus.pointIds.length,
      candidateObstacleCount: 0,
      busStartObstacleCount: 0,
      busEndObstacleCount: 0,
      phase: "scan-obstacles",
    }
  }

  computeProgress(): number {
    if (this.solved) {
      return 1
    }

    if (this.candidateObstacleIndices.length > 0) {
      return 0.5
    }

    return 0
  }

  override _step() {
    if (this.iterations === 1) {
      const busPointIds = new Set(this.inputProblem.bus.pointIds)
      this.candidateObstacleIndices = getConnectedBusObstacleIndices(
        this.inputProblem.obstacles,
        busPointIds,
      )
      this.stats = {
        totalObstacles: this.inputProblem.obstacles.length,
        busPointCount: this.inputProblem.bus.pointIds.length,
        candidateObstacleCount: this.candidateObstacleIndices.length,
        busStartObstacleCount: 0,
        busEndObstacleCount: 0,
        phase: "cluster-candidates",
      }

      if (this.candidateObstacleIndices.length < 2) {
        this.error = `Unable to find two bus terminal obstacle groups; found ${this.candidateObstacleIndices.length} bus-connected obstacles.`
        this.failed = true
      }

      return
    }

    const [clusterAIndices, clusterBIndices] = clusterCandidateObstacles(
      this.inputProblem.obstacles,
      this.candidateObstacleIndices,
    )
    const clusterA = {
      obstacleIndices: clusterAIndices,
      centroid: computeCentroid(this.inputProblem.obstacles, clusterAIndices),
    }
    const clusterB = {
      obstacleIndices: clusterBIndices,
      centroid: computeCentroid(this.inputProblem.obstacles, clusterBIndices),
    }
    const [busStart, busEnd] =
      comparePointsLexicographically(clusterA.centroid, clusterB.centroid) <= 0
        ? [clusterA, clusterB]
        : [clusterB, clusterA]

    this.output = {
      candidateObstacleIndices: sortObstacleIndices(
        this.candidateObstacleIndices,
      ),
      busStart,
      busEnd,
    }
    this.stats = {
      totalObstacles: this.inputProblem.obstacles.length,
      busPointCount: this.inputProblem.bus.pointIds.length,
      candidateObstacleCount: this.output.candidateObstacleIndices.length,
      busStartObstacleCount: this.output.busStart.obstacleIndices.length,
      busEndObstacleCount: this.output.busEnd.obstacleIndices.length,
      phase: "done",
    }
    this.solved = true
  }

  override visualize(): GraphicsObject {
    const highlightedObstacleIndices = new Set<number>(
      this.output
        ? [
            ...this.output.busStart.obstacleIndices,
            ...this.output.busEnd.obstacleIndices,
          ]
        : this.candidateObstacleIndices,
    )
    const summaryLines = [
      `Candidate bus obstacles: ${this.candidateObstacleIndices.length}`,
      this.output
        ? `Bus start/end: ${this.output.busStart.obstacleIndices.length}/${this.output.busEnd.obstacleIndices.length}`
        : "Bus start/end: pending split",
    ]

    if (this.failed && this.error) {
      summaryLines.push(`Error: ${this.error}`)
    }

    return createBusTerminalObstacleVisualization({
      obstacles: this.inputProblem.obstacles,
      highlightedObstacleIndices,
      summaryLines,
      busStartCentroid: this.output?.busStart.centroid,
      busEndCentroid: this.output?.busEnd.centroid,
      title: "Stage 1 - Identify Bus Terminals",
    })
  }

  override getOutput(): BusTerminalObstacleDetectionOutput | null {
    return this.output
  }

  override getConstructorParams() {
    return [this.inputProblem]
  }
}
