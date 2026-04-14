import { expect, test } from "bun:test"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import { FindBusPathSolver } from "lib/FindBusPathSolver"
import { FindFanoutStartEndSolver } from "lib/FindFanoutStartEndSolver"
import { GridBuilderSolver, GridCellFlags } from "lib/GridBuilderSolver"
import { IdentifyBusTerminalObstaclesSolver } from "lib/IdentifyBusTerminalObstaclesSolver"
import { SplitIntoTracePathsSolver } from "lib/SplitIntoTracePathsSolver"
import exampleBus from "./assets/CM5IO_bus1.json"
import exampleSrj from "./assets/CM5IO.srj.json"

const getCandidateDirectionVector = (
  orientation: "horizontal" | "vertical",
  direction: "negative" | "positive",
) =>
  orientation === "horizontal"
    ? {
        columnStep: 0,
        rowStep: direction === "negative" ? -1 : 1,
      }
    : {
        columnStep: direction === "negative" ? -1 : 1,
        rowStep: 0,
      }

const invertDirectionVector = (direction: {
  columnStep: number
  rowStep: number
}) => ({
  columnStep: direction.columnStep === 0 ? 0 : -direction.columnStep,
  rowStep: direction.rowStep === 0 ? 0 : -direction.rowStep,
})

const getPathDeltas = (
  path: Array<{
    column: number
    row: number
  }>,
) =>
  path.slice(1).map((cell, index) => ({
    columnStep: cell.column - path[index]!.column,
    rowStep: cell.row - path[index]!.row,
  }))

const getDirectionRuns = (
  deltas: Array<{
    columnStep: number
    rowStep: number
  }>,
) => {
  if (deltas.length === 0) {
    return []
  }

  const runs: Array<{
    columnStep: number
    rowStep: number
    length: number
  }> = []
  let currentDelta = deltas[0]!
  let currentLength = 1

  for (let index = 1; index < deltas.length; index += 1) {
    const nextDelta = deltas[index]!

    if (
      nextDelta.columnStep === currentDelta.columnStep &&
      nextDelta.rowStep === currentDelta.rowStep
    ) {
      currentLength += 1
      continue
    }

    runs.push({
      columnStep: currentDelta.columnStep,
      rowStep: currentDelta.rowStep,
      length: currentLength,
    })
    currentDelta = nextDelta
    currentLength = 1
  }

  runs.push({
    columnStep: currentDelta.columnStep,
    rowStep: currentDelta.rowStep,
    length: currentLength,
  })

  return runs
}

const getPointDistance = (
  pointA: {
    x: number
    y: number
  },
  pointB: {
    x: number
    y: number
  },
) => Math.hypot(pointA.x - pointB.x, pointA.y - pointB.y)

const getDotProduct = (
  vectorA: {
    x: number
    y: number
  },
  vectorB: {
    x: number
    y: number
  },
) => vectorA.x * vectorB.x + vectorA.y * vectorB.y

const subtractPoints = (
  pointA: {
    x: number
    y: number
  },
  pointB: {
    x: number
    y: number
  },
) => ({
  x: pointA.x - pointB.x,
  y: pointA.y - pointB.y,
})

const clamp = (value: number, min: number, max: number) =>
  Math.max(min, Math.min(max, value))

const getPointToSegmentDistance = (
  point: {
    x: number
    y: number
  },
  segmentStart: {
    x: number
    y: number
  },
  segmentEnd: {
    x: number
    y: number
  },
) => {
  const segment = subtractPoints(segmentEnd, segmentStart)
  const offset = subtractPoints(point, segmentStart)
  const denominator = getDotProduct(segment, segment)

  if (denominator === 0) {
    return getPointDistance(point, segmentStart)
  }

  const factor = clamp(getDotProduct(offset, segment) / denominator, 0, 1)

  return getPointDistance(point, {
    x: segmentStart.x + segment.x * factor,
    y: segmentStart.y + segment.y * factor,
  })
}

const getOrientation = (
  pointA: {
    x: number
    y: number
  },
  pointB: {
    x: number
    y: number
  },
  pointC: {
    x: number
    y: number
  },
) =>
  (pointB.x - pointA.x) * (pointC.y - pointA.y) -
  (pointB.y - pointA.y) * (pointC.x - pointA.x)

const isPointOnSegment = (
  segmentStart: {
    x: number
    y: number
  },
  segmentEnd: {
    x: number
    y: number
  },
  point: {
    x: number
    y: number
  },
) =>
  point.x >= Math.min(segmentStart.x, segmentEnd.x) - 1e-6 &&
  point.x <= Math.max(segmentStart.x, segmentEnd.x) + 1e-6 &&
  point.y >= Math.min(segmentStart.y, segmentEnd.y) - 1e-6 &&
  point.y <= Math.max(segmentStart.y, segmentEnd.y) + 1e-6

const doSegmentsIntersect = (
  segmentAStart: {
    x: number
    y: number
  },
  segmentAEnd: {
    x: number
    y: number
  },
  segmentBStart: {
    x: number
    y: number
  },
  segmentBEnd: {
    x: number
    y: number
  },
) => {
  const orientation1 = getOrientation(segmentAStart, segmentAEnd, segmentBStart)
  const orientation2 = getOrientation(segmentAStart, segmentAEnd, segmentBEnd)
  const orientation3 = getOrientation(segmentBStart, segmentBEnd, segmentAStart)
  const orientation4 = getOrientation(segmentBStart, segmentBEnd, segmentAEnd)

  if (
    ((orientation1 > 1e-6 && orientation2 < -1e-6) ||
      (orientation1 < -1e-6 && orientation2 > 1e-6)) &&
    ((orientation3 > 1e-6 && orientation4 < -1e-6) ||
      (orientation3 < -1e-6 && orientation4 > 1e-6))
  ) {
    return true
  }

  if (
    Math.abs(orientation1) <= 1e-6 &&
    isPointOnSegment(segmentAStart, segmentAEnd, segmentBStart)
  ) {
    return true
  }

  if (
    Math.abs(orientation2) <= 1e-6 &&
    isPointOnSegment(segmentAStart, segmentAEnd, segmentBEnd)
  ) {
    return true
  }

  if (
    Math.abs(orientation3) <= 1e-6 &&
    isPointOnSegment(segmentBStart, segmentBEnd, segmentAStart)
  ) {
    return true
  }

  if (
    Math.abs(orientation4) <= 1e-6 &&
    isPointOnSegment(segmentBStart, segmentBEnd, segmentAEnd)
  ) {
    return true
  }

  return false
}

const getSegmentToSegmentDistance = (
  segmentAStart: {
    x: number
    y: number
  },
  segmentAEnd: {
    x: number
    y: number
  },
  segmentBStart: {
    x: number
    y: number
  },
  segmentBEnd: {
    x: number
    y: number
  },
) => {
  if (
    doSegmentsIntersect(segmentAStart, segmentAEnd, segmentBStart, segmentBEnd)
  ) {
    return 0
  }

  return Math.min(
    getPointToSegmentDistance(segmentAStart, segmentBStart, segmentBEnd),
    getPointToSegmentDistance(segmentAEnd, segmentBStart, segmentBEnd),
    getPointToSegmentDistance(segmentBStart, segmentAStart, segmentAEnd),
    getPointToSegmentDistance(segmentBEnd, segmentAStart, segmentAEnd),
  )
}

const getMinimumTracePathDistance = (
  tracePaths: Array<{
    points: Array<{
      x: number
      y: number
    }>
  }>,
) => {
  let minimumDistance = Number.POSITIVE_INFINITY

  for (let pathIndexA = 0; pathIndexA < tracePaths.length; pathIndexA += 1) {
    for (
      let pathIndexB = pathIndexA + 1;
      pathIndexB < tracePaths.length;
      pathIndexB += 1
    ) {
      const pointsA = tracePaths[pathIndexA]!.points
      const pointsB = tracePaths[pathIndexB]!.points

      for (let segmentAIndex = 1; segmentAIndex < pointsA.length; segmentAIndex += 1) {
        for (
          let segmentBIndex = 1;
          segmentBIndex < pointsB.length;
          segmentBIndex += 1
        ) {
          minimumDistance = Math.min(
            minimumDistance,
            getSegmentToSegmentDistance(
              pointsA[segmentAIndex - 1]!,
              pointsA[segmentAIndex]!,
              pointsB[segmentBIndex - 1]!,
              pointsB[segmentBIndex]!,
            ),
          )
        }
      }
    }
  }

  return minimumDistance
}

const exampleInput = {
  obstacles: exampleSrj.obstacles,
  bus: exampleBus,
  traceWidth: 0.1,
  traceSpacing: 0.5,
}

test("IdentifyBusTerminalObstaclesSolver splits the example bus into two obstacle groups", () => {
  const solver = new IdentifyBusTerminalObstaclesSolver(exampleInput)

  solver.solve()

  const output = solver.getOutput()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(output?.candidateObstacleIndices).toHaveLength(18)
  expect(output?.busStart.obstacleIndices).toHaveLength(9)
  expect(output?.busEnd.obstacleIndices).toHaveLength(9)
  expect(output?.busStart.centroid.x).toBeLessThan(
    output?.busEnd.centroid.x ?? 0,
  )
})

test("BusRoutePipeline initialVisualize shows the SRJ obstacle set before pipeline stages", () => {
  const solver = new BusRoutePipeline(exampleInput)

  const visualization = solver.initialVisualize()

  expect(visualization).not.toBeNull()
  expect(visualization?.title).toBe("Initial SRJ")
  expect(visualization?.rects).toHaveLength(exampleSrj.obstacles.length)
  expect(
    visualization?.texts?.some((text) =>
      text.text.startsWith(`SRJ obstacles: ${exampleSrj.obstacles.length}`),
    ),
  ).toBe(true)
})

test("GridBuilderSolver builds a half-bus-size grid and keeps bus-connected obstacles out of obstacle occupancy", () => {
  const identifySolver = new IdentifyBusTerminalObstaclesSolver(exampleInput)

  identifySolver.solve()

  const terminalObstacles = identifySolver.getOutput()

  expect(terminalObstacles).not.toBeNull()

  const solver = new GridBuilderSolver({
    inputProblem: exampleInput,
    terminalObstacles: terminalObstacles!,
  })

  solver.solve()

  const output = solver.getOutput()
  const visualization = solver.visualize()
  const startAreaCenterFlags =
    output?.grid[output?.startArea.centerCell.index ?? 0] ?? 0
  const endAreaCenterFlags =
    output?.grid[output?.endArea.centerCell.index ?? 0] ?? 0

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(output?.grid).toBeInstanceOf(Int32Array)
  expect(output?.traceCount).toBe(9)
  expect(output?.traceWidth).toBeCloseTo(0.1)
  expect(output?.traceSpacing).toBeCloseTo(0.5)
  expect(output?.requiredBusWidth).toBeCloseTo(4.9)
  expect(output?.cellSize).toBeCloseTo(2.45)
  expect(output?.gridWidth).toBeGreaterThan(0)
  expect(output?.gridHeight).toBeGreaterThan(0)
  expect(output?.obstacleCellCount).toBeGreaterThan(0)
  expect(output?.busConnectedObstacleIndices).toHaveLength(18)
  expect(startAreaCenterFlags & GridCellFlags.obstacle).toBe(0)
  expect(endAreaCenterFlags & GridCellFlags.obstacle).toBe(0)
  expect(startAreaCenterFlags & GridCellFlags.startArea).toBe(
    GridCellFlags.startArea,
  )
  expect(endAreaCenterFlags & GridCellFlags.endArea).toBe(GridCellFlags.endArea)
  expect(
    visualization.rects?.filter(
      (rect) => rect.label === "bus-start-area-obstacle",
    ),
  ).toHaveLength(9)
  expect(
    visualization.rects?.filter(
      (rect) => rect.label === "bus-end-area-obstacle",
    ),
  ).toHaveLength(9)
  expect(
    visualization.texts?.some((text) => text.text === "Bus Start Area"),
  ).toBe(true)
  expect(
    visualization.texts?.some((text) => text.text === "Bus End Area"),
  ).toBe(true)
})

test("FindFanoutStartEndSolver finds candidate fanout and fanin positions and selects the closest pair", () => {
  const identifySolver = new IdentifyBusTerminalObstaclesSolver(exampleInput)

  identifySolver.solve()

  const gridBuilderSolver = new GridBuilderSolver({
    inputProblem: exampleInput,
    terminalObstacles: identifySolver.getOutput()!,
  })

  gridBuilderSolver.solve()

  const solver = new FindFanoutStartEndSolver({
    inputProblem: exampleInput,
    grid: gridBuilderSolver.getOutput()!,
  })

  solver.solve()

  const output = solver.getOutput()
  const visualization = solver.visualize()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(output?.fanoutCandidates.length).toBeGreaterThan(0)
  expect(output?.faninCandidates.length).toBeGreaterThan(0)
  expect(output?.selectedDistance).toBeGreaterThan(0)
  expect(
    visualization.lines?.some((line) => line.label === "fanout-area-line"),
  ).toBe(true)
  expect(
    visualization.lines?.some((line) => line.label === "fanin-area-line"),
  ).toBe(true)
  expect(
    visualization.lines?.some(
      (line) => line.label === "selected-fanout-fanin-pair",
    ),
  ).toBe(true)
  expect(
    visualization.circles?.filter(
      (circle) => circle.label === "fanout-candidate",
    ),
  ).toHaveLength(output?.fanoutCandidates.length ?? 0)
  expect(
    visualization.circles?.filter(
      (circle) => circle.label === "fanin-candidate",
    ),
  ).toHaveLength(output?.faninCandidates.length ?? 0)
  expect(
    visualization.circles?.some((circle) => circle.label === "selected-fanout"),
  ).toBe(true)
  expect(
    visualization.circles?.some((circle) => circle.label === "selected-fanin"),
  ).toBe(true)
})

test("FindBusPathSolver steps through A* candidates and enforces diagonal and direction-run path constraints", () => {
  const identifySolver = new IdentifyBusTerminalObstaclesSolver(exampleInput)

  identifySolver.solve()

  const gridBuilderSolver = new GridBuilderSolver({
    inputProblem: exampleInput,
    terminalObstacles: identifySolver.getOutput()!,
  })

  gridBuilderSolver.solve()

  const fanoutStartEndSolver = new FindFanoutStartEndSolver({
    inputProblem: exampleInput,
    grid: gridBuilderSolver.getOutput()!,
  })

  fanoutStartEndSolver.solve()

  const solver = new FindBusPathSolver({
    inputProblem: exampleInput,
    grid: gridBuilderSolver.getOutput()!,
    fanoutStartEnd: fanoutStartEndSolver.getOutput()!,
  })

  solver.step()
  solver.step()

  const steppedVisualization = solver.visualize()

  expect(solver.solved).toBe(false)
  expect(
    steppedVisualization.lines?.some((line) => line.label === "current-path"),
  ).toBe(true)
  expect(
    steppedVisualization.rects?.some(
      (rect) => rect.label === "current-path-cell",
    ),
  ).toBe(true)
  expect(
    steppedVisualization.circles?.some(
      (circle) => circle.label === "current-candidate",
    ),
  ).toBe(true)

  solver.solve()

  const output = solver.getOutput()
  const visualization = solver.visualize()
  const fanoutStartEnd = fanoutStartEndSolver.getOutput()!
  const pathDeltas = getPathDeltas(output?.path ?? [])
  const directionRuns = getDirectionRuns(pathDeltas)
  const expectedFaninDirection = getCandidateDirectionVector(
    fanoutStartEnd.faninRegionLine.orientation,
    fanoutStartEnd.selectedFaninCandidate.direction,
  )
  const expectedFanoutApproachDirection = invertDirectionVector(
    getCandidateDirectionVector(
      fanoutStartEnd.fanoutRegionLine.orientation,
      fanoutStartEnd.selectedFanoutCandidate.direction,
    ),
  )

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(output?.path.length).toBeGreaterThan(2)
  expect(output?.pathCost).toBeGreaterThan(0)
  expect(output?.greedyMultiplier).toBeCloseTo(1.5)
  expect(output?.obstacleSearchCells).toBe(10)
  expect(output?.obstacleProximityPenalty).toBeCloseTo(12.25)
  expect(output?.ninetyDegreeTurnPenalty).toBeCloseTo(4.9)
  expect(output?.faninCell.index).toBe(
    fanoutStartEndSolver.getOutput()?.selectedFaninCandidate.cell.index,
  )
  expect(output?.fanoutCell.index).toBe(
    fanoutStartEndSolver.getOutput()?.selectedFanoutCandidate.cell.index,
  )
  expect(pathDeltas[0]).toEqual(expectedFaninDirection)
  expect(pathDeltas[1]).toEqual(expectedFaninDirection)
  expect(pathDeltas.at(-1)).toEqual(expectedFanoutApproachDirection)
  expect(pathDeltas.at(-2)).toEqual(expectedFanoutApproachDirection)
  expect(
    pathDeltas.some(
      (delta) =>
        Math.abs(delta.columnStep) === 1 && Math.abs(delta.rowStep) === 1,
    ),
  ).toBe(true)
  expect(directionRuns.every((run) => run.length >= 2)).toBe(true)
  expect(
    visualization.lines?.some((line) => line.label === "current-path"),
  ).toBe(true)
  expect(
    visualization.circles?.some(
      (circle) => circle.label === "current-candidate",
    ),
  ).toBe(true)
})

test("SplitIntoTracePathsSolver expands the centerline bus path into per-trace paths with segmented turns", () => {
  const identifySolver = new IdentifyBusTerminalObstaclesSolver(exampleInput)

  identifySolver.solve()

  const gridBuilderSolver = new GridBuilderSolver({
    inputProblem: exampleInput,
    terminalObstacles: identifySolver.getOutput()!,
  })

  gridBuilderSolver.solve()

  const fanoutStartEndSolver = new FindFanoutStartEndSolver({
    inputProblem: exampleInput,
    grid: gridBuilderSolver.getOutput()!,
  })

  fanoutStartEndSolver.solve()

  const findBusPathSolver = new FindBusPathSolver({
    inputProblem: exampleInput,
    grid: gridBuilderSolver.getOutput()!,
    fanoutStartEnd: fanoutStartEndSolver.getOutput()!,
  })

  findBusPathSolver.solve()

  const solver = new SplitIntoTracePathsSolver({
    inputProblem: exampleInput,
    grid: gridBuilderSolver.getOutput()!,
    fanoutStartEnd: fanoutStartEndSolver.getOutput()!,
    busPath: findBusPathSolver.getOutput()!,
  })

  solver.solve()

  const output = solver.getOutput()
  const visualization = solver.visualize()
  const centerlineRunCount = output?.centerlineRunCount ?? 0
  const traceCount = output?.traceCount ?? 0
  const tracePitch = output?.tracePitch ?? 0
  const minimumTracePathDistance = getMinimumTracePathDistance(
    output?.tracePaths ?? [],
  )
  const outerTraceStartDistance = getPointDistance(
    output?.tracePaths[0]?.points[0] ?? { x: 0, y: 0 },
    output?.tracePaths.at(-1)?.points[0] ?? { x: 0, y: 0 },
  )

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(output?.tracePaths).toHaveLength(gridBuilderSolver.getOutput()?.traceCount ?? 0)
  expect(output?.tracePitch).toBeCloseTo(0.6)
  expect(output?.centerlineRunCount).toBeGreaterThan(1)
  expect(output?.turnCount).toBe(output ? output.centerlineRunCount - 1 : 0)
  expect(
    output?.tracePaths.every((tracePath) => tracePath.points.length > 1),
  ).toBe(true)
  expect(
    output?.tracePaths.some(
      (tracePath) => tracePath.points.length > centerlineRunCount + 1,
    ),
  ).toBe(true)
  expect(outerTraceStartDistance).toBeCloseTo(
    (traceCount - 1) * tracePitch,
    5,
  )
  expect(minimumTracePathDistance).toBeGreaterThanOrEqual(
    (output?.tracePitch ?? 0) - 0.01,
  )
  expect(
    visualization.lines?.some((line) => line.label === "bus-centerline-path"),
  ).toBe(true)
  expect(
    visualization.lines?.filter((line) => line.label === "trace-path"),
  ).toHaveLength(output?.tracePaths.length ?? 0)
})

test("BusRoutePipeline runs through bus path finding and visualizes the current/final path", () => {
  const solver = new BusRoutePipeline(exampleInput)

  solver.solve()

  const output = solver.getOutput()
  const gridOutput = solver.getStageOutput("gridBuilderSolver")
  const fanoutOutput = solver.getStageOutput("findFanoutStartEndSolver")
  const visualization = solver.visualize()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(gridOutput).not.toBeNull()
  expect(fanoutOutput).not.toBeNull()
  expect(output).not.toBeNull()
  expect(gridOutput?.cellSize).toBeCloseTo(2.45)
  expect(output?.path.length).toBeGreaterThan(2)
  expect(output?.pathCost).toBeGreaterThan(0)
  expect(output?.tracePaths).toHaveLength(9)
  expect(
    visualization.lines?.some(
      (line) => line.label === "selected-fanout-fanin-pair",
    ),
  ).toBe(true)
  expect(
    visualization.lines?.some((line) => line.label === "trace-path"),
  ).toBe(true)
  expect(
    visualization.lines?.some((line) => line.label === "bus-centerline-path"),
  ).toBe(true)
})
