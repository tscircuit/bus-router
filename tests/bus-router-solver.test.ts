import { expect, test } from "bun:test"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import { FindFanoutStartEndSolver } from "lib/FindFanoutStartEndSolver"
import { GridBuilderSolver, GridCellFlags } from "lib/GridBuilderSolver"
import { IdentifyBusTerminalObstaclesSolver } from "lib/IdentifyBusTerminalObstaclesSolver"
import exampleBus from "./assets/CM5IO_bus1.json"
import exampleSrj from "./assets/CM5IO.srj.json"

const exampleInput = {
  obstacles: exampleSrj.obstacles,
  bus: exampleBus,
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
  expect(output?.requiredBusWidth).toBeCloseTo(2.1)
  expect(output?.cellSize).toBeCloseTo(1.05)
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

test("BusRoutePipeline runs through fanout/fanin selection and visualizes the selected pair", () => {
  const solver = new BusRoutePipeline(exampleInput)

  solver.solve()

  const output = solver.getOutput()
  const gridOutput = solver.getStageOutput("gridBuilderSolver")
  const visualization = solver.visualize()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(gridOutput).not.toBeNull()
  expect(output).not.toBeNull()
  expect(gridOutput?.cellSize).toBeCloseTo(1.05)
  expect(output?.fanoutCandidates.length).toBeGreaterThan(0)
  expect(output?.faninCandidates.length).toBeGreaterThan(0)
  expect(
    visualization.lines?.some(
      (line) => line.label === "selected-fanout-fanin-pair",
    ),
  ).toBe(true)
  expect(
    visualization.circles?.some((circle) => circle.label === "selected-fanout"),
  ).toBe(true)
  expect(
    visualization.circles?.some((circle) => circle.label === "selected-fanin"),
  ).toBe(true)
})
