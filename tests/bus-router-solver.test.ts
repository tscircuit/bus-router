import { expect, test } from "bun:test"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
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

test("GridBuilderSolver computes an Int32 obstacle grid and marks the terminal cells", () => {
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

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(output?.grid).toBeInstanceOf(Int32Array)
  expect(output?.traceCount).toBe(9)
  expect(output?.requiredBusWidth).toBeCloseTo(2.1)
  expect(output?.cellSize).toBeCloseTo(0.525)
  expect(output?.gridWidth).toBeGreaterThan(0)
  expect(output?.gridHeight).toBeGreaterThan(0)
  expect(output?.obstacleCellCount).toBeGreaterThan(0)
  expect(output?.startCell.index).not.toBe(output?.endCell.index)
  expect(
    (output?.grid[output?.startCell.index ?? 0] ?? 0) & GridCellFlags.start,
  ).toBe(GridCellFlags.start)
  expect(
    (output?.grid[output?.endCell.index ?? 0] ?? 0) & GridCellFlags.end,
  ).toBe(GridCellFlags.end)
  expect(
    visualization.rects?.filter((rect) => rect.label === "obstacle-cell"),
  ).toHaveLength(output?.obstacleCellCount ?? 0)
  expect(
    visualization.rects?.some((rect) => rect.label === "bus-start-cell"),
  ).toBe(true)
  expect(
    visualization.rects?.some((rect) => rect.label === "bus-end-cell"),
  ).toBe(true)
})

test("BusRoutePipeline runs the grid builder stage and visualizes grid occupancy", () => {
  const solver = new BusRoutePipeline(exampleInput)

  solver.solve()

  const output = solver.getOutput()
  const terminalOutput = solver.getStageOutput(
    "identifyBusTerminalObstaclesSolver",
  )
  const visualization = solver.visualize()
  const obstacleCellRects =
    visualization.rects?.filter((rect) => rect.label === "obstacle-cell") ?? []

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(terminalOutput).not.toBeNull()
  expect(output?.grid).toBeInstanceOf(Int32Array)
  expect(output?.traceCount).toBe(9)
  expect(output?.requiredBusWidth).toBeCloseTo(2.1)
  expect(output?.cellSize).toBeCloseTo(0.525)
  expect(output?.obstacleCellCount).toBeGreaterThan(0)
  expect(obstacleCellRects).toHaveLength(output?.obstacleCellCount ?? 0)
  expect(
    visualization.rects?.some((rect) => rect.label === "bus-start-cell"),
  ).toBe(true)
  expect(
    visualization.rects?.some((rect) => rect.label === "bus-end-cell"),
  ).toBe(true)
  expect(visualization.texts?.some((text) => text.text === "Bus Start")).toBe(
    true,
  )
  expect(visualization.texts?.some((text) => text.text === "Bus End")).toBe(
    true,
  )
})
