import { expect, test } from "bun:test"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
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

test("BusRoutePipeline runs the first bus pipeline stage and visualizes start/end obstacles", () => {
  const solver = new BusRoutePipeline(exampleInput)

  solver.solve()
  const output = solver.getOutput()
  const visualization = solver.visualize()
  const blueObstacleRects =
    visualization.rects?.filter((rect) => rect.stroke === "#2563eb") ?? []
  const redObstacleRects =
    visualization.rects?.filter((rect) => rect.stroke === "#dc2626") ?? []

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(output).not.toBeNull()
  expect(output?.busStart.obstacleIndices).toHaveLength(9)
  expect(output?.busEnd.obstacleIndices).toHaveLength(9)
  expect(blueObstacleRects).toHaveLength(18)
  expect(redObstacleRects).toHaveLength(
    exampleSrj.obstacles.length - blueObstacleRects.length,
  )
  expect(visualization.texts?.some((text) => text.text === "Bus Start")).toBe(
    true,
  )
  expect(visualization.texts?.some((text) => text.text === "Bus End")).toBe(
    true,
  )
})
