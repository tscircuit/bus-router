import { expect } from "bun:test"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import type { BusRouterInput } from "lib/IdentifyBusTerminalObstaclesSolver"

export const expectRk3326ReproToRoute = (input: BusRouterInput) => {
  const solver = new BusRoutePipeline(input)

  solver.solve()

  const output = solver.getOutput()

  expect(solver.failed).toBe(false)
  expect(solver.solved).toBe(true)
  expect(output).not.toBeNull()
  expect(output?.tracePaths).toHaveLength(input.bus.connectionPatches.length)
  expect(output?.path.length).toBeGreaterThan(1)
}
