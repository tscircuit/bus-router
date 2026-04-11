import { expect, test } from "bun:test"
import { BusRouterSolver } from "lib/bus-router-solver"

test("BusRouterSolver routes every default port", () => {
  const solver = new BusRouterSolver()

  solver.solve()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(solver.stats.routedPorts).toBe(solver.stats.totalPorts)
  expect(solver.visualize().lines?.length).toBeGreaterThan(0)
})
