import { expect, test } from "bun:test"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import { rk3326_03 } from "./fixtures/rk3326"
import { getRk3326OutputVisualization } from "./fixtures/run-rk3326-repro"

test("rk3326-03 captures the USB OTG diagonal fanout failure", async () => {
  const solver = new BusRoutePipeline(rk3326_03)

  solver.solve()

  expect(solver.failed).toBe(true)
  expect(solver.error).toBe(
    "Unable to extend the selected fanin corridor 2 cells away from the bus.",
  )
  await expect(
    getRk3326OutputVisualization(rk3326_03, solver),
  ).toMatchGraphicsSvg(import.meta.path)
})
