import { expect, test } from "bun:test"
import { rk3326_01 } from "./fixtures/rk3326"
import {
  expectRk3326ReproToRoute,
  getRk3326OutputVisualization,
} from "./fixtures/run-rk3326-repro"

test("rk3326-01 routes the 8-bit eMMC data bus between fanouts", async () => {
  const solver = expectRk3326ReproToRoute(rk3326_01)

  await expect(
    getRk3326OutputVisualization(rk3326_01, solver),
  ).toMatchGraphicsSvg(import.meta.path)
})
