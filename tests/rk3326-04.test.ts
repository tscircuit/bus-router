import { expect, test } from "bun:test"
import { rk3326_04 } from "./fixtures/rk3326"
import {
  expectRk3326ReproToRoute,
  getRk3326OutputVisualization,
} from "./fixtures/run-rk3326-repro"

test("rk3326-04 routes the 10-line MIPI DSI bus between opposing fanouts", async () => {
  const solver = expectRk3326ReproToRoute(rk3326_04)

  await expect(
    getRk3326OutputVisualization(rk3326_04, solver),
  ).toMatchGraphicsSvg(import.meta.path)
})
