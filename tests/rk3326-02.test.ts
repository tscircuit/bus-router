import { expect, test } from "bun:test"
import { rk3326_02 } from "./fixtures/rk3326"
import {
  expectRk3326ReproToRoute,
  getRk3326OutputVisualization,
} from "./fixtures/run-rk3326-repro"

test("rk3326-02 routes the eMMC control bus around to the bottom fanout", async () => {
  const solver = expectRk3326ReproToRoute(rk3326_02)

  await expect(
    getRk3326OutputVisualization(rk3326_02, solver),
  ).toMatchGraphicsSvg(import.meta.path)
})
