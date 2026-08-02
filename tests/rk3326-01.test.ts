import { test } from "bun:test"
import { rk3326_01 } from "./fixtures/rk3326"
import { expectRk3326ReproToRoute } from "./fixtures/run-rk3326-repro"

test("rk3326-01 routes the 8-bit eMMC data bus between fanouts", () => {
  expectRk3326ReproToRoute(rk3326_01)
})
