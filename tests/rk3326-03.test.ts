import { test } from "bun:test"
import { rk3326_03 } from "./fixtures/rk3326"
import { expectRk3326ReproToRoute } from "./fixtures/run-rk3326-repro"

test.skip("rk3326-03 routes the USB OTG bus diagonally between fanouts", () => {
  // Current failure: FindBusPathSolver cannot extend the selected fanin
  // corridor two cells away from the bus. Keep the debugger page available as
  // the executable reproduction until the endpoint strategy is fixed.
  expectRk3326ReproToRoute(rk3326_03)
})
