import { test } from "bun:test"
import { rk3326_04 } from "./fixtures/rk3326"
import { expectRk3326ReproToRoute } from "./fixtures/run-rk3326-repro"

test("rk3326-04 routes the 10-line MIPI DSI bus between opposing fanouts", () => {
  expectRk3326ReproToRoute(rk3326_04)
})
