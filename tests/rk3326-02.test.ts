import { test } from "bun:test"
import { rk3326_02 } from "./fixtures/rk3326"
import { expectRk3326ReproToRoute } from "./fixtures/run-rk3326-repro"

test("rk3326-02 routes the eMMC control bus around to the bottom fanout", () => {
  expectRk3326ReproToRoute(rk3326_02)
})
