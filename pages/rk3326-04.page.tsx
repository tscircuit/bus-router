import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import { rk3326_04 } from "tests/fixtures/rk3326"

export default (
  <GenericSolverDebugger createSolver={() => new BusRoutePipeline(rk3326_04)} />
)
