import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import { BusRouterSolver } from "lib/bus-router-solver"

export default (
  <GenericSolverDebugger createSolver={() => new BusRouterSolver()} />
)
