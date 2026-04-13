import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import exampleBus from "../tests/assets/CM5IO_bus1.json"
import exampleSrj from "../tests/assets/CM5IO.srj.json"

export default (
  <GenericSolverDebugger
    createSolver={() =>
      new BusRoutePipeline({
        obstacles: exampleSrj.obstacles,
        bus: exampleBus,
      })
    }
  />
)
