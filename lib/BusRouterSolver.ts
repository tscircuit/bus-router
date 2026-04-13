import {
  BasePipelineSolver,
  definePipelineStep,
  type PipelineStep,
} from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import {
  createBusTerminalObstacleVisualization,
  IdentifyBusTerminalObstaclesSolver,
  type BusRouterInput,
  type BusTerminalObstacleDetectionOutput,
} from "./IdentifyBusTerminalObstaclesSolver"

export class BusRouterSolver extends BasePipelineSolver<BusRouterInput> {
  identifyBusTerminalObstaclesSolver?: IdentifyBusTerminalObstaclesSolver

  override pipelineDef: PipelineStep<any>[] = [
    definePipelineStep(
      "identifyBusTerminalObstaclesSolver",
      IdentifyBusTerminalObstaclesSolver,
      (instance: BusRouterSolver) => [instance.inputProblem],
    ),
  ]

  override getOutput(): BusTerminalObstacleDetectionOutput | null {
    return this.identifyBusTerminalObstaclesSolver?.getOutput() ?? null
  }

  override visualize(): GraphicsObject {
    if (this.activeSubSolver) {
      return this.activeSubSolver.visualize()
    }

    if (this.identifyBusTerminalObstaclesSolver) {
      return this.identifyBusTerminalObstaclesSolver.visualize()
    }

    return createBusTerminalObstacleVisualization({
      obstacles: this.inputProblem.obstacles,
      highlightedObstacleIndices: new Set<number>(),
      summaryLines: [
        "Stage 1: Identify the obstacle groups for bus start and bus end.",
      ],
      title: "Bus Router - Initial Obstacles",
    })
  }

  override getConstructorParams() {
    return [this.inputProblem]
  }
}
