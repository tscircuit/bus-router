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
import { GridBuilderSolver, type GridBuilderOutput } from "./GridBuilderSolver"
import {
  FindFanoutStartEndSolver,
  type FindFanoutStartEndOutput,
} from "./FindFanoutStartEndSolver"

export class BusRoutePipeline extends BasePipelineSolver<BusRouterInput> {
  identifyBusTerminalObstaclesSolver?: IdentifyBusTerminalObstaclesSolver
  gridBuilderSolver?: GridBuilderSolver
  findFanoutStartEndSolver?: FindFanoutStartEndSolver

  override pipelineDef: PipelineStep<any>[] = [
    definePipelineStep(
      "identifyBusTerminalObstaclesSolver",
      IdentifyBusTerminalObstaclesSolver,
      (instance: BusRoutePipeline) => [instance.inputProblem],
    ),
    definePipelineStep(
      "gridBuilderSolver",
      GridBuilderSolver,
      (instance: BusRoutePipeline) => [
        {
          inputProblem: instance.inputProblem,
          terminalObstacles:
            instance.getStageOutput<BusTerminalObstacleDetectionOutput>(
              "identifyBusTerminalObstaclesSolver",
            )!,
        },
      ],
    ),
    definePipelineStep(
      "findFanoutStartEndSolver",
      FindFanoutStartEndSolver,
      (instance: BusRoutePipeline) => [
        {
          inputProblem: instance.inputProblem,
          grid: instance.getStageOutput<GridBuilderOutput>(
            "gridBuilderSolver",
          )!,
        },
      ],
    ),
  ]

  override getOutput(): FindFanoutStartEndOutput | null {
    return this.findFanoutStartEndSolver?.getOutput() ?? null
  }

  override visualize(): GraphicsObject {
    if (this.activeSubSolver) {
      return this.activeSubSolver.visualize()
    }

    if (this.findFanoutStartEndSolver) {
      return this.findFanoutStartEndSolver.visualize()
    }

    if (this.gridBuilderSolver) {
      return this.gridBuilderSolver.visualize()
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
