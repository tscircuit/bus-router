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
import { FindBusPathSolver, type FindBusPathOutput } from "./FindBusPathSolver"
import {
  SplitIntoTracePathsSolver,
  type SplitIntoTracePathsOutput,
} from "./SplitIntoTracePathsSolver"

export class BusRoutePipeline extends BasePipelineSolver<BusRouterInput> {
  identifyBusTerminalObstaclesSolver?: IdentifyBusTerminalObstaclesSolver
  gridBuilderSolver?: GridBuilderSolver
  findFanoutStartEndSolver?: FindFanoutStartEndSolver
  findBusPathSolver?: FindBusPathSolver
  splitIntoTracePathsSolver?: SplitIntoTracePathsSolver

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
    definePipelineStep(
      "findBusPathSolver",
      FindBusPathSolver,
      (instance: BusRoutePipeline) => [
        {
          inputProblem: instance.inputProblem,
          grid: instance.getStageOutput<GridBuilderOutput>(
            "gridBuilderSolver",
          )!,
          fanoutStartEnd: instance.getStageOutput<FindFanoutStartEndOutput>(
            "findFanoutStartEndSolver",
          )!,
        },
      ],
    ),
    definePipelineStep(
      "splitIntoTracePathsSolver",
      SplitIntoTracePathsSolver,
      (instance: BusRoutePipeline) => [
        {
          inputProblem: instance.inputProblem,
          grid: instance.getStageOutput<GridBuilderOutput>(
            "gridBuilderSolver",
          )!,
          fanoutStartEnd: instance.getStageOutput<FindFanoutStartEndOutput>(
            "findFanoutStartEndSolver",
          )!,
          busPath: instance.getStageOutput<FindBusPathOutput>(
            "findBusPathSolver",
          )!,
        },
      ],
    ),
  ]

  override initialVisualize(): GraphicsObject | null {
    return createBusTerminalObstacleVisualization({
      obstacles: this.inputProblem.obstacles,
      highlightedObstacleIndices: new Set<number>(),
      summaryLines: [
        `SRJ obstacles: ${this.inputProblem.obstacles.length}`,
        `Bus traces: ${this.inputProblem.bus.connectionPatches.length}`,
      ],
      title: "Initial SRJ",
    })
  }

  override getOutput(): SplitIntoTracePathsOutput | null {
    return this.splitIntoTracePathsSolver?.getOutput() ?? null
  }

  override getConstructorParams() {
    return [this.inputProblem]
  }
}
