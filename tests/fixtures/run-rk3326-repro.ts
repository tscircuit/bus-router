import { expect } from "bun:test"
import type { GraphicsObject } from "graphics-debug"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import type { BusRouterInput } from "lib/IdentifyBusTerminalObstaclesSolver"

export const expectRk3326ReproToRoute = (input: BusRouterInput) => {
  const solver = new BusRoutePipeline(input)

  solver.solve()

  const output = solver.getOutput()

  expect(solver.failed).toBe(false)
  expect(solver.solved).toBe(true)
  expect(output).not.toBeNull()
  expect(output?.tracePaths).toHaveLength(input.bus.connectionPatches.length)
  expect(output?.path.length).toBeGreaterThan(1)

  return solver
}

export const getRk3326OutputVisualization = (
  input: BusRouterInput,
  solver: BusRoutePipeline,
): GraphicsObject => {
  const output = solver.getOutput()

  return {
    title: output
      ? `${input.bus.busId} output`
      : `${input.bus.busId} failure output`,
    coordinateSystem: "cartesian",
    points: [],
    lines:
      output?.tracePaths.map((tracePath) => ({
        points: tracePath.points,
        strokeColor: `hsl(${210 - tracePath.traceIndex * 18} 72% 44%)`,
        strokeWidth: output.traceWidth,
        label: `trace-${tracePath.traceIndex + 1}`,
      })) ?? [],
    rects: input.obstacles.map((obstacle) => ({
      center: obstacle.center,
      width: obstacle.width,
      height: obstacle.height,
      stroke: obstacle.connectedTo ? "#2563eb" : "#475569",
      fill: obstacle.connectedTo
        ? "rgba(37, 99, 235, 0.24)"
        : "rgba(71, 85, 105, 0.18)",
    })),
    circles: [],
    texts: [
      {
        x: -7.4,
        y: 19,
        text: output
          ? `${output.traceCount} routed traces`
          : (solver.error ?? "Bus routing failed"),
        fontSize: output ? 1.2 : 0.7,
        color: output ? "#0f172a" : "#dc2626",
      },
    ],
  }
}
