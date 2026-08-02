import type {
  BusRouterInput,
  XYPoint,
} from "lib/IdentifyBusTerminalObstaclesSolver"

type FanoutConnection = {
  start: XYPoint
  end: XYPoint
}

// These endpoint coordinates come from the global autorouting phase generated
// after the RK3326, eMMC, USB, and DSI local fanout phases have completed.
const traceWidth = 0.09
const traceSpacing = 0.075
const terminalSize = 0.15

const componentKeepouts: BusRouterInput["obstacles"] = [
  {
    type: "rect",
    center: { x: 0, y: 0 },
    width: 14.8,
    height: 14.8,
    layers: ["top", "inner1", "inner2", "inner3", "bottom"],
  },
  {
    type: "rect",
    center: { x: 17, y: -4 },
    width: 8,
    height: 8,
    layers: ["top", "inner1", "inner2", "inner3", "bottom"],
  },
  {
    type: "rect",
    center: { x: 0, y: -20 },
    width: 5,
    height: 5,
    layers: ["top", "inner1", "inner2", "inner3", "bottom"],
  },
  {
    type: "rect",
    center: { x: 0, y: 15.5 },
    width: 14.5,
    height: 4.5,
    layers: ["top", "inner1", "inner2", "inner3", "bottom"],
  },
]

const createRk3326BusInput = (
  busId: string,
  connections: FanoutConnection[],
): BusRouterInput => {
  const connectionPatches = connections.map((connection, index) => {
    const connectionId = `${busId}-${index}`
    const startPointId = `${connectionId}-start`
    const endPointId = `${connectionId}-end`

    return {
      connectionId,
      pointIds: [startPointId, endPointId],
      _bus: { id: busId },
    }
  })
  const pointIds = connectionPatches.flatMap(
    (connectionPatch) => connectionPatch.pointIds,
  )
  const terminalObstacles = connections.flatMap((connection, index) => {
    const connectionPatch = connectionPatches[index]!

    return [
      {
        type: "rect",
        center: connection.start,
        width: terminalSize,
        height: terminalSize,
        connectedTo: [connectionPatch.pointIds[0]!],
      },
      {
        type: "rect",
        center: connection.end,
        width: terminalSize,
        height: terminalSize,
        connectedTo: [connectionPatch.pointIds[1]!],
      },
    ]
  })

  return {
    obstacles: [...componentKeepouts, ...terminalObstacles],
    bus: {
      busId,
      pointIds,
      connectionPatches,
    },
    traceWidth,
    traceSpacing,
  }
}

export const rk3326_01 = createRk3326BusInput("rk3326-emmc-data", [
  { start: { x: 7.65, y: -0.5775 }, end: { x: 15.000004, y: 2 } },
  { start: { x: 7.65, y: 4.225 }, end: { x: 15.500003, y: 2 } },
  { start: { x: 7.65, y: 5.525 }, end: { x: 17.2475, y: 2 } },
  { start: { x: 7.65, y: -0.975 }, end: { x: 14.749941, y: 2 } },
  { start: { x: 7.65, y: 3.575 }, end: { x: 15.250067, y: 2 } },
  { start: { x: 7.65, y: 5.2 }, end: { x: 15.749939, y: 2 } },
  { start: { x: 7.65, y: 5.85 }, end: { x: 17.4125, y: 2 } },
  { start: { x: 7.65, y: 6.175 }, end: { x: 17.5775, y: 2 } },
])

export const rk3326_02 = createRk3326BusInput("rk3326-emmc-control", [
  { start: { x: 7.65, y: -0.165 }, end: { x: 16.749937, y: -10 } },
  { start: { x: 7.65, y: 0 }, end: { x: 16.500001, y: -10 } },
  { start: { x: 7.65, y: 3.25 }, end: { x: 17, y: -10 } },
])

export const rk3326_03 = createRk3326BusInput("rk3326-usb-otg", [
  { start: { x: 7.65, y: -1.625 }, end: { x: 0.2475, y: -16.9249851 } },
  { start: { x: 7.65, y: -5.85 }, end: { x: -0.0825, y: -16.9249851 } },
  { start: { x: 7.65, y: -1.3 }, end: { x: 0.0825, y: -16.9249851 } },
  { start: { x: 7.65, y: -1.95 }, end: { x: -0.2475, y: -16.9249851 } },
])

export const rk3326_04 = createRk3326BusInput("rk3326-mipi-dsi", [
  { start: { x: 3.9, y: 7.65 }, end: { x: -3.749929, y: 12.45008665 } },
  { start: { x: 4.225, y: 7.65 }, end: { x: -3.250057, y: 12.45008665 } },
  { start: { x: 2.925, y: 7.65 }, end: { x: -2.250059, y: 12.45008665 } },
  { start: { x: 3.25, y: 7.65 }, end: { x: -1.749933, y: 12.45008665 } },
  { start: { x: 1.95, y: 7.65 }, end: { x: -0.749935, y: 12.45008665 } },
  { start: { x: 1.3, y: 7.65 }, end: { x: 0.0825, y: 12.45008665 } },
  { start: { x: -0.7425, y: 7.65 }, end: { x: 0.2475, y: 12.45008665 } },
  { start: { x: -0.5775, y: 7.65 }, end: { x: 0.4125, y: 12.45008665 } },
  { start: { x: 2.6, y: 7.65 }, end: { x: 1.249807, y: 12.45008665 } },
  { start: { x: 2.275, y: 7.65 }, end: { x: 1.749933, y: 12.45008665 } },
])
