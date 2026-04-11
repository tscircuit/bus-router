import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"

type BusPort = {
  x: number
  y: number
  label: string
}

const DEFAULT_PORTS: BusPort[] = [
  { x: -6, y: 5, label: "A0" },
  { x: -2, y: 3, label: "A1" },
  { x: 2, y: -3, label: "B0" },
  { x: 6, y: -5, label: "B1" },
]

export class BusRouterSolver extends BaseSolver {
  private readonly ports: BusPort[]
  private readonly backbone = {
    start: { x: -8, y: 0 },
    end: { x: 8, y: 0 },
  }
  private readonly routedLabels = new Set<string>()

  constructor(params: { ports?: BusPort[] } = {}) {
    super()
    this.ports = params.ports ?? DEFAULT_PORTS
    this.MAX_ITERATIONS = Math.max(1, this.ports.length)
    this.stats = {
      routedPorts: 0,
      totalPorts: this.ports.length,
    }
  }

  override _step() {
    const nextPort = this.ports[this.iterations - 1]

    if (nextPort) {
      this.routedLabels.add(nextPort.label)
      this.stats = {
        routedPorts: this.routedLabels.size,
        totalPorts: this.ports.length,
      }
    }

    if (this.routedLabels.size >= this.ports.length) {
      this.solved = true
    }
  }

  override visualize(): GraphicsObject {
    const graphics: GraphicsObject = {
      lines: [
        {
          points: [this.backbone.start, this.backbone.end],
          strokeColor: "#2563eb",
          strokeWidth: 2,
        },
      ],
      points: [],
      rects: [],
      circles: [],
      texts: [],
    }

    for (const port of this.ports) {
      const isRouted = this.routedLabels.has(port.label)
      const junction = { x: port.x, y: 0 }

      graphics.lines?.push({
        points: [port, junction],
        strokeColor: isRouted ? "#16a34a" : "#94a3b8",
        strokeWidth: 1.5,
      })

      graphics.points?.push({
        x: port.x,
        y: port.y,
        color: isRouted ? "#16a34a" : "#0f172a",
        label: port.label,
      })

      graphics.points?.push({
        x: junction.x,
        y: junction.y,
        color: isRouted ? "#16a34a" : "#2563eb",
      })
    }

    graphics.texts?.push({
      x: -8,
      y: 7,
      text: `Routed ${this.routedLabels.size}/${this.ports.length} ports`,
      fontSize: 12,
      color: "#0f172a",
    })

    if (this.solved) {
      graphics.texts?.push({
        x: -8,
        y: 5.5,
        text: "Backbone assigned",
        fontSize: 12,
        color: "#16a34a",
      })
    }

    return graphics
  }

  override getConstructorParams() {
    return []
  }
}
