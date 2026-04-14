import {
  GenericSolverStatsSummary,
  PipelineStagesTable,
  SolverBreadcrumbInputDownloader,
} from "@tscircuit/solver-utils/react"
import { getSvgFromGraphicsObject, type GraphicsObject } from "graphics-debug"
import { useEffect, useMemo, useReducer, useRef, useState } from "react"
import { BusRoutePipeline } from "lib/BusRoutePipeline"
import exampleBus from "../tests/assets/CM5IO_bus1.json"
import exampleSrj from "../tests/assets/CM5IO.srj.json"

const EMPTY_GRAPHICS: GraphicsObject = {
  points: [],
  lines: [],
  rects: [],
  circles: [],
  texts: [],
}

const createSolver = () =>
  new BusRoutePipeline({
    obstacles: exampleSrj.obstacles,
    bus: exampleBus,
  })

const hasVisibleGraphics = (graphics: GraphicsObject) =>
  (graphics.points?.length ?? 0) > 0 ||
  (graphics.lines?.length ?? 0) > 0 ||
  (graphics.infiniteLines?.length ?? 0) > 0 ||
  (graphics.rects?.length ?? 0) > 0 ||
  (graphics.circles?.length ?? 0) > 0 ||
  (graphics.polygons?.length ?? 0) > 0 ||
  (graphics.arrows?.length ?? 0) > 0 ||
  (graphics.texts?.length ?? 0) > 0

const makeResponsiveSvgMarkup = (svgMarkup: string) =>
  svgMarkup.replace(
    "<svg ",
    '<svg style="display:block;width:100%;height:auto;max-height:70vh;background:white" ',
  )

const downloadFile = (name: string, contents: string, type: string) => {
  const blob = new Blob([contents], { type })
  const url = URL.createObjectURL(blob)
  const link = document.createElement("a")
  link.href = url
  link.download = name
  document.body.appendChild(link)
  link.click()
  document.body.removeChild(link)
  URL.revokeObjectURL(url)
}

function SolverPage() {
  const [solver, setSolver] = useState(() => createSolver())
  const [renderCount, triggerRender] = useReducer((count: number) => count + 1, 0)
  const [isAnimating, setIsAnimating] = useState(false)
  const [animationSpeed, setAnimationSpeed] = useState(25)
  const animationRef = useRef<ReturnType<typeof setInterval> | null>(null)

  useEffect(() => {
    if (typeof document === "undefined") {
      return
    }

    if (
      document.querySelector(
        'script[src="https://cdn.jsdelivr.net/npm/@tailwindcss/browser@4"]',
      )
    ) {
      return
    }

    const script = document.createElement("script")
    script.src = "https://cdn.jsdelivr.net/npm/@tailwindcss/browser@4"
    document.head.appendChild(script)
  }, [])

  useEffect(() => {
    if (!isAnimating) {
      if (animationRef.current) {
        clearInterval(animationRef.current)
        animationRef.current = null
      }
      return
    }

    animationRef.current = setInterval(() => {
      if (solver.solved || solver.failed) {
        setIsAnimating(false)
        return
      }

      solver.step()
      triggerRender()
    }, Math.max(1, animationSpeed))

    return () => {
      if (animationRef.current) {
        clearInterval(animationRef.current)
        animationRef.current = null
      }
    }
  }, [animationSpeed, isAnimating, solver])

  useEffect(() => {
    if ((solver.solved || solver.failed) && isAnimating) {
      setIsAnimating(false)
    }
  }, [isAnimating, solver.failed, solver.solved])

  const visualization = useMemo(() => {
    try {
      return solver.visualize() ?? EMPTY_GRAPHICS
    } catch (error) {
      console.error("Visualization error:", error)
      return EMPTY_GRAPHICS
    }
  }, [renderCount, solver])

  const svgMarkup = useMemo(() => {
    try {
      return makeResponsiveSvgMarkup(getSvgFromGraphicsObject(visualization))
    } catch (error) {
      console.error("SVG render error:", error)
      return ""
    }
  }, [visualization])

  const handleStep = () => {
    if (solver.solved || solver.failed) {
      return
    }

    solver.step()
    triggerRender()
  }

  const handleSolve = () => {
    if (solver.solved || solver.failed) {
      return
    }

    solver.solve()
    triggerRender()
  }

  const handleReset = () => {
    setIsAnimating(false)
    setSolver(createSolver())
    triggerRender()
  }

  const handleDownloadVisualization = () => {
    downloadFile(
      `${solver.getSolverName()}-visualization.json`,
      JSON.stringify(visualization, null, 2),
      "application/json",
    )
  }

  const handleStepUntilPhase = (phaseName: string) => {
    const pipelineSolver = solver as BusRoutePipeline
    const stageIndex = pipelineSolver.pipelineDef.findIndex(
      (stage) => stage.solverName === phaseName,
    )

    if (stageIndex === -1 || solver.solved || solver.failed) {
      return
    }

    while (
      !solver.solved &&
      !solver.failed &&
      pipelineSolver.currentPipelineStageIndex <= stageIndex
    ) {
      solver.step()
    }

    triggerRender()
  }

  const status = solver.failed
    ? "Failed"
    : solver.solved
      ? "Solved"
      : "Running"

  return (
    <div
      style={{
        display: "flex",
        flexDirection: "column",
        gap: 16,
        padding: 16,
        fontFamily:
          'ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif',
        color: "#0f172a",
        background: "#f8fafc",
        minHeight: "100vh",
        boxSizing: "border-box",
      }}
    >
      <div
        style={{
          display: "flex",
          flexWrap: "wrap",
          gap: 12,
          alignItems: "center",
          justifyContent: "space-between",
        }}
      >
        <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
          <h1 style={{ margin: 0, fontSize: 20, fontWeight: 700 }}>
            BusRoutePipeline
          </h1>
          <div style={{ fontSize: 13, color: "#475569" }}>
            Iteration {solver.iterations} · Progress{" "}
            {Math.round((solver.progress ?? 0) * 100)}% · {status}
            {solver.error ? ` · ${solver.error}` : ""}
          </div>
        </div>

        <div style={{ display: "flex", flexWrap: "wrap", gap: 8 }}>
          <button
            type="button"
            onClick={handleStep}
            disabled={solver.solved || solver.failed}
          >
            Step
          </button>
          <button
            type="button"
            onClick={() => setIsAnimating((current) => !current)}
            disabled={solver.solved || solver.failed}
          >
            {isAnimating ? "Stop" : "Animate"}
          </button>
          <button
            type="button"
            onClick={handleSolve}
            disabled={solver.solved || solver.failed}
          >
            Solve
          </button>
          <button type="button" onClick={handleReset}>
            Reset
          </button>
          <button type="button" onClick={handleDownloadVisualization}>
            Download Visualization
          </button>
          <label
            style={{
              display: "flex",
              alignItems: "center",
              gap: 6,
              fontSize: 13,
              color: "#334155",
            }}
          >
            Speed
            <select
              value={animationSpeed}
              onChange={(event) =>
                setAnimationSpeed(Number(event.currentTarget.value))
              }
            >
              <option value={250}>Slow</option>
              <option value={100}>Normal</option>
              <option value={25}>Fast</option>
              <option value={10}>Fast 2x</option>
              <option value={1}>Fast 10x</option>
            </select>
          </label>
        </div>
      </div>

      <SolverBreadcrumbInputDownloader solver={solver} />

      <GenericSolverStatsSummary
        solverName={solver.getSolverName()}
        stats={solver.stats}
      />

      <div
        style={{
          border: "1px solid #cbd5e1",
          borderRadius: 12,
          background: "#ffffff",
          boxShadow: "0 1px 2px rgba(15, 23, 42, 0.08)",
          overflow: "hidden",
        }}
      >
        <div
          style={{
            padding: "10px 14px",
            borderBottom: "1px solid #e2e8f0",
            fontSize: 14,
            fontWeight: 600,
          }}
        >
          {visualization.title ?? "Visualization"}
        </div>

        {!hasVisibleGraphics(visualization) ? (
          <div style={{ padding: 16, color: "#64748b" }}>No Graphics Yet</div>
        ) : (
          <div
            style={{
              padding: 12,
              overflow: "auto",
              background: "#ffffff",
            }}
            dangerouslySetInnerHTML={{ __html: svgMarkup }}
          />
        )}
      </div>

      <div
        style={{
          border: "1px solid #cbd5e1",
          borderRadius: 12,
          background: "#ffffff",
          boxShadow: "0 1px 2px rgba(15, 23, 42, 0.08)",
          overflow: "hidden",
        }}
      >
        <PipelineStagesTable
          solver={solver}
          onStepUntilPhase={handleStepUntilPhase}
          triggerRender={triggerRender}
        />
      </div>
    </div>
  )
}

export default <SolverPage />
