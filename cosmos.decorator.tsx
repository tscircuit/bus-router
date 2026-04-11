import type { ReactNode } from "react"

export default function CosmosDecorator({ children }: { children: ReactNode }) {
  return <div style={{ width: "100vw", height: "100vh" }}>{children}</div>
}
