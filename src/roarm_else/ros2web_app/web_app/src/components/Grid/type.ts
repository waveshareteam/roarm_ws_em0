import { Layout } from "react-grid-layout";

export const BREAK_POINTS = ["lg", "md", "sm", "xs", "xxs"] as const;
export type BreakpointType = (typeof BREAK_POINTS)[number];
export function isBreakpoint(breakpoint: string): breakpoint is BreakpointType {
  return BREAK_POINTS.includes(breakpoint as BreakpointType);
}

export type LayoutsType = {
  [key in BreakpointType]?: GridItem[];
};

export type GridLayoutProps = {
  className: string;
  items?: number;
  rowHeight?: number;
  preventCollision?: boolean;
  compactType?: "vertical" | "horizontal" | null;
  currentBreakpoint?: BreakpointType;
  measureBeforeMount?: boolean;
  useCSSTransforms?: boolean;
  breakpoints: { lg: number; md: number; sm: number; xs: number; xxs: number };
  cols: { lg: number; md: number; sm: number; xs: number; xxs: number };
  margin?: [number, number];
  containerPadding?: [number, number];
  layouts: LayoutsType;
};

export type GridItem = {
  widgetName: string;
  props: Record<string, unknown>;
  initState: Record<string, unknown>;
} & Layout;

export type GridItemPropsType = {
  minW?: number;
  maxW?: number;
  minH?: number;
  maxH?: number;
  isResizable?: boolean;
  resizeHandles?: Array<"s" | "w" | "e" | "n" | "sw" | "nw" | "se" | "ne">;
};
