import { ChangeEvent, useCallback, useMemo, useState } from "react";
import { Layout, Responsive, WidthProvider } from "react-grid-layout";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { Switch, Typography } from "@mui/material";
import Box from "@mui/material/Box";
import { styled } from "@mui/material/styles";
import Stack from "@mui/material/Stack";

import "./resizable.css";
import "./react-grid-layout.css";
import "./grid.scss";
import {
  isBreakpoint,
  GridLayoutProps,
  LayoutsType,
  BREAK_POINTS,
  GridItem,
} from "@/components/Grid/type";
import Widget from "@/components/Widget";

const Div = styled("div")(({ theme }) => ({
  width: "100%",
  paddingTop: `${theme.padding.dense + 1}px`,
  // backgroundColor: "#eee",
}));

const ResponsiveGridLayout = WidthProvider(Responsive);

const defaultGridLayoutProps: GridLayoutProps = {
  className: "layout",
  // items: 50,
  rowHeight: 60,
  preventCollision: true,
  compactType: null,
  currentBreakpoint: "lg",
  // To turn off animation when mounting,
  // set useCSSTransforms to false and measureBeforeMount to true
  // measureBeforeMount: false,
  useCSSTransforms: true,
  breakpoints: { lg: 1200, md: 996, sm: 768, xs: 480, xxs: 0 },
  cols: { lg: 12, md: 10, sm: 6, xs: 4, xxs: 2 },
  margin: [0, 0],
  containerPadding: [8, 8],
  layouts: {
    lg: [],
    md: [],
    sm: [],
    xs: [],
    xxs: [],
  },
};

type GridState = {
  gridLayoutProps: GridLayoutProps;
  static: boolean;
  onLayoutChange?: (layout: GridItem[]) => void;
};

export type GridProps = {
  onLayoutChange?: (layout: GridItem[]) => void;
  title?: string;
  initLayouts?: LayoutsType;
};

export default function Grid(props: GridProps) {
  const [state, setState] = useState<GridState>(() => {
    const gridLayoutProps = {
      ...defaultGridLayoutProps,
      ...(props.initLayouts ? { layouts: props.initLayouts } : {}),
    };

    return {
      gridLayoutProps,
      static: true,
      onLayoutChange: props.onLayoutChange,
    };
  });

  const onBreakpointChange = useCallback((newBreakpoint: string) => {
    if (isBreakpoint(newBreakpoint)) {
      setState((prevState) => {
        const gridLayoutProps = prevState.gridLayoutProps;
        gridLayoutProps.currentBreakpoint = newBreakpoint;
        return { ...prevState, gridLayoutProps };
      });
    }
  }, []);

  const onItemCallback = useCallback((currentLayout: Layout[]) => {
    setState((prevState) => {
      const gridLayoutProps = prevState.gridLayoutProps;
      const prevLayout =
        gridLayoutProps.layouts[gridLayoutProps.currentBreakpoint!] || [];

      const layout: GridItem[] = [];
      for (const item of currentLayout) {
        const index = prevLayout.findIndex((v) => v.i === item.i);
        if (index === -1) continue;
        const newItem = { ...prevLayout[index], ...item };
        layout.push(newItem);
      }
      gridLayoutProps.layouts = {
        ...gridLayoutProps.layouts,
        [gridLayoutProps.currentBreakpoint!]: layout,
      };
      return { ...prevState, gridLayoutProps };
    });
  }, []);

  const onChangeStatic = useCallback((event: ChangeEvent<HTMLInputElement>) => {
    const checked = !event.target.checked;

    setState((prevState) => {
      const gridLayoutProps = prevState.gridLayoutProps;
      const layouts: LayoutsType = {};
      for (const key of BREAK_POINTS) {
        layouts[key] = [];
        const layout = gridLayoutProps.layouts[key];
        if (layout === undefined) continue;
        for (const item of layout) {
          layouts[key]?.push({ ...item, static: checked });
        }
      }
      gridLayoutProps.layouts = layouts;

      return { ...prevState, gridLayoutProps, static: checked };
    });
  }, []);

  const children = useMemo(() => {
    const currentBreakpoint = state.gridLayoutProps.currentBreakpoint;
    const layout = state.gridLayoutProps.layouts[currentBreakpoint!] || [];
    return GenerateDom(layout, state.static);
  }, [
    state.gridLayoutProps.currentBreakpoint,
    state.gridLayoutProps.layouts,
    state.static,
  ]);

  return (
    <Div>
      <AppBar color="inherit" position="fixed" elevation={1}>
        <Toolbar variant="dense">
          <Typography
            variant="overline"
            sx={{ userSelect: "none", flexGrow: 1 }}
          >
            {props.title}
          </Typography>
          <Switch onChange={onChangeStatic} />
        </Toolbar>
      </AppBar>
      <ResponsiveGridLayout
        {...state.gridLayoutProps}
        onBreakpointChange={onBreakpointChange}
        // onLayoutChange={onLayoutChange}
        onDragStart={onItemCallback}
        onDrag={onItemCallback}
        onDragStop={onItemCallback}
        onResizeStart={onItemCallback}
        onResize={onItemCallback}
        onResizeStop={onItemCallback}
      >
        {children}
      </ResponsiveGridLayout>
    </Div>
  );
}

function GenerateDom(layout: GridItem[], staticFlag: boolean) {
  return layout.map((item) => {
    return (
      <Box key={item.i} className={staticFlag ? "static" : ""}>
        <Box
          sx={{
            position: "absolute",
            top: 0,
            bottom: 0,
            left: 0,
            right: 0,
            p: 1,
          }}
        >
          <Box
            sx={{
              position: "relative",
              p: 0,
              width: "100%",
              height: "100%",
              overflow: "visible",
            }}
          >
            <Widget
              name={item.widgetName}
              props={item.props}
              initState={item.initState}
            />
          </Box>
        </Box>
        <Box
          sx={{
            position: "absolute",
            // display: "none",
            display: item.static ? "none" : "flex",
            userSelect: "none",
            bgcolor: "rgba(0, 0, 0, 0.5)",
            width: "100%",
            height: "100%",
            // p: 1,
          }}
        >
          <Stack spacing={0}>
            <Typography variant="caption" sx={{ color: "#ddd" }}>
              {item.widgetName}
            </Typography>
            <Typography
              variant="caption"
              sx={{
                paddingLeft: 1,
                paddingRight: 1,
                // userSelect: "text",
                backgroundColor: "#fff",
                color: "#555",
              }}
            >
              {`x: ${item.x}, y: ${item.y}, w: ${item.w}, h: ${item.h}`}
            </Typography>
          </Stack>
        </Box>
      </Box>
    );
  });
}
