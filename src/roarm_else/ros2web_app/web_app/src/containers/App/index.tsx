import { useCallback, useEffect, useMemo } from "react";
import { useParams, useSearchParams } from "react-router-dom";
import { extractBindKeys, sizeValidation } from "@/utils";
// import { useLocalStorage } from "@/hooks";
import RequestApp from "./app";

import Box from "@mui/material/Box";
import Typography from "@mui/material/Typography";

import Grid from "@/components/Grid";
import {
  GridItem,
  LayoutsType,
  BREAK_POINTS,
  GridItemPropsType,
} from "@/components/Grid/type";
import { WidgetGridProps } from "@/components/Widget";

export default function App() {
  const params = useParams();
  const [searchParams] = useSearchParams();
  const { appName } = params;

  if (appName === undefined) throw new Error("Node name does not exist.");

  useEffect(() => {
    document.title = appName;
  }, [appName]);

  // TODO: save layout to local storage
  // const [layout, setLayout] = useLocalStorage<Record<string, GridItem>>(
  //   `${appName}-layout`,
  //   {},
  // );

  const query = searchParams.toString();
  const response = RequestApp(appName, query);

  const onLayoutChange = useCallback(() => {
    // const data: { [key: string]: unknown } = {};
    // for (const item of items) {
    //   const elementKey = item.element.props["elementKey"];
    //   data[elementKey] = item;
    // }
    // setLayout(data);
  }, []);

  const memo = useMemo(() => {
    const app = response.app;

    if (app === undefined) return <></>;

    const layouts: LayoutsType = {
      lg: [],
      md: [],
      sm: [],
      xs: [],
      xxs: [],
    };
    const ui = app.ui;
    const grid = ui["grid"];
    if (grid !== undefined && Array.isArray(grid)) {
      let counter = 0;
      for (const item of grid) {
        const widgetName = item["widget"];
        const props = item["props"] || {};
        const appState = app.state;
        const itemKey = `grid-item-${counter}`;

        const bindKeys = new Set<string>();
        if (props) extractBindKeys(props, bindKeys);
        const initState: Record<string, unknown> = {};
        for (const key of bindKeys) {
          initState[key] = appState[key];
        }
        const gridProps: GridItemPropsType = WidgetGridProps(widgetName);
        const layout = item["layout"] || {};

        const size = sizeValidation(layout, gridProps);
        const baseLayout = {
          x: layout["x"] !== undefined ? layout["x"] : counter % 12,
          y: layout["y"] !== undefined ? layout["y"] : Math.floor(counter / 12),
          w: size.width !== undefined ? size.width : 1,
          h: size.height !== undefined ? size.height : 1,
        };

        const itemLayouts = item["layouts"] || {};
        for (const key of BREAK_POINTS) {
          const layout = itemLayouts[key] || {};
          const size = sizeValidation(layout, gridProps);
          const gridItem: GridItem = {
            ...baseLayout,
            ...layout,
            ...size,
            ...gridProps,
            i: itemKey,
            static: true,
            widgetName,
            props,
            initState,
          };
          layouts[key]!.push(gridItem);
        }
        counter++;
      }
    }
    return (
      <Grid
        title={appName}
        onLayoutChange={onLayoutChange}
        initLayouts={layouts}
      />
    );
  }, [appName, onLayoutChange, response.app]);

  return response.status.type === "loading" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Loading...</Typography>
    </Box>
  ) : response.status.type === "error" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Error: {response.status.error}</Typography>
    </Box>
  ) : response.status.type === "success" ? (
    <div>{memo}</div>
  ) : (
    <></>
  );
}
