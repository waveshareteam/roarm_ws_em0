import { useState, useEffect } from "react";
import List from "@mui/material/List";
import ListItemButton from "@mui/material/ListItemButton";
import ListItemText from "@mui/material/ListItemText";

import { AppEvent } from "@/containers/App/app-state";
import { GridItemPropsType } from "@/components/Grid/type.ts";
import WidgetBox from "@/components/WidgetBox.tsx";

export type SimpleListProps = {
  labels: string[];
  on_select?: (event: AppEvent) => void;
  selected_index?: number;
};

type SimpleListState = {
  selectedIndex: number;
};

export default function SimpleList(props: SimpleListProps) {
  const { labels, on_select, selected_index } = props;

  const [state, setState] = useState<SimpleListState>(() => ({
    selectedIndex: selected_index || 0,
  }));

  const setSelectedId = (selectedIndex: number) => {
    setState((prevState) => {
      if (prevState.selectedIndex === selectedIndex) {
        return prevState;
      } else {
        return { ...prevState, selectedIndex: selectedIndex };
      }
    });
  };

  useEffect(() => {
    setSelectedId(selected_index || 0);
  }, [selected_index]);

  const onClick = (index: number) => {
    setSelectedId(index);
    if (on_select) {
      const event: AppEvent = {
        type: "select",
        value: index,
      };
      on_select(event);
    }
  };
  return (
    <WidgetBox>
      <List
        sx={{
          padding: 0,
          height: "100%",
          width: "100%",
          overflowY: "auto",
        }}
      >
        {labels &&
          labels.map((label, index) => {
            return (
              <ListItemButton
                key={index}
                onClick={() => onClick(index)}
                selected={state.selectedIndex === index}
              >
                <ListItemText primary={label} />
              </ListItemButton>
            );
          })}
      </List>
    </WidgetBox>
  );
}

export const SimpleListGridProps: GridItemPropsType = {
  minW: 1,
  minH: 2,
};
