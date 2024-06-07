import { useState, useEffect } from "react";

import TextField from "@mui/material/TextField";
import Grid from "@mui/material/Unstable_Grid2";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button";

import { GridItemPropsType } from "@/components/Grid/type";
import { AppEvent } from "@/containers/App/app-state";
import WidgetBox from "@/components/WidgetBox";
import { LabelTypo } from "@/components/utils";

type SimpleInputWidgetProps = {
  group_name?: string;
  items?: TextFieldItem[];
  columns?: number;
  on_change?: (event: AppEvent) => void;
  on_change_committed?: (event: AppEvent) => void;
  on_click?: (event: AppEvent) => void;
  button?: boolean;
  button_label?: string;
};

type TextFieldItem = {
  label: string;
  value: string;
  type: string;
};
type SimpleInputWidgetState = {
  items: TextFieldItem[];
};

export default function SimpleInputWidget(props: SimpleInputWidgetProps) {
  const {
    group_name,
    items,
    columns,
    on_change,
    on_change_committed,
    on_click,
    button,
    button_label,
  } = props;

  const groupName = group_name || "";
  const col = Math.abs(columns || 1);
  const xs = 12 / col;

  const [state, setState] = useState<SimpleInputWidgetState>(() => {
    const fields = items || [];
    return {
      items: fields.map((item) => {
        return {
          label: item["label"] || "",
          value: item["value"] || "",
          type: item["type"] || "text",
        };
      }),
    };
  });

  useEffect(() => {
    setState((prevState) => {
      const fields = items || [];
      return {
        ...prevState,
        items: fields.map((item) => {
          return {
            label: item["label"] || "",
            value: item["value"] || "",
            type: item["type"] || "text",
          };
        }),
      };
    });
  }, [items]);
  const onChange = (value: string, index: number) => {
    const items = state.items.map((item, i) => {
      if (i === index) {
        return { ...item, value: value };
      } else {
        return item;
      }
    });

    setState((prevState) => {
      return { ...prevState, items: items };
    });

    if (on_change) {
      const event: AppEvent = {
        type: "change",
        value: items,
      };
      on_change(event);
    }
  };
  const onChangeCommitted = () => {
    if (on_change_committed) {
      const event: AppEvent = {
        type: "change_committed",
        value: state.items,
      };
      on_change_committed(event);
    }
  };

  const onClick = () => {
    if (on_click) {
      const event: AppEvent = {
        type: "click",
        value: state.items,
      };
      on_click(event);
    }
  };

  return (
    <WidgetBox
      elevation={1}
      sx={{
        overflow: "hidden",
      }}
    >
      <Box
        sx={{
          position: "absolute",
          top: 0,
          left: 0,
          right: 0,
          height: "60px",
          display: "flex",
          alignItems: "flex-end",
          justifyContent: "center",
        }}
      >
        <LabelTypo variant={"overline"}>{groupName}</LabelTypo>
      </Box>
      <Box
        sx={{
          position: "absolute",
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          p: 1,
          display: "flex",
          alignItems: "flex-end",
          justifyContent: "center",
        }}
      >
        <Grid container spacing={1}>
          {state.items.map((item, index) => (
            <Grid xs={xs} key={index}>
              <TextField
                label={item.label}
                variant="outlined"
                size="small"
                type={item.type}
                value={item.value}
                onChange={(event) => {
                  onChange(event.target.value, index);
                }}
                onKeyDown={(event) => {
                  if (event.keyCode === 13) {
                    onChangeCommitted();
                  }
                }}
              />
            </Grid>
          ))}
          {button && (
            <Grid
              xs={12}
              sx={{
                display: "flex",
                alignItems: "center",
                justifyContent: "flex-end",
              }}
            >
              <Button
                sx={{ width: "100%" }}
                variant="contained"
                onClick={onClick}
              >
                {button_label || ""}
              </Button>
            </Grid>
          )}
        </Grid>
      </Box>
    </WidgetBox>
  );
}

export const InputGroupWidgetGridProps: GridItemPropsType = {
  minW: 1,
  minH: 1,
};
