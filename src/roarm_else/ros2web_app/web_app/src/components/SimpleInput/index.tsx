import { useState, useEffect } from "react";

import TextField from "@mui/material/TextField";
import Grid from "@mui/material/Unstable_Grid2";
import Box from "@mui/material/Box";
import KeyboardReturnIcon from "@mui/icons-material/KeyboardReturn";
import IconButton from "@mui/material/IconButton";

import { GridItemPropsType } from "@/components/Grid/type";
import { AppEvent } from "@/containers/App/app-state";
import WidgetBox from "@/components/WidgetBox";
import { LabelTypo } from "@/components/utils";

type SimpleInputWidgetProps = {
  label?: string;
  value?: string;
  input_type?: string;
  on_change?: (event: AppEvent) => void;
  on_change_committed?: (event: AppEvent) => void;
  on_click?: (event: AppEvent) => void;
  button?: boolean;
};

type SimpleInputWidgetState = {
  value: string;
};
export default function SimpleInputWidget(props: SimpleInputWidgetProps) {
  const {
    value,
    input_type,
    label,
    on_change,
    on_change_committed,
    on_click,
    button,
  } = props;

  const [state, setState] = useState<SimpleInputWidgetState>(() => ({
    value: value || "",
  }));

  const setValue = (newValue: string) => {
    setState((prevState) => {
      if (prevState.value === newValue) {
        return prevState;
      } else {
        return { ...prevState, value: newValue };
      }
    });
  };

  useEffect(() => {
    setValue(value || "");
  }, [value]);

  const labelValue = label || "";
  const inputType = input_type || "text";

  const onChange = (newValue: string) => {
    setValue(newValue);
    if (on_change) {
      const event: AppEvent = {
        type: "change",
        value: newValue,
      };
      on_change(event);
    }
  };
  const onChangeCommitted = () => {
    if (on_change_committed) {
      const event: AppEvent = {
        type: "change_committed",
        value: state.value,
      };
      on_change_committed(event);
    }
  };
  const onClick = () => {
    if (on_click) {
      const event: AppEvent = {
        type: "click",
        value: state.value,
      };
      on_click(event);
    }
  };

  return (
    <WidgetBox elevation={1}>
      <Box
        sx={{
          position: "absolute",
          top: 0,
          left: 0,
          right: 2,
          bottom: 0,
          display: "flex",
          alignItems: "center",
          justifyContent: "flex-end",
        }}
      >
        <Grid
          container
          spacing={1}
          sx={{
            width: "100%",
          }}
        >
          <Grid xs={button ? 5 : 6}>
            <LabelTypo variant={"overline"}>{labelValue}</LabelTypo>
          </Grid>
          <Grid xs={button ? 5 : 6}>
            <TextField
              variant="outlined"
              size="small"
              type={inputType}
              inputProps={{
                sx: {
                  textAlign: inputType === "number" ? "right" : "left",
                },
              }}
              value={state.value}
              onChange={(event) => {
                onChange(event.target.value);
              }}
              onKeyDown={(event) => {
                if (event.keyCode === 13) {
                  onChangeCommitted();
                }
              }}
            />
          </Grid>
          {button && (
            <Grid xs={2}>
              <IconButton color="primary" edge={"end"} onClick={onClick}>
                <KeyboardReturnIcon />
              </IconButton>
            </Grid>
          )}
        </Grid>
      </Box>
    </WidgetBox>
  );
}

export const SimpleInputWidgetGridProps: GridItemPropsType = {
  minW: 1,
  minH: 1,
  maxH: 1,
};
