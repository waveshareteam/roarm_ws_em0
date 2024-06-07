import { useState, useEffect } from "react";
import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Slider from "@mui/material/Slider";
import TextField from "@mui/material/TextField";
import Grid from "@mui/material/Unstable_Grid2";
import Box from "@mui/material/Box";
import { GridItemPropsType } from "@/components/Grid/type";
import { AppEvent } from "@/containers/App/app-state";
import WidgetBox from "@/components/WidgetBox.tsx";

const StyledType = styled(Typography)(() => ({
  overflow: "hidden",
  textOverflow: "ellipsis",
  whiteSpace: "nowrap",
  height: "38px",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
}));

const StyledSlider = styled(Slider)(() => ({
  paddingTop: "19px",
}));

type SliderWidgetProps = {
  label?: string;
  value?: number;
  range?: { min: number; max: number; step: number };
  on_change?: (event: AppEvent) => void;
  on_change_committed?: (event: AppEvent) => void;
  text_field?: boolean;
};

type SliderWidgetState = {
  value: number;
};

export default function SliderWidget(props: SliderWidgetProps) {
  const { text_field, range, value, label, on_change_committed, on_change } =
    props;

  const justifyContent = text_field ? "flex-end" : "center";
  const rangeValue = range || { step: 1, min: 0, max: 100 };
  const labelValue = label || "";
  const newValue = value || rangeValue.min;

  const [state, setState] = useState<SliderWidgetState>(() => ({
    value: newValue,
  }));

  const setValue = (newValue: number) => {
    setState((prevState) => {
      if (prevState.value === newValue) {
        return prevState;
      } else {
        return { ...prevState, value: newValue };
      }
    });
  };

  useEffect(() => {
    setValue(newValue);
  }, [newValue]);

  const onChange = (newValue: number) => {
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

  return (
    <WidgetBox>
      <Box
        sx={{
          position: "absolute",
          top: 0,
          left: 0,
          right: 2,
          bottom: 0,
          display: "flex",
          alignItems: "center",
          justifyContent,
        }}
      >
        <Grid
          container
          spacing={2}
          sx={{
            width: "100%",
          }}
        >
          <Grid xs={text_field ? 3 : 4}>
            <StyledType variant={"overline"}>{labelValue}</StyledType>
          </Grid>
          <Grid xs={text_field ? 6 : 8}>
            <StyledSlider
              valueLabelDisplay={text_field ? "off" : "on"}
              step={rangeValue.step}
              min={rangeValue.min}
              max={rangeValue.max}
              value={state.value}
              onChange={(_event, value) => onChange(value as number)}
              onChangeCommitted={onChangeCommitted}
            />
          </Grid>
          {text_field && (
            <Grid xs={3}>
              <TextField
                variant="outlined"
                size="small"
                inputProps={{
                  sx: {
                    textAlign: "right",
                  },
                }}
                value={state.value}
                onChange={(event) => {
                  const v = Number(event.target.value);
                  if (!isNaN(v)) {
                    if (rangeValue.min <= v && v <= rangeValue.max) onChange(v);
                  }
                }}
                onKeyDown={(event) => {
                  if (event.keyCode === 13) {
                    if (onChangeCommitted) onChangeCommitted();
                  }
                }}
              />
            </Grid>
          )}
        </Grid>
      </Box>
    </WidgetBox>
  );
}

export const SliderWidgetGridProps: GridItemPropsType = {
  minW: 1,
  minH: 1,
  maxH: 1,
};
