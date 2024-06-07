import { useState, ChangeEvent, useEffect } from "react";
import { styled } from "@mui/material/styles";
import Switch from "@mui/material/Switch";
import Typography from "@mui/material/Typography";
import Grid from "@mui/material/Unstable_Grid2";
import Box from "@mui/material/Box";

import WidgetBox from "@/components/WidgetBox";
import { GridItemPropsType } from "@/components/Grid/type";
import { AppEvent } from "@/containers/App/app-state";

const StyledType = styled(Typography)(() => ({
  overflow: "hidden",
  textOverflow: "ellipsis",
  whiteSpace: "nowrap",
  height: "100%",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
}));

const StyledSwitch = styled(Switch)(() => ({}));

type SwitchWidgetProps = {
  label?: string;
  on_change?: (event: AppEvent) => void;
  value?: boolean;
  disabled?: boolean;
};

type SwitchWidgetState = {
  value: boolean;
  disabled: boolean;
};

export default function SwitchWidget(props: SwitchWidgetProps) {
  const { label, value, disabled, on_change } = props;

  const labelValue = label || "Switch";
  const [state, setState] = useState<SwitchWidgetState>(() => {
    return {
      value: !!value,
      disabled: !!disabled,
    };
  });

  const updateState = (newValue: boolean, newDisabled: boolean) => {
    setState((prevState) => {
      const renew =
        prevState.value !== newValue || prevState.disabled !== newDisabled;
      if (renew) {
        return { value: newValue, disabled: newDisabled };
      } else {
        return prevState;
      }
    });
  };

  useEffect(() => {
    updateState(!!value, !!disabled);
  }, [value, disabled]);

  const onChange = (event: ChangeEvent<HTMLInputElement>) => {
    updateState(event.target.checked, true);
    if (on_change !== undefined) {
      const appEvent: AppEvent = {
        type: "change",
        value: event.target.checked,
      };
      on_change(appEvent);
    }
  };

  return (
    <WidgetBox>
      <Box
        sx={{
          position: "absolute",
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          p: 0,
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
        }}
      >
        <Grid container spacing={1}>
          <Grid xs={8}>
            <StyledType variant={"overline"}>{labelValue}</StyledType>
          </Grid>
          <Grid xs={4}>
            <StyledSwitch
              checked={state.value}
              disabled={state.disabled}
              onChange={onChange}
            />
          </Grid>
        </Grid>
      </Box>
    </WidgetBox>
  );
}

export const SwitchWidgetGridProps: GridItemPropsType = {
  minW: 2,
  minH: 1,
  maxH: 1,
};
