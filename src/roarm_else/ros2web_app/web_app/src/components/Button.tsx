import Button, { ButtonProps } from "@mui/material/Button";
import { GridItemPropsType } from "@/components/Grid/type";
import { AppEvent } from "@/containers/App/app-state.ts";
import WidgetBox from "@/components/WidgetBox.tsx";

type SimpleButtonProps = {
  label?: string;
  on_click?: (event: AppEvent) => void;
} & ButtonProps;

export default function SimpleButton(props: SimpleButtonProps) {
  const { label, on_click } = props;
  if (typeof label !== "string") return undefined;

  const onClick = () => {
    const event: AppEvent = {
      type: "click",
      value: undefined,
    };
    if (on_click !== undefined) on_click(event);
  };

  return (
    <WidgetBox>
      <Button
        variant="contained"
        onClick={onClick}
        sx={{
          width: "100%",
          height: "100%",
        }}
      >
        {label}
      </Button>
    </WidgetBox>
  );
}

export const SimpleButtonGridProps: GridItemPropsType = {
  minW: 1,
  minH: 1,
};
