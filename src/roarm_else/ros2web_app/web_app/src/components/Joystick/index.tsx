import { Joystick } from "react-joystick-component";
import { IJoystickUpdateEvent } from "react-joystick-component/build/lib/Joystick";
import { AppEvent } from "@/containers/App/app-state";
import { GridItemPropsType } from "@/components/Grid/type.ts";
import WidgetBox from "@/components/WidgetBox.tsx";

export type JoystickWidgetProps = {
  base_color?: string;
  stick_color?: string;
  on_change?: (event: AppEvent) => void;
};

export default function JoystickWidget(props: JoystickWidgetProps) {
  // const theme = useTheme();
  const { base_color, stick_color, on_change } = props;

  const baseColor =
    base_color !== undefined ? base_color : "rgba(35, 50, 79, 0.3)";
  const stickColor = stick_color !== undefined ? stick_color : "#23324F";

  const changeHandler = (jev: IJoystickUpdateEvent) => {
    const event: AppEvent = {
      type: jev.type,
      value: {
        direction: jev.direction,
        distance: jev.distance,
        x: jev.x,
        y: jev.y,
      },
    };
    if (on_change !== undefined) on_change(event);
  };

  return (
    <WidgetBox
      sx={{
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
      }}
    >
      <Joystick
        baseColor={baseColor}
        stickColor={stickColor}
        start={changeHandler}
        move={changeHandler}
        stop={changeHandler}
      />
    </WidgetBox>
  );
}

export const JoystickWidgetGridProps: GridItemPropsType = {
  minW: 1,
  minH: 3,
  maxW: 3,
  maxH: 4,
};
