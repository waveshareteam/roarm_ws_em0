import { ReactNode } from "react";
import { useAppState } from "@/containers/App/app-state";
import { updateProps } from "@/containers/App/update-props";

import SimpleButton, { SimpleButtonGridProps } from "@/components/Button";
import ImageWidget, { ImageGridProps } from "@/components/Image";
import JoystickWidget, { JoystickWidgetGridProps } from "@/components/Joystick";
import SimpleList, { SimpleListGridProps } from "@/components/SimpleList";
import SliderWidget, { SliderWidgetGridProps } from "@/components/Slider";
import SwitchWidget, { SwitchWidgetGridProps } from "@/components/Switch";
import SimpleInputWidget, {
  SimpleInputWidgetGridProps,
} from "@/components/SimpleInput";

import InputGroupWidget, {
  InputGroupWidgetGridProps,
} from "@/components/InputGroup";

export type WidgetProps = {
  name: string;
  props: Record<string, unknown>;
  initState: Record<string, unknown>;
};
export default function Widget(props: WidgetProps): ReactNode | undefined {
  const { initState, props: wProps } = props;

  const [state, emit] = useAppState(initState);
  const newProps = updateProps(wProps, state, emit);

  switch (props.name) {
    case "Button":
      return <SimpleButton {...newProps} />;
    case "Image":
      return <ImageWidget {...newProps} />;
    case "Joystick":
      return <JoystickWidget {...newProps} />;
    case "List":
      return <SimpleList {...newProps} />;
    case "Slider":
      return <SliderWidget {...newProps} />;
    case "Switch":
      return <SwitchWidget {...newProps} />;
    case "Input":
      return <SimpleInputWidget {...newProps} />;
    case "InputGroup":
      return <InputGroupWidget {...newProps} />;
    default:
      return undefined;
  }
}

export function WidgetGridProps(name: string) {
  switch (name) {
    case "Button":
      return SimpleButtonGridProps;
    case "Image":
      return ImageGridProps;
    case "Joystick":
      return JoystickWidgetGridProps;
    case "List":
      return SimpleListGridProps;
    case "Slider":
      return SliderWidgetGridProps;
    case "Switch":
      return SwitchWidgetGridProps;
    case "Input":
      return SimpleInputWidgetGridProps;
    case "InputGroup":
      return InputGroupWidgetGridProps;
    default:
      return {};
  }
}
