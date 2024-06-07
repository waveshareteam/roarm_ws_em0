import WidgetBox from "@/components/WidgetBox";
import { GridItemPropsType } from "@/components/Grid/type";

import ImageView from "./image";

const DEFAULT_CONFIG = {
  sdpSemantics: "unified-plan",
};

export type ImageWidgetProps = {
  topic?: string;
  rtc_config?: Record<string, unknown>;
};

export default function ImageWidget(props: ImageWidgetProps) {
  const { rtc_config, topic } = props;

  const offerUrl = `/plugin/web_rtc/offer`;

  const config: Record<string, unknown> =
    rtc_config === undefined
      ? DEFAULT_CONFIG
      : { ...DEFAULT_CONFIG, ...rtc_config };

  return (
    <WidgetBox
      sx={{
        overflow: "hidden",
        borderRadius: 1,
        display: "flex",
        alignItems: "center",
      }}
      elevation={0}
    >
      <ImageView topic={topic} offerUrl={offerUrl} config={config} />
    </WidgetBox>
  );
}

export const ImageGridProps: GridItemPropsType = {
  minW: 1,
  minH: 2,
  maxW: 3,
  maxH: 6,
};
