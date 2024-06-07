import React from "react";
import Box from "@mui/material/Box";
import Paper from "@mui/material/Paper";

export type WidgetBoxProps = {
  children?: React.ReactNode;
  sx?: Record<string, unknown>;
  elevation?: number;
};

export default function WidgetBox(props: WidgetBoxProps) {
  const { children, sx, elevation } = props;

  const elevationValue = elevation !== undefined ? elevation : 1;

  return (
    <Box
      sx={{
        position: "absolute",
        top: 0,
        bottom: 0,
        left: 0,
        right: 0,
      }}
    >
      <Paper
        elevation={elevationValue}
        sx={{
          position: "relative",
          p: 0,
          width: "100%",
          height: "100%",
          overflow: "visible",
          ...(elevationValue !== 0 ? { border: "1px solid #ddd" } : {}),
          ...sx,
        }}
      >
        {children}
      </Paper>
    </Box>
  );
}
//
// export default function WidgetBox(props: Props) {
//   const { justify_content, align_items, children, sx } = props;
//   const boxProps = {
//     ...(justify_content
//       ? { justifyContent: justify_content }
//       : { justifyContent: "center" }),
//     ...(align_items ? { alignItems: align_items } : { alignItems: "center" }),
//   };
//   return (
//     <Box
//       sx={{ height: "100%", width: "100%", display: "flex", ...sx }}
//       {...boxProps}
//     >
//       {children}
//     </Box>
//   );
// }
