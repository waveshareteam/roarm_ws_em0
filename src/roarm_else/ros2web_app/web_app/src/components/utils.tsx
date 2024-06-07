import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";

export const LabelTypo = styled(Typography)(() => ({
  overflow: "hidden",
  textOverflow: "ellipsis",
  whiteSpace: "nowrap",
  height: "38px",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
}));
