import { createTheme } from "@mui/material/styles";
import { red } from "@mui/material/colors";

declare module "@mui/material/styles" {
  interface Theme {
    border: {
      thin: string;
    };

    padding: {
      dense: number;
    };
  }
  interface ThemeOptions {
    border?: {
      thin?: string;
    };
    padding?: {
      dense?: number;
    };
  }
}

export const theme = createTheme({
  palette: {
    primary: {
      main: "#23324F",
    },
    secondary: {
      main: "rgba(35, 50, 79, 0.3)",
    },
    error: {
      main: red.A400,
    },
  },
  border: {
    thin: "thin solid rgba(0, 0, 0, 0.12)",
  },
  padding: {
    dense: 48,
  },
});
