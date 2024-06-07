// import React from "react";

import ReactDOM from "react-dom/client";

import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
// import { ReactQueryDevtools } from "@tanstack/react-query-devtools";

import { ThemeProvider } from "@mui/material/styles";
import { theme } from "@/styles/theme";

import AppRoutes from "./routes";
import "./main.css";

import "@fontsource/roboto/300.css";
import "@fontsource/roboto/400.css";
import "@fontsource/roboto/500.css";
import "@fontsource/roboto/700.css";

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      staleTime: Infinity,
    },
  },
});

ReactDOM.createRoot(document.getElementById("root")!).render(
  <QueryClientProvider client={queryClient}>
    <ThemeProvider theme={theme}>
      <AppRoutes />
    </ThemeProvider>
  </QueryClientProvider>,
);

//
// ReactDOM.createRoot(document.getElementById("root")!).render(
//   <QueryClientProvider client={queryClient}>
//     <ThemeProvider theme={theme}>
//       <AppRoutes />
//     </ThemeProvider>
//     <ReactQueryDevtools initialIsOpen={false} />
//   </QueryClientProvider>,
// );
