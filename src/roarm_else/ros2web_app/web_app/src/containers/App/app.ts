import { useEffect, useState, useRef } from "react";
import { useQueryClient } from "@tanstack/react-query";

import axios from "axios";
import { AppState } from "./app-state";

type AppInfo = {
  name: string;
  ui: Record<string, unknown>;
  clientId: string;
  state: Record<string, unknown>;
};

type StatusType = {
  type: "loading" | "success" | "error";
  error?: string;
};

type AppResponse = {
  app: AppInfo | undefined;
  status: StatusType;
};

const getNewApp = async (appName: string, query?: string) => {
  const baseUrl = `/${appName}/app/new`;
  const url = query ? baseUrl + `?${query}` : baseUrl;
  const response = await axios.get<AppInfo>(url);
  return response.data;
};

export default function RequestApp(
  appName: string,
  query?: string,
): AppResponse {
  const queryClient = useQueryClient();
  const appState = useRef<AppState>();
  const [response, setResponse] = useState<AppResponse>({
    app: undefined,
    status: { type: "loading" },
  });

  useEffect(() => {
    getNewApp(appName, query)
      .then((app) => {
        if (app.name != appName) throw new Error("Failed to get state");

        appState.current = new AppState(app.name, app.clientId, queryClient);
        setResponse({
          app,
          status: { type: "success" },
        });
      })
      .catch((reason) => {
        setResponse({
          app: undefined,
          status: { type: "error", error: reason.message },
        });
      });
    return () => {
      appState.current?.close();
    };
  }, [appName, query, queryClient]);

  return response;
}
