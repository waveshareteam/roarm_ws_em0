import { Dispatch } from "react";
import { useQueries, QueryClient, useQueryClient } from "@tanstack/react-query";

export type AppEvent = {
  id?: string;
  type: string;
  value?: unknown;
};

type OpenEvent = {
  operation: "open";
  clientId: string;
};

type EmitEvent = {
  operation: "emit";
  clientId: string;
  event: AppEvent;
};

type InvalidateEvent = {
  operation: "invalidate";
  clientId: string;
};

type UpdateEvent = {
  operation: "update";
  updateState: Record<string, unknown>;
};

type RecieveEvent = InvalidateEvent | UpdateEvent;

export class AppState {
  private websocket: WebSocket | undefined;
  private queryClient: QueryClient;

  private readonly clientId: string;
  private readonly url: string;

  constructor(appName: string, clientId: string, queryClient: QueryClient) {
    this.url = `//${window.location.host}/ws/${appName}?clientId=${clientId}`;
    this.clientId = clientId;
    this.queryClient = queryClient;

    this.queryClient.setQueryData<AppState>(["app-state"], this);
    this.__connect();
  }

  private __connect = () => {
    this.websocket = new WebSocket(
      (window.location.protocol === "https:" ? "wss:" : "ws:") + this.url,
    );
    this.websocket.onopen = this.__onopen;
    this.websocket.onclose = this.__onclose;
    this.websocket.onerror = this.__onerror;
    this.websocket.onmessage = this.__onmessage;
  };

  private __onopen = () => {
    const evt: OpenEvent = {
      operation: "open",
      clientId: this.clientId,
    };
    if (this.websocket) this.websocket.send(JSON.stringify(evt));
  };

  private __onclose = (event: CloseEvent) => {
    console.log("onclose", event);
    // setTimeout(function () {
    //   this.connect();
    // }, 1000);
  };

  private __onerror = (event: Event) => {
    console.log("onerror", event);
  };

  private __onmessage = (event: MessageEvent) => {
    const data: RecieveEvent = JSON.parse(event.data);

    switch (data.operation) {
      case "invalidate":
        // this.queryClient.invalidateQueries(["state"].filter(Boolean));
        break;
      case "update":
        Object.entries(data.updateState).forEach(([key, value]) => {
          this.queryClient.setQueriesData(["state", key], value);
        });
        break;
    }
  };

  close = () => {
    console.log("close");
  };

  emit = (event: AppEvent) => {
    const ev: EmitEvent = {
      operation: "emit",
      clientId: this.clientId,
      event,
    };
    const msg = JSON.stringify(ev);
    if (this.websocket) this.websocket.send(msg);
  };
}

export function useAppState(
  initialState: Record<string, unknown>,
): [Record<string, unknown>, Dispatch<AppEvent>] {
  const state: Record<string, unknown> = {};
  const keys = Object.keys(initialState);

  const results = useQueries({
    queries: keys.map((key: string) => ({
      queryKey: ["state", key],
      initialData: initialState[key],
      enabled: false,
    })),
  });

  keys.forEach((key, index) => {
    const { data } = results[index];
    state[key] = data;
  });

  const queryClient = useQueryClient();
  const appState = queryClient.getQueryData<AppState>(["app-state"]);

  const emit = (event: AppEvent) => {
    if (appState) {
      appState.emit(event);
    } else {
      console.error("AppState is not found.");
    }
  };

  return [state, emit];
}
