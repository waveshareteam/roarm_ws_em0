import { Dispatch } from "react";
import { extractBindKey, extractEventId } from "@/utils/extract";
import { AppEvent } from "./app-state";

export function updateProps(
  prevProps: any,
  state: Record<string, any>,
  emit: Dispatch<AppEvent>,
): any {
  if (prevProps === undefined || prevProps === null) {
    return {};
  } else if (typeof prevProps === "string") {
    const bindKey = extractBindKey(prevProps);
    if (bindKey !== null) {
      const stateValue = state[bindKey];
      if (typeof stateValue === "string") {
        const eventId = extractEventId(stateValue);
        if (eventId !== null) {
          return (event: AppEvent) => {
            event.id = eventId;
            emit(event);
          };
        }
      }
      return stateValue;
    }
  } else if (Array.isArray(prevProps)) {
    return prevProps.map((value) => updateProps(value, state, emit));
  } else if (prevProps.constructor == Object) {
    const props: Record<string, any> = {};
    for (const [prop, propValue] of Object.entries<any>(prevProps)) {
      props[prop] = updateProps(propValue, state, emit);
    }
    return props;
  }
  return prevProps;
}
