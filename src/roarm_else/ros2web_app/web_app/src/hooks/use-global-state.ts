import { Dispatch, SetStateAction } from "react";
import { useQuery, useQueryClient, QueryKey } from "@tanstack/react-query";

export function useGlobalState<S>(
  queryKey: QueryKey,
  initialState?: S | (() => S)
): [S, Dispatch<SetStateAction<S>>] {
  const state = useQuery<S>(queryKey, {
    enabled: false,
    ...(initialState ? { initialData: initialState } : {}),
  }).data as S;
  const queryClient = useQueryClient();
  const setState = (value: SetStateAction<S>): void => {
    let newState: S;
    if (typeof value === "function") {
      const prevState = queryClient.getQueryData<S>(queryKey);
      newState = (value as any)(prevState);
    } else {
      newState = value;
    }
    queryClient.setQueryData<S>(queryKey, newState);
  };
  return [state, setState];
}
