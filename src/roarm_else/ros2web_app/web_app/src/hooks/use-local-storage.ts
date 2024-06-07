import React, { Dispatch, SetStateAction } from "react";

export function useLocalStorage<T>(
  storageKey:string,
  fallbackState?: T
): [T, Dispatch<SetStateAction<T>>] {

  const [state, setState] = React.useState<T>(()=>{
    const storage = localStorage ? localStorage.getItem(storageKey) || '' : '';
    return JSON.parse(storage) ?? fallbackState
    }
  );
  
  React.useEffect(() => {
    if(state !== undefined)
      localStorage.setItem(storageKey, JSON.stringify(state));
  }, [state, storageKey]);
  return [state, setState];
}