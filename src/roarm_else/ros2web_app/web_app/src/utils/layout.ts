import { GridItemPropsType } from "@/components/Grid/type";

export type Size = {
  width?: number;
  height?: number;
};
export function sizeValidation(
  layout: { [key: string]: number },
  gridItemProps: GridItemPropsType,
): Size {
  const minW = gridItemProps.minW || 1;
  const minH = gridItemProps.minH || 1;
  const maxW = gridItemProps.maxW || Infinity;
  const maxH = gridItemProps.maxH || Infinity;

  const size: Size = {};
  const w: number | undefined = layout["w"];
  const h: number | undefined = layout["h"];

  if (w !== undefined) {
    size["width"] = Math.min(Math.max(w, minW), maxW);
  }
  if (h !== undefined) {
    size["height"] = Math.min(Math.max(h, minH), maxH);
  }
  return size;
}
