export function camelToSnake(str: string): string {
  if(typeof str !== 'string') return str;

  const tmp = str.replace(/^([A-Z]+)/, (s) => s.toLowerCase())
  const tmp2 = tmp.replace(/([A-Z])/g, (s) => "_" + s.toLowerCase());
  return tmp2.replace(/__/g, "_");
}

export function snakeToCamel(str: string): string {
  if(typeof str !== 'string') return str;

  return str.replace(/_./g, (s) => s.charAt(1).toUpperCase());
}
