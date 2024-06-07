export function extractEventId(value: string): string | null {
  if (typeof value === "string" && value.startsWith("$$")) {
    const m = value.match(/^\$\${(.+)}$/);
    const eventId = m && m.length > 1 ? value : null;
    return eventId;
  } else {
    return null;
  }
}

export function extractBindKey(value: string): string | null {
  if (typeof value === "string" && value.startsWith("$")) {
    const m = value.match(/^\${(.+)}$/);
    const state_key = m && m.length > 1 ? m[1] : null;
    return state_key;
  } else {
    return null;
  }
}

export function extractBindKeys(data: any, keys: Set<string>) {
  if (typeof data === "string") {
    const stateKey = extractBindKey(data);
    if (stateKey !== null) {
      keys.add(stateKey);
    }
  } else if (Array.isArray(data)) {
    data.forEach((d) => {
      extractBindKeys(d, keys);
    });
  } else if (data.constructor == Object) {
    for (const d of Object.values(data)) {
      extractBindKeys(d, keys);
    }
  }
}
