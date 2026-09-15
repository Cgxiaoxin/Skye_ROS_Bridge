export function connectStateStream(onSnapshot, onStatus) {
  let ws = null;
  let reconnectTimer = null;
  let closed = false;

  function connect() {
    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    ws = new WebSocket(`${proto}//${window.location.host}/ws/state`);

    ws.onopen = () => onStatus?.('connected');
    ws.onclose = () => {
      onStatus?.(closed ? 'disconnected' : 'reconnecting');
      if (!closed) {
        reconnectTimer = window.setTimeout(connect, 1000);
      }
    };
    ws.onerror = () => ws?.close();
    ws.onmessage = (event) => {
      try {
        onSnapshot(JSON.parse(event.data));
      } catch {
        /* ignore malformed frames */
      }
    };
  }

  connect();

  return {
    close() {
      closed = true;
      if (reconnectTimer !== null) {
        window.clearTimeout(reconnectTimer);
      }
      ws?.close();
    },
  };
}
