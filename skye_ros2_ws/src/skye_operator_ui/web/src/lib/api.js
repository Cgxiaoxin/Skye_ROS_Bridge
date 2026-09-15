async function parseError(response) {
  const text = await response.text();
  try {
    const body = JSON.parse(text);
    return body.reason || text;
  } catch {
    return text || response.statusText;
  }
}

export async function postCommand(op) {
  const r = await fetch('/api/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ op }),
  });
  if (!r.ok) throw new Error(await parseError(r));
  return r.json();
}

export async function startSession(profile, mode) {
  const r = await fetch('/api/session/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ profile, mode }),
  });
  if (!r.ok) throw new Error(await parseError(r));
  return r.json();
}

export async function stopSession() {
  const r = await fetch('/api/session/stop', { method: 'POST' });
  if (!r.ok) throw new Error(await parseError(r));
  return r.json();
}

export async function retryStep() {
  const r = await fetch('/api/session/retry_step', { method: 'POST' });
  if (!r.ok) throw new Error(await parseError(r));
  return r.json();
}
