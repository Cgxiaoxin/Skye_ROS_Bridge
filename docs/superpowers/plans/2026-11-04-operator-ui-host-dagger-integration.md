# Operator UI Host DAgger Integration Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.

**Goal:** DAgger playbook uses host HITL dual launch instead of Marvin Docker.

**Spec:** [`docs/superpowers/specs/2026-11-04-operator-ui-host-dagger-integration-design.md`](../specs/2026-11-04-operator-ui-host-dagger-integration-design.md)

**Tech Stack:** Python (skye_operator_ui), Svelte steps.js, pytest

## File map

| File | Change |
|------|--------|
| `playbooks.py` | `_dagger_playbook`: `leaders` step = dual_hitl launch |
| `teardown.py` | dagger defaults: no docker_rm / no disable_via_docker |
| `supervisor.py` | dagger precheck `skip_docker_rm=True` |
| `web/src/lib/steps.js` | marvin → leaders |
| tests | update playbook / teardown / supervisor |
| `docs/Operator_UI使用说明.md` + `项目开发梳理.md` | host path |

---

### Task 1: Playbook + teardown defaults (TDD)

- [x] Update failing tests for dagger playbook / teardown / precheck
- [x] Implement playbooks + teardown + supervisor
- [x] Run pytest for touched tests

### Task 2: Frontend steps + docs

- [x] `steps.js` leaders
- [x] Operator_UI + 项目开发梳理
- [x] Rebuild web dist if required by package install (source change; dist may lag — update if repo tracks dist)
