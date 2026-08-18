from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.registry import BackendId, BackendSpec, get_backend


DESIRED_BACKEND_PARAM = "LongitudinalPlannerMode"
ACTIVE_BACKEND_PARAM = "ActiveLongitudinalBackend"


def latch_active_backend(params) -> BackendSpec:
  """Select one installed provider for the lifetime of an onroad session."""
  active = params.get(ACTIVE_BACKEND_PARAM)
  if active is not None:
    return get_backend(active)

  backend = get_backend(params.get(DESIRED_BACKEND_PARAM, return_default=True))
  params.put(ACTIVE_BACKEND_PARAM, int(backend.id), block=True)
  return backend


def active_backend_id(params) -> BackendId:
  return latch_active_backend(params).id
