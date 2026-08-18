"""Selectable longitudinal planner providers."""


def create_longitudinal_planner(*args, **kwargs):
  # Keep registry and UI tooling importable without loading Params or planner
  # build products. plannerd pays the factory import cost only at construction.
  from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_backends.factory import create_longitudinal_planner as create
  return create(*args, **kwargs)


__all__ = ("create_longitudinal_planner",)
