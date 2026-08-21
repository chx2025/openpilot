from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.selfdrive.ui.sunnypilot.onroad.smart_cruise_control import tesla_longitudinal_label


def test_dynamic_stock_owner_uses_acc_label():
  flags = TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE | TeslaFlagsSP.DYNAMIC_STOCK_ACTIVE
  assert tesla_longitudinal_label(flags, is_tesla=True) == ("ACC", False)


def test_ap_hybrid_stock_owner_uses_ap_label_and_lateral_state():
  flags = (TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE | TeslaFlagsSP.AP_HYBRID_ACTIVE |
           TeslaFlagsSP.AP_HYBRID_STOCK_LATERAL_ACTIVE)
  assert tesla_longitudinal_label(flags, is_tesla=True) == ("AP", True)


def test_sp_owned_longitudinal_keeps_scc_v_label():
  assert tesla_longitudinal_label(0, is_tesla=True) == ("SCC-V", False)
  assert tesla_longitudinal_label(TeslaFlagsSP.AP_HYBRID_ACTIVE, is_tesla=True) == ("SCC-V", False)
  assert tesla_longitudinal_label(TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE, is_tesla=False) == ("SCC-V", False)


def test_manual_stock_owner_uses_acc_label():
  flags = TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE | TeslaFlagsSP.MANUAL_STOCK_ACTIVE
  assert tesla_longitudinal_label(flags, is_tesla=True) == ("ACC", False)
