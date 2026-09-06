from openpilot.selfdrive.debug.device_network import select_lan_ipv4


def interface(name, address, state="UP"):
  return {"ifname": name, "operstate": state, "addr_info": [{"family": "inet", "scope": "global", "local": address}]}


def test_lan_ip_prefers_wifi_and_excludes_virtual_or_disconnected_interfaces():
  assert select_lan_ipv4([interface("eth0", "192.168.2.10"), interface("wlan0", "192.168.10.179"),
                          interface("docker0", "172.17.0.1"), interface("tun0", "10.0.0.2")]) == "192.168.10.179"
  assert select_lan_ipv4([interface("wlan0", "192.168.10.179", "DOWN"), interface("eth0", "192.168.2.10")]) == "192.168.2.10"


def test_hotspot_and_unavailable_addresses():
  assert select_lan_ipv4([interface("ap0", "192.168.43.1")]) == "192.168.43.1"
  assert select_lan_ipv4([]) is None
  assert select_lan_ipv4([interface("lo", "127.0.0.1"), interface("wlan0", "169.254.1.2"),
                          interface("eth0", "bad address")]) is None
