#!/usr/bin/env python3
import time
import argparse
from panda import Panda
from hexdump import hexdump
from panda.python.isotp import isotp_send, isotp_recv

# Nissal Leaf 2018 IC

ecu_type = {
  0x707: "UNKNOWN",
  0x70e: "BRAKE",
  0x73d: "UNKNOWN",
  0x73f: "VSP",
  0x740: "UNKNOWN",
  0x742: "EPS",
  0x743: "UNKNOWN",
  0x744: "UNKNOWN",
  0x745: "BCM",
  0x746: "TCU",
  0x747: "MULTI AV",
  0x75b: "UNKNOWN",
  0x74d: "IPDM E/R",
  0x74e: "UNKNOWN",
  0x752: "UNKNOWN",
  0x755: "PARKING BRAKE",
  0x758: "AVM",
  0x75d: "UNKNOWN",
  0x766: "UNKNOWN",
  0x76a: "UNKNOWN",
  0x784: "MOTOR CONTROL",
  0x792: "CHARGER",
  0x796: "UNKNOWN",
  0x797: "EV/HEV",
  0x79b: "HV BATTERY",
  0x79d: "SHIFT",
  0x7b7: "UNKNOWN",
  0x7d4: "UNKNOWN",
  0x18dad0f1: "UNKNOWN",
}

can_req = {
  'ecu_fw_version'  : b"\x21\x83",
  'ecu_serial'      : b"\x21\x84"
}

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--rxoffset')
  parser.add_argument('--debug', action='store_true')
  parser.add_argument('--addr')
  args = parser.parse_args()

  if args.addr:
    addrs = [int(args.addr, base=16)]
  else:
    addrs = [0x700 + i for i in range(256)]
    addrs += [0x18da0000 + (i << 8) + 0xf1 for i in range(256)]
  results = {}

  if args.rxoffset:
    rx_offsets = [int(args.rxoffset, base=16)]
  else:
    rx_offsets = [0x1, 0x3, 0x8, 0x20, 0x22, 0x37]

  # for ISO-TP
  DEBUG=args.debug

  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ELM327)
  panda.can_clear(0)

  # 21 81 = Get VIN
  isotp_send(panda, b"\x21\x81", 0x797)
  resp = isotp_recv(panda, 0x79a)
  hexdump(resp)
  print("VIN: %s" % "".join(map(chr, resp[:2])))

  with tqdm(addrs) as t:
    for addr in t:
      # skip functional broadcast addrs
      if addr == 0x7df or addr == 0x18db33f1:
        continue
      t.set_description(hex(addr))

      resp = {}
      for rx_offset in rx_offsets:
        for label, req in can_req:
          isotp_send(panda, req, addr)
          # request all responses for iso-tp request
          #panda.can_send(addr, "30000a0000000000", 0)
          resp[label] = isotp_recv(panda, addr + rx_offset)
          if args.debug:
            print(f"{addr} {rx_offset}")
            hexdump(resp[label])

      if resp.keys():
        results[addr] = resp

    if len(results.items()):
      for addr, resp in results.items():
        print(f"\n\n*** Results for address 0x{addr:X} ({ecu_type[addr]})***\n\n")
        for req, dat in resp.items():
          print(f"{req} {dat}")
    else:
      print("no fw versions found!")
