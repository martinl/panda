#!/usr/bin/env python3
import time
import struct
from panda import Panda
from hexdump import hexdump
from panda.python.isotp import isotp_send, isotp_recv

# 0x7e0 = Toyota
# 0x18DB33F1 for Honda?


if __name__ == "__main__":
  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ELM327)
  panda.can_clear(0)

  can_msgs = [b"\x21\x01", b"\x21\x02", b"\x21\x04", b"\x21\x06", b"\x21\x09", b"\x21\x10", b"\x21\x1f", b"\x21\x83", b"\x21\x84"]

  tx_addr = 0x79b
  rx_addr = 0x7bb

  for msg in can_msgs:
    isotp_send(panda, msg, tx_addr)
    ret = isotp_recv(panda, rx_addr)
    hexdump(ret)

    print("%s: %s" % (bytes.hex(msg), "".join(map(chr, ret[:2]))))

    # send nissan custom msg (tester present?)
    panda.can_send(tx_addr, "3000000000000000", 0)
    panda.send_heartbeat()
