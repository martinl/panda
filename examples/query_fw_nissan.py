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

  tx_addrs = [0x707, 0x70e, 0x73d, 0x73f, 0x740, 0x742, 0x743, 0x744, 0x745, 0x746, 0x747, 0x74d, 0x752, 0x755, 0x758, 0x75d, 0x784, 0x792, 0x796, 0x797, 0x79b, 0x79d, 0x7b7, 0x7d4]
  rx_addrs = [0x727, 0x70f, 0x73e, 0x761, 0x760, 0x762, 0x763, 0x764, 0x765, 0x783, 0x767, 0x76d, 0x772, 0x775, 0x778, 0x77d, 0x78c, 0x793, 0x7b6, 0x79a, 0x7bb, 0x7bd, 0x7ba, 0x7d5]

  can_msgs = [b"\x21\x01", b"\x21\x02", b"\x21\x04", b"\x21\x06", b"\x21\x09", b"\x21\x10", b"\x21\x1f", b"\x21\x83", b"\x21\x84"]

  for i in range(len(tx_addrs)):
    for msg in can_msgs:
      print(hex(tx_addrs[i]))

      panda.send_heartbeat()
      isotp_send(panda, msg, tx_addrs[i])
      ret = isotp_recv(panda, rx_addrs[i])
      hexdump(ret)

      print("%s: %s" % (bytes.hex(msg), "".join(map(chr, ret[:2]))))

      # send nissan custom msg (tester present?)
      panda.can_send(tx_addr, "3000000000000000", 0)
      #panda.can_send(tx_addr, "30000a0000000000", 0)
