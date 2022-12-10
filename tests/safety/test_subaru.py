#!/usr/bin/env python3
import unittest
from panda import Panda
from panda.tests.safety import libpandasafety_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda


class TestSubaruSafety(common.PandaSafetyTest, common.DriverTorqueSteeringSafetyTest):
  TX_MSGS = [[0x122, 0], [0x221, 0], [0x321, 0], [0x322, 0], [0x40, 2], [0x139, 2]]
  STANDSTILL_THRESHOLD = 0  # kph
  RELAY_MALFUNCTION_ADDR = 0x122
  RELAY_MALFUNCTION_BUS = 0
  FWD_BLACKLISTED_ADDRS = {0: [0x40, 0x139], 2: [0x122, 0x321, 0x322]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE = 2047

  MAX_RT_DELTA = 940
  RT_INTERVAL = 250000

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_BUS = 0

  def setUp(self):
    self.packer = CANPackerPanda("subaru_global_2017_generated")
    self.safety = libpandasafety_py.libpandasafety
    self.safety.set_safety_hooks(Panda.SAFETY_SUBARU, 0)
    self.safety.init_tests()

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  # TODO: this is unused
  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_panda("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    # subaru safety doesn't use the scaled value, so undo the scaling
    values = {s: speed * 0.057 for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_panda("Wheel_Speeds", self.ALT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_panda("Brake_Status", self.ALT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque}
    return self.packer.make_can_msg_panda("ES_LKAS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_panda("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_panda("CruiseControl", self.ALT_BUS, values)


class TestSubaruGen2Safety(TestSubaruSafety):
  TX_MSGS = [[0x122, 0], [0x221, 1], [0x321, 0], [0x322, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [0x122, 0x321, 0x322]}
  ALT_BUS = 1

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE = 1000

  def setUp(self):
    self.packer = CANPackerPanda("subaru_global_2017_generated")
    self.safety = libpandasafety_py.libpandasafety
    self.safety.set_safety_hooks(Panda.SAFETY_SUBARU, Panda.FLAG_SUBARU_GEN2)
    self.safety.init_tests()

class TestSubaruCrosstrekHybridSafety(TestSubaruSafety):
  TX_MSGS = [[0x122, 0], [0x321, 0], [0x322, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [0x122, 0x321, 0x322]}

  def setUp(self):
    self.packer = CANPackerPanda("subaru_global_2020_hybrid_generated")
    self.safety = libpandasafety_py.libpandasafety
    self.safety.set_safety_hooks(Panda.SAFETY_SUBARU, Panda.FLAG_SUBARU_CROSSTREK_HYBRID)
    self.safety.init_tests()

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_panda("Brake_Hybrid", 1, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_panda("Throttle_Hybrid", 1, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_panda("ES_DashStatus", 2, values)

class TestSubaruForesterHybridSafety(TestSubaruSafety):

  def setUp(self):
    self.packer = CANPackerPanda("subaru_global_2017_generated")
    self.safety = libpandasafety_py.libpandasafety
    self.safety.set_safety_hooks(Panda.SAFETY_SUBARU, Panda.FLAG_SUBARU_FORESTER_HYBRID)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_panda("ES_Status", 2, values)

class TestSubaruForester2022Safety(TestSubaruSafety):
  TX_MSGS = [[0x124, 0], [0x221, 0], [0x321, 0], [0x322, 0], [0x40, 2], [0x139, 2]]
  RELAY_MALFUNCTION_ADDR = 0x124
  FWD_BLACKLISTED_ADDRS = {0: [0x40, 0x139], 2: [0x124, 0x321, 0x322]}

  def setUp(self):
    self.packer = CANPackerPanda("subaru_global_2022_generated")
    self.safety = libpandasafety_py.libpandasafety
    self.safety.set_safety_hooks(Panda.SAFETY_SUBARU, Panda.FLAG_SUBARU_FORESTER_2022)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_panda("ES_Status", 2, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque}
    return self.packer.make_can_msg_panda("ES_LKAS_2", 0, values)

if __name__ == "__main__":
  unittest.main()
