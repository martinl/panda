#!/usr/bin/env python3
import unittest
from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda


MSG_SUBARU_Brake_Status     = 0x13c
MSG_SUBARU_CruiseControl    = 0x240
MSG_SUBARU_Throttle         = 0x40
MSG_SUBARU_Steering_Torque  = 0x119
MSG_SUBARU_Wheel_Speeds     = 0x13a
MSG_SUBARU_ES_LKAS          = 0x122
MSG_SUBARU_ES_Brake         = 0x220
MSG_SUBARU_ES_Distance      = 0x221
MSG_SUBARU_ES_Status        = 0x222
MSG_SUBARU_ES_DashStatus    = 0x321
MSG_SUBARU_ES_LKAS_State    = 0x322
MSG_SUBARU_ES_Infotainment  = 0x323


def lkas_tx_msgs(alt_bus):
  return [[MSG_SUBARU_ES_LKAS,          0], 
          [MSG_SUBARU_ES_Distance,      alt_bus],
          [MSG_SUBARU_ES_DashStatus,    0],
          [MSG_SUBARU_ES_LKAS_State,    0],
          [MSG_SUBARU_ES_Infotainment,  0]]

def long_tx_msgs():
  return [[MSG_SUBARU_ES_Brake,         0],
          [MSG_SUBARU_ES_Status,        0],
          [MSG_SUBARU_CruiseControl,    2],
          [MSG_SUBARU_Brake_Status,     2]]


class TestSubaruSafetyBase(common.PandaSafetyTest, common.DriverTorqueSteeringSafetyTest):
  FLAGS = 0
  STANDSTILL_THRESHOLD = 0 # kph
  RELAY_MALFUNCTION_ADDR = MSG_SUBARU_ES_LKAS
  RELAY_MALFUNCTION_BUS = 0
  FWD_BUS_LOOKUP = {0: 2, 2: 0}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_SUBARU_ES_LKAS, MSG_SUBARU_ES_DashStatus, MSG_SUBARU_ES_LKAS_State, MSG_SUBARU_ES_Infotainment]}

  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE = 2047

  MAX_RT_DELTA = 940
  RT_INTERVAL = 250000

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_BUS = 0

  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE = 2047

  @classmethod
  def setUpClass(cls):
    if cls.__name__.endswith("Base"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest
  
  def setUp(self):
    self.packer = CANPackerPanda("subaru_global_2017_generated")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_SUBARU, self.FLAGS)
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


class TestSubaruGen2SafetyBase(TestSubaruSafetyBase):
  ALT_BUS = 1

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE = 1000


class TestSubaruLongitudinalSafetyBase(TestSubaruSafetyBase, common.LongitudinalGasBrakeSafetyTest):
  MAX_THROTTLE = 3400
  MAX_BRAKE = 400
  
  FWD_BLACKLISTED_ADDRS = {0: [MSG_SUBARU_CruiseControl, MSG_SUBARU_Brake_Status],
                           2: [MSG_SUBARU_ES_LKAS, MSG_SUBARU_ES_Brake, MSG_SUBARU_ES_Distance,
                               MSG_SUBARU_ES_Status, MSG_SUBARU_ES_DashStatus,
                               MSG_SUBARU_ES_LKAS_State, MSG_SUBARU_ES_Infotainment]}

  def _brake_msg(self, brake):
    values = {"Brake_Pressure": brake}
    return self.packer.make_can_msg_panda("ES_Brake", self.ALT_BUS, values)

  def _throttle_msg(self, throttle):
    values = {"Cruise_Throttle": throttle}
    return self.packer.make_can_msg_panda("ES_Distance", self.ALT_BUS, values)

  def _rpm_msg(self, rpm):
    values = {"Cruise_RPM": rpm}
    return self.packer.make_can_msg_panda("ES_Status", self.ALT_BUS, values)

  def test_rpm_safety_check(self):
    self._generic_limit_safety_check(self._rpm_msg, self.MIN_THROTTLE, self.MAX_THROTTLE, 1.2)


class TestSubaruGen1Safety(TestSubaruSafetyBase):
  FLAGS = 0
  TX_MSGS = lkas_tx_msgs(0)


class TestSubaruGen2Safety(TestSubaruGen2SafetyBase):
  FLAGS = Panda.FLAG_SUBARU_GEN2
  TX_MSGS = lkas_tx_msgs(1)


class TestSubaruGen1LongitudinalSafety(TestSubaruLongitudinalSafetyBase):
  FLAGS = Panda.FLAG_SUBARU_LONG
  TX_MSGS = lkas_tx_msgs(0) + long_tx_msgs()


if __name__ == "__main__":
  unittest.main()
