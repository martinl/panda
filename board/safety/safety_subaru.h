const SteeringLimits SUBARU_STEERING_LIMITS = {
  .max_steer = 2047,
  .max_rt_delta = 940,
  .max_rt_interval = 250000,
  .max_rate_up = 50,
  .max_rate_down = 70,
  .driver_torque_factor = 50,
  .driver_torque_allowance = 60,
  .type = TorqueDriverLimited,
};

const SteeringLimits SUBARU_GEN2_STEERING_LIMITS = {
  .max_steer = 1000,
  .max_rt_delta = 940,
  .max_rt_interval = 250000,
  .max_rate_up = 40,
  .max_rate_down = 40,
  .driver_torque_factor = 50,
  .driver_torque_allowance = 60,
  .type = TorqueDriverLimited,
};

const CanMsg SUBARU_TX_MSGS[] = {
  {0x122, 0, 8},
  {0x221, 0, 8},
  {0x321, 0, 8},
  {0x322, 0, 8},
  {0x40,  2, 8},
  {0x139, 2, 8}
};
#define SUBARU_TX_MSGS_LEN (sizeof(SUBARU_TX_MSGS) / sizeof(SUBARU_TX_MSGS[0]))

const CanMsg SUBARU_GEN2_TX_MSGS[] = {
  {0x122, 0, 8},
  {0x221, 1, 8},
  {0x321, 0, 8},
  {0x322, 0, 8}
};
#define SUBARU_GEN2_TX_MSGS_LEN (sizeof(SUBARU_GEN2_TX_MSGS) / sizeof(SUBARU_GEN2_TX_MSGS[0]))

const CanMsg SUBARU_CROSSTREK_HYBRID_TX_MSGS[] = {
  {0x122, 0, 8},
  {0x321, 0, 8},
  {0x322, 0, 8}
};
#define SUBARU_CROSSTREK_HYBRID_TX_MSGS_LEN (sizeof(SUBARU_CROSSTREK_HYBRID_TX_MSGS) / sizeof(SUBARU_CROSSTREK_HYBRID_TX_MSGS[0]))

const CanMsg SUBARU_FORESTER_2022_TX_MSGS[] = {
  {0x124, 0, 8},
  {0x221, 0, 8},
  {0x321, 0, 8},
  {0x322, 0, 8},
  {0x40,  2, 8},
  {0x139, 2, 8}
};
#define SUBARU_FORESTER_2022_TX_MSGS_LEN (sizeof(SUBARU_FORESTER_2022_TX_MSGS) / sizeof(SUBARU_FORESTER_2022_TX_MSGS[0]))

AddrCheckStruct subaru_addr_checks[] = {
  {.msg = {{ 0x40, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{0x119, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x13a, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x13c, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x240, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 50000U}, { 0 }, { 0 }}},
};
#define SUBARU_ADDR_CHECK_LEN (sizeof(subaru_addr_checks) / sizeof(subaru_addr_checks[0]))
addr_checks subaru_rx_checks = {subaru_addr_checks, SUBARU_ADDR_CHECK_LEN};

AddrCheckStruct subaru_gen2_addr_checks[] = {
  {.msg = {{ 0x40, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{0x119, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x13a, 1, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x13c, 1, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x240, 1, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 50000U}, { 0 }, { 0 }}},
};
#define SUBARU_GEN2_ADDR_CHECK_LEN (sizeof(subaru_gen2_addr_checks) / sizeof(subaru_gen2_addr_checks[0]))

AddrCheckStruct subaru_crosstrek_hybrid_addr_checks[] = {
  {.msg = {{0x119, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep =  20000U}, { 0 }, { 0 }}},
  {.msg = {{0x139, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep =  20000U}, { 0 }, { 0 }}},
  {.msg = {{0x13a, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep =  20000U}, { 0 }, { 0 }}},
  {.msg = {{0x168, 1, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep =  40000U}, { 0 }, { 0 }}},
  {.msg = {{0x226, 1, 8, .expected_timestep = 40000U}, { 0 }, { 0 }}},
  {.msg = {{0x321, 2, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 100000U}, { 0 }, { 0 }}},
};
#define SUBARU_CROSSTREK_HYBRID_ADDR_CHECK_LEN (sizeof(subaru_crosstrek_hybrid_addr_checks) / sizeof(subaru_crosstrek_hybrid_addr_checks[0]))

AddrCheckStruct subaru_forester_hybrid_addr_checks[] = {
  {.msg = {{ 0x40, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{0x119, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x13a, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x13c, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{0x222, 2, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 50000U}, { 0 }, { 0 }}},
  {.msg = {{0x321, 2, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 100000U}, { 0 }, { 0 }}},
};
#define SUBARU_FORESTER_HYBRID_ADDR_CHECK_LEN (sizeof(subaru_forester_hybrid_addr_checks) / sizeof(subaru_forester_hybrid_addr_checks[0]))

const uint16_t SUBARU_PARAM_GEN2 = 1;
const uint16_t SUBARU_PARAM_CROSSTREK_HYBRID = 2;
const uint16_t SUBARU_PARAM_FORESTER_HYBRID = 4;
const uint16_t SUBARU_PARAM_FORESTER_2022 = 8;
bool subaru_gen2 = false;
bool subaru_crosstrek_hybrid = false;
bool subaru_forester_hybrid = false;
bool subaru_forester_2022 = false;


static uint32_t subaru_get_checksum(CANPacket_t *to_push) {
  return (uint8_t)GET_BYTE(to_push, 0);
}

static uint8_t subaru_get_counter(CANPacket_t *to_push) {
  return (uint8_t)(GET_BYTE(to_push, 1) & 0xFU);
}

static uint32_t subaru_compute_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = (uint8_t)(addr) + (uint8_t)((unsigned int)(addr) >> 8U);
  for (int i = 1; i < len; i++) {
    checksum += (uint8_t)GET_BYTE(to_push, i);
  }
  return checksum;
}

static int subaru_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &subaru_rx_checks,
                                 subaru_get_checksum, subaru_compute_checksum, subaru_get_counter);

  if (valid) {
    const int bus = GET_BUS(to_push);
    const int alt_bus = subaru_gen2 ? 1 : 0;
    const int stock_ecu = subaru_forester_2022 ? 0x124 : 0x122;

    int addr = GET_ADDR(to_push);
    if ((addr == 0x119) && (bus == 0)) {
      int torque_driver_new;
      torque_driver_new = ((GET_BYTES_04(to_push) >> 16) & 0x7FFU);
      torque_driver_new = -1 * to_signed(torque_driver_new, 11);
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((addr == 0x240) && (bus == alt_bus) && !subaru_crosstrek_hybrid && !subaru_forester_hybrid && !subaru_forester_2022) {
      bool cruise_engaged = GET_BIT(to_push, 41U) != 0U;
      pcm_cruise_check(cruise_engaged);
    }

    if ((addr == 0x321) && (bus == 2) && subaru_crosstrek_hybrid) {
      bool cruise_engaged = ((GET_BYTES_48(to_push) >> 4) & 1U);
      pcm_cruise_check(cruise_engaged);
    }

    if ((addr == 0x222) && (bus == 2) && (subaru_forester_hybrid || subaru_forester_2022)) {
      bool cruise_engaged = GET_BIT(to_push, 29U) != 0U;
      pcm_cruise_check(cruise_engaged);
    }

    // update vehicle moving with any non-zero wheel speed
    if ((addr == 0x13a) && (bus == alt_bus)) {
      vehicle_moving = ((GET_BYTES_04(to_push) >> 12) != 0U) || (GET_BYTES_48(to_push) != 0U);
    }

    if ((addr == 0x13c) && (bus == alt_bus) && !subaru_crosstrek_hybrid) {
      brake_pressed = ((GET_BYTE(to_push, 7) >> 6) & 1U);
    }

    if ((addr == 0x40) && (bus == 0) && !subaru_crosstrek_hybrid) {
      gas_pressed = GET_BYTE(to_push, 4) != 0U;
    }

    // exit controls on rising edge of brake press (Brake_Hybrid)
    if ((addr == 0x226) && (bus == 1) && subaru_crosstrek_hybrid) {
      brake_pressed = ((GET_BYTES_48(to_push) >> 5) & 1U);
    }

    // exit controls on rising edge of gas press (Throttle_Hybrid)
    if ((addr == 0x168) && (bus == 1) && subaru_crosstrek_hybrid) {
      gas_pressed = GET_BYTE(to_push, 4) != 0U;
    }
    generic_rx_checks((addr == stock_ecu) && (bus == 0));
  }
  return valid;
}

static int subaru_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {
  UNUSED(longitudinal_allowed);

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (subaru_gen2) {
    tx = msg_allowed(to_send, SUBARU_GEN2_TX_MSGS, SUBARU_GEN2_TX_MSGS_LEN);
  } else if (subaru_crosstrek_hybrid) {
    tx = msg_allowed(to_send, SUBARU_CROSSTREK_HYBRID_TX_MSGS, SUBARU_CROSSTREK_HYBRID_TX_MSGS_LEN);
  } else if (subaru_forester_2022) {
    tx = msg_allowed(to_send, SUBARU_FORESTER_2022_TX_MSGS, SUBARU_FORESTER_2022_TX_MSGS_LEN);
  } else {
    tx = msg_allowed(to_send, SUBARU_TX_MSGS, SUBARU_TX_MSGS_LEN);
  }

  // steer cmd checks
  if ((addr == 0x122) && !subaru_forester_2022) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x1FFFU);
    desired_torque = -1 * to_signed(desired_torque, 13);

    const SteeringLimits limits = subaru_gen2 ? SUBARU_GEN2_STEERING_LIMITS : SUBARU_STEERING_LIMITS;
    if (steer_torque_cmd_checks(desired_torque, -1, limits)) {
      tx = 0;
    }

  }

  if ((addr == 0x124) && subaru_forester_2022) {
    int desired_torque = ((GET_BYTES_48(to_send) >> 8) & 0x3FFFFU);
    desired_torque = -1 * to_signed(desired_torque, 17);

    const SteeringLimits limits = SUBARU_STEERING_LIMITS;
    if (steer_torque_cmd_checks(desired_torque, -1, limits)) {
      tx = 0;
    }

  }

  return tx;
}

static int subaru_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (bus_num == 0) {
    // Global platform
    // 0x40 Throttle
    // 0x139 Brake_Pedal
    bool block_msg = (subaru_gen2 || subaru_crosstrek_hybrid) ? false : (addr == 0x40) || (addr == 0x139);
    if (!block_msg) {
      bus_fwd = 2; // forward to camera
    }
  }
  if (bus_num == 2) {
    // Global platform
    // 0x122 ES_LKAS
    // 0x124 ES_LKAS_2
    // 0x321 ES_DashStatus
    // 0x322 ES_LKAS_State
    int lkas_msg = subaru_forester_2022 ? 0x124 : 0x122;
    bool block_lkas = (addr == lkas_msg) || (addr == 0x321) || (addr == 0x322);
    if (!block_lkas) {
      bus_fwd = 0;  // Main CAN
    }
  }
  // fallback to do not forward
  return bus_fwd;
}

static const addr_checks* subaru_init(uint16_t param) {
  subaru_gen2 = GET_FLAG(param, SUBARU_PARAM_GEN2);
  subaru_crosstrek_hybrid = GET_FLAG(param, SUBARU_PARAM_CROSSTREK_HYBRID);
  subaru_forester_hybrid = GET_FLAG(param, SUBARU_PARAM_FORESTER_HYBRID);
  subaru_forester_2022 = GET_FLAG(param, SUBARU_PARAM_FORESTER_2022);

  if (subaru_gen2) {
    subaru_rx_checks = (addr_checks){subaru_gen2_addr_checks, SUBARU_GEN2_ADDR_CHECK_LEN};
  } else if (subaru_crosstrek_hybrid) {
    subaru_rx_checks = (addr_checks){subaru_crosstrek_hybrid_addr_checks, SUBARU_CROSSTREK_HYBRID_ADDR_CHECK_LEN};
  } else if (subaru_forester_hybrid || subaru_forester_2022) {
    subaru_rx_checks = (addr_checks){subaru_forester_hybrid_addr_checks, SUBARU_FORESTER_HYBRID_ADDR_CHECK_LEN};
  } else {
    subaru_rx_checks = (addr_checks){subaru_addr_checks, SUBARU_ADDR_CHECK_LEN};
  }

  return &subaru_rx_checks;
}

const safety_hooks subaru_hooks = {
  .init = subaru_init,
  .rx = subaru_rx_hook,
  .tx = subaru_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = subaru_fwd_hook,
};
