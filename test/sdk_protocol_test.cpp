#include "unitree_motor_ros_driver/model.hpp"
#include <unitreeMotor/unitreeMotor.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

namespace ud = unitree_driver;
TEST(SdkProtocol, GearRatiosAndGainEncodingMatchBundledBinary) {
  for (const auto type : {ud::MotorType::A1, ud::MotorType::B1, ud::MotorType::GO_M8010_6}) {
    const auto sdk_type = type == ud::MotorType::A1 ? ::MotorType::A1 :
      type == ud::MotorType::B1 ? ::MotorType::B1 : ::MotorType::GO_M8010_6;
    EXPECT_NEAR(ud::profile(type).sdk_gear_ratio, queryGearRatio(sdk_type), 1e-5);
    ud::Command output;
    output.position = -1;
    output.velocity = -2;
    output.torque = 3;
    output.kp = 25;
    output.kd = 0.6;
    const auto wire = ud::toWire(type, output);
    MotorCmd cmd;
    std::memset(static_cast<void*>(&cmd), 0, sizeof(cmd));
    cmd.motorType = sdk_type;
    cmd.mode = queryMotorMode(sdk_type, MotorMode::FOC);
    cmd.q = wire.values.position;
    cmd.dq = wire.values.velocity;
    cmd.tau = wire.values.torque;
    cmd.kp = wire.values.kp;
    cmd.kd = wire.values.kd;
    cmd.modify_data(&cmd);
    if (type == ud::MotorType::GO_M8010_6) {
      ControlData_t packet;
      std::memcpy(&packet, cmd.get_motor_send_data(), sizeof(packet));
      EXPECT_EQ(1, packet.mode.status);
      EXPECT_EQ(static_cast<int16_t>(wire.values.velocity / 6.2832 * 256), packet.comd.spd_des);
      EXPECT_EQ(static_cast<int32_t>(wire.values.position / 6.2832 * 32768), packet.comd.pos_des);
      EXPECT_EQ(static_cast<uint16_t>(wire.values.kp / 25.6 * 32768), packet.comd.k_pos);
      EXPECT_EQ(static_cast<uint16_t>(wire.values.kd / 25.6 * 32768), packet.comd.k_spd);
    } else {
      MasterComdDataV3 packet;
      std::memcpy(&packet, cmd.get_motor_send_data(), sizeof(packet));
      EXPECT_EQ(queryMotorMode(sdk_type, MotorMode::FOC), packet.Mdata.mode);
      EXPECT_EQ(static_cast<int16_t>(wire.values.kp * 2048.f), packet.Mdata.K_P);
      EXPECT_EQ(static_cast<int16_t>(wire.values.kd * (type == ud::MotorType::A1 ? 1024.f : 512.f)),
                packet.Mdata.K_W);
    }
  }
}
TEST(SdkProtocol, GoSdkSaturationIsRejectedByValidation) {
  ud::Config config;
  config.serial_port = "fake";
  config.kp_limit = 2000;
  config.kd_limit = 2000;
  config.normalize();
  ud::Command output;
  std::string reason;
  output.kp = 1100;
  EXPECT_FALSE(ud::validateCommand(config, output, reason));
  output.kp = 0;
  output.kd = 1100;
  EXPECT_FALSE(ud::validateCommand(config, output, reason));
  output.kd = 0;
  output.position = 70000;
  EXPECT_FALSE(ud::validateCommand(config, output, reason));
  output.position = 60000;
  EXPECT_TRUE(ud::validateCommand(config, output, reason));
}
