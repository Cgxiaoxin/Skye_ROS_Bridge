#include <gtest/gtest.h>

#include "L1Robot.h"

TEST(GentoSdkAbi, RequiredControlSymbolsLink) {
  EXPECT_NE(FX_L1_System_GetSDKVersion(), 0);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_System_Link), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Fbk_GetRT), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_State_SwitchToPositionMode), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Runtime_SetJointPosCmd), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Runtime_SetSpeedRatio), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Runtime_StopTraj), nullptr);
}
