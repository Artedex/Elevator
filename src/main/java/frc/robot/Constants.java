// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static Mode getMode() {
    return currentMode;
  }

  public static enum Mode {
    REAL,

    SIM,

    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DriveConstants {
    public static final double kMaxLinearSpeedMetersPerSec = 4.5;
    public static final double kMaxAngularSpeedRadPerSec = Math.PI * 2.0;
  }
}