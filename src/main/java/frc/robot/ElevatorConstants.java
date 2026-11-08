package frc.robot;

import edu.wpi.first.math.util.Units;


public final class ElevatorConstants {
  private ElevatorConstants() {}

  // ---- Hardware ----
  public static final int kMotorCanId = 10; 
  public static final boolean kMotorInverted = false; 
  public static final double kStatorCurrentLimitAmps = 60.0;

  // ---- Mechanics ----
  public static final double kGearing = 10.0; // motor speed / drum speed
  public static final double kCarriageMassKg = 4.0;
  public static final double kDrumRadiusMeters = Units.inchesToMeters(1.0);
  public static final double kMinHeightMeters = 0.0;
  public static final double kMaxHeightMeters = 1.25;

  //---- Preset target heights ----
  public static final double kLowMeters = 0.0;
  public static final double kMidMeters = 0.6;
  public static final double kHighMeters = 1.2;
  // ---- Feedforward ----
  public static final double kS = 0.0;  // income
  public static final double kG = 0.17; // gravity
  public static final double kV = 7.5;  // v*s/m

  // ---- Feedback ----
  public static final double kP = 10.0;
  public static final double kI = 0.0;
  public static final double kD = 0.5;


  // ---- Motion profile limits ----
  public static final double kMaxVelocity = 1.0;     // m/s
  public static final double kMaxAcceleration = 2.0; // m/s^2

  public static final double kToleranceMeters = 0.02;


  public static final double kMinDriveSpeedScale = 0.4;
}