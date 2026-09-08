package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  private Rotation2d yawPosition = new Rotation2d();
  private double yawVelocityRadPerSec = 0.0;

  @Override
  public void updateInputs(GyroIOInputs inputs) {
yawPosition = yawPosition.plus(Rotation2d.fromRadians(yawVelocityRadPerSec * 0.02));    inputs.connected = true;
    inputs.yawPosition = yawPosition;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
  }

  public void setYawVelocity(double yawVelocityRadPerSec) {
    this.yawVelocityRadPerSec = yawVelocityRadPerSec;
  }

  @Override
  public void resetHeading(Rotation2d newHeading) {
    this.yawPosition = newHeading;
  }
}