package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  public GyroIOPigeon2(int canId) {
    pigeon = new Pigeon2(canId);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon.isConnected();
    inputs.yawPosition = pigeon.getRotation2d();
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
  }

  @Override
  public void resetHeading(Rotation2d newHeading) {
    pigeon.setYaw(newHeading.getDegrees());
  }
}