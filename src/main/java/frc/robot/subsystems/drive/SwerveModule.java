package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final PIDController turnFeedback = new PIDController(7.0, 0.0, 0.0);

  public SwerveModule(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  public void runSetpoint(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    double turnVolts = turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians());
    io.setTurnVoltage(turnVolts);

    double driveVolts = optimizedState.speedMetersPerSecond * 3.0; 
    io.setDriveVoltage(driveVolts);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.turnAbsolutePositionRad);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionRad, getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityRadPerSec, getAngle());
  }
}