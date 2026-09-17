package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final PIDController turnFeedback = new PIDController(7.0, 0.0, 0.0);

  private final PIDController driveFeedback = new PIDController(0.5, 0.0, 0.0);
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.1, 2.3);

  private static final double WHEEL_RADIUS_METERS = 0.05;
  private static final double DRIVE_GEAR_RATIO = 6.75;

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
    // Modern ve deprecate olmayan optimize yöntemi
    SwerveModuleState optimizedState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    optimizedState.optimize(getAngle());

    double turnVolts = turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians());
    io.setTurnVoltage(turnVolts);

    double targetMotorRotPerSec = metersToMotorRot(optimizedState.speedMetersPerSecond);

    // Radyan cinsinden gelen anlık hızı rotasyon/saniyeye çeviriyoruz
    double currentMotorRotPerSec = inputs.driveVelocityRadPerSec / (2.0 * Math.PI); 

    double feedforwardVolts = driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
    double pidVolts = driveFeedback.calculate(currentMotorRotPerSec, targetMotorRotPerSec);

    double driveVolts = feedforwardVolts + pidVolts;
    io.setDriveVoltage(driveVolts);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.turnAbsolutePositionRad);
  }

  public SwerveModulePosition getPosition() {
    // Radyan pozisyonunu rotasyona çevirip yardımcı metoda veriyoruz
    double motorRotations = inputs.drivePositionRad / (2.0 * Math.PI);
    double positionMeters = motorRotToMeters(motorRotations);
    return new SwerveModulePosition(positionMeters, getAngle());
  }

  public SwerveModuleState getState() {
    // Radyan hızını rotasyon/saniyeye çevirip yardımcı metoda veriyoruz
    double motorRotationsPerSec = inputs.driveVelocityRadPerSec / (2.0 * Math.PI);
    double velocityMetersPerSec = motorRotToMeters(motorRotationsPerSec);
    return new SwerveModuleState(velocityMetersPerSec, getAngle());
  }

  private double motorRotToMeters(double motorRot) {
    double meters = (motorRot * 2 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO;
    return meters;
  }

  private double metersToMotorRot(double meters) {
    double motorRot = meters * DRIVE_GEAR_RATIO / (2 * Math.PI * WHEEL_RADIUS_METERS);
    return motorRot;
  }
}