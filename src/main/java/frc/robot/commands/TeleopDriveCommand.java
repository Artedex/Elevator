package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final DoubleSupplier speedScaleSupplier;

  /**
   * @param speedScaleSupplier 
   * @code
   */
  public TeleopDriveCommand(
      Drive drive,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier speedScaleSupplier) {
    this.drive = drive;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.speedScaleSupplier = speedScaleSupplier;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    double xInput = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), 0.08);
    double yInput = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), 0.08);
    double omegaInput = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.08);

    double scale = MathUtil.clamp(speedScaleSupplier.getAsDouble(), 0.0, 1.0);

    double xVelocity = xInput * DriveConstants.kMaxLinearSpeedMetersPerSec * scale;
    double yVelocity = yInput * DriveConstants.kMaxLinearSpeedMetersPerSec * scale;
    double omegaVelocity = omegaInput * DriveConstants.kMaxAngularSpeedRadPerSec * scale;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, omegaVelocity, drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }
}