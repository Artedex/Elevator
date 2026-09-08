package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  public TeleopDriveCommand(
      Drive drive,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drive = drive;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Stick üzerindeki minik titreşimleri elemek için deadband (0.05)
    double xInput = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), 0.05);
    double yInput = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), 0.05);
    double omegaInput = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.05);

    // Hız katsayıları: Çizgisel hız max 0.8 m/s, Dönüş hızı max 0.8 rad/s
    double xVelocity = xInput * 0.8; 
    double yVelocity = yInput * 0.8; 
    double omegaVelocity = omegaInput * 0.8; 

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            yVelocity,
            xVelocity,
            omegaVelocity,
            drive.getRotation()
        )
    );
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }
}