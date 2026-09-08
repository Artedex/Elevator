package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;

public class RobotContainer {
  private final Drive m_drive;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    m_drive = new Drive(
        new GyroIO() {},
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim()
    );

    configureBindings();

    // 1. Parametre: X Ekseni (İleri / Geri) -> -getLeftY()
    // 2. Parametre: Y Ekseni (Sağ / Sol)   -> -getLeftX()
    // 3. Parametre: Dönüş (Omega)           -> -getRightX()
    m_drive.setDefaultCommand(
        new TeleopDriveCommand(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        )
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}