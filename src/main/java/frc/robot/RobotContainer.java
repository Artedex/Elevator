package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOSim;

public class RobotContainer {
  private final Drive m_drive;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    if (RobotBase.isReal()) {
      m_drive = new Drive(
          new GyroIOPigeon2(1), // Pigeon 2 CAN ID
          new ModuleIOTalonFX(2, 3), // FL 
          new ModuleIOTalonFX(4, 5), // FR
          new ModuleIOTalonFX(6, 7), // BL
          new ModuleIOTalonFX(8, 9)  // BR
      );
    } else {
      // SIM
      m_drive = new Drive(
          new GyroIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim()
      );
    }

    configureBindings();
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