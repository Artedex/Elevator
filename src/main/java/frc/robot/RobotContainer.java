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
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;

public class RobotContainer {
  private final Drive m_drive;
  private final Elevator m_elevator;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    if (RobotBase.isReal()) {
      m_drive = new Drive(
          new GyroIOPigeon2(1), // Pigeon 2 CAN ID
          new ModuleIOTalonFX(2, 3), // FL
          new ModuleIOTalonFX(4, 5), // FR
          new ModuleIOTalonFX(6, 7), // BL
          new ModuleIOTalonFX(8, 9)  // BR
      );
      m_elevator = new Elevator(new ElevatorIOTalonFX(ElevatorConstants.kMotorCanId));
    } else {
      // SIM
      m_drive = new Drive(
          new GyroIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim()
      );
      m_elevator = new Elevator(new ElevatorIOSim());
    }

    configureBindings();
    m_drive.setDefaultCommand(
        new TeleopDriveCommand(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(),
            m_elevator::getDriveSpeedScale 
        )
    );
  }


  private void configureBindings() {
    // A = alt, B = orta, Y = üst
    m_driverController.a().onTrue(m_elevator.goTo(ElevatorConstants.kLowMeters));
    m_driverController.b().onTrue(m_elevator.goTo(ElevatorConstants.kMidMeters));
    m_driverController.y().onTrue(m_elevator.goTo(ElevatorConstants.kHighMeters));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}