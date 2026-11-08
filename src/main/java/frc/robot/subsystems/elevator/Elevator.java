package frc.robot.subsystems.elevator;

import static frc.robot.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
* Motor voltage = ProfiledPID (feedback) + ElevatorFeedforward (feedforward)
 */
public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV);

  private double lastGoalMeters = kLowMeters;

  public Elevator(ElevatorIO io) {
    this.io = io;
    controller.setTolerance(kToleranceMeters);

    setDefaultCommand(
        startRun(() -> controller.reset(getHeightMeters()), () -> reachGoal(lastGoalMeters)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/GoalMeters", controller.getGoal().position);
    Logger.recordOutput("Elevator/SetpointMeters", controller.getSetpoint().position);
    Logger.recordOutput("Elevator/AtGoal", atGoal());
  }

  public double getHeightMeters() {
    return inputs.positionMeters;
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public double getDriveSpeedScale() {
    double fraction =
        MathUtil.clamp(
            (getHeightMeters() - kMinHeightMeters) / (kMaxHeightMeters - kMinHeightMeters),
            0.0,
            1.0);
    return MathUtil.interpolate(1.0, kMinDriveSpeedScale, fraction);
  }


  public Command goTo(double goalMeters) {
    double safeGoal = MathUtil.clamp(goalMeters, kMinHeightMeters, kMaxHeightMeters);
    return startRun(
        () -> {
          lastGoalMeters = safeGoal;
          controller.reset(getHeightMeters()); 
        },
        () -> reachGoal(safeGoal));
  }

  private void reachGoal(double goalMeters) {
    controller.setGoal(goalMeters);

    double pidOutput = controller.calculate(getHeightMeters());
    double feedforwardOutput = feedforward.calculate(controller.getSetpoint().velocity);

    double volts = MathUtil.clamp(pidOutput + feedforwardOutput, -12.0, 12.0);
    io.setVoltage(volts);
    Logger.recordOutput("Elevator/OutputVolts", volts);
  }
}