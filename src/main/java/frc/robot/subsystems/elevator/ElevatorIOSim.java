package frc.robot.subsystems.elevator;

import static frc.robot.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          kGearing,
          kCarriageMassKg,
          kDrumRadiusMeters,
          kMinHeightMeters,
          kMaxHeightMeters,
          true, //simulate gravity
          kMinHeightMeters); // starting height

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02); // standard cycle time

    inputs.connected = true;
    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }
}