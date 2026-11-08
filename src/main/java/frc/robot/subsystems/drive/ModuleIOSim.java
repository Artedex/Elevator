package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_GEARING = 6.75;
  private static final double TURN_GEARING = 150.0 / 7.0;

  private final DCMotorSim driveSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, DRIVE_GEARING),
      DCMotor.getNEO(1)
  );

  private final DCMotorSim turnSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, TURN_GEARING),
      DCMotor.getNEO(1)
  );

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(0.02);
    turnSim.update(0.02);


    inputs.drivePositionRad = driveSim.getAngularPositionRad() * DRIVE_GEARING;
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec() * DRIVE_GEARING;
    inputs.driveAppliedVolts = driveAppliedVolts;

    inputs.turnPositionRad = MathUtil.angleModulus(turnSim.getAngularPositionRad());
    inputs.turnAbsolutePositionRad = inputs.turnPositionRad;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }
}