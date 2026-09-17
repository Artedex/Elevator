package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  public ModuleIOTalonFX(int driveCanId, int turnCanId) {
    driveMotor = new TalonFX(driveCanId);
    turnMotor = new TalonFX(turnCanId);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Sürüş verileri 
    inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
    inputs.driveVelocityRadPerSec = driveMotor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
    inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();

    // Dönüş verileri
    inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnMotor.getPosition().getValueAsDouble());
    inputs.turnPositionRad = Units.rotationsToRadians(turnMotor.getPosition().getValueAsDouble());
    inputs.turnVelocityRadPerSec = turnMotor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
    inputs.turnAppliedVolts = turnMotor.getMotorVoltage().getValueAsDouble();
    inputs.turnCurrentAmps = turnMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}