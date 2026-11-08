package frc.robot.subsystems.elevator;

import static frc.robot.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX motor;

  public ElevatorIOTalonFX(int canId) {
    motor = new TalonFX(canId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
    config.MotorOutput.Inverted =
        kMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);

    motor.setPosition(0.0);
  }

  private static double rotationsToMeters(double motorRotations) {
    return motorRotations / kGearing * 2.0 * Math.PI * kDrumRadiusMeters;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connected = motor.isConnected();
    inputs.positionMeters = rotationsToMeters(motor.getPosition().getValueAsDouble());
    inputs.velocityMetersPerSec = rotationsToMeters(motor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}