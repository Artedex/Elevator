package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;

public class ModuleIOSparkMax implements ModuleIO {
  private final SparkMax driveSparkMax;
  private final SparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turnAbsoluteEncoder;

  public ModuleIOSparkMax(int driveSparkId, int turnSparkId, boolean isDriveInverted, boolean isTurnInverted) {
    driveSparkMax = new SparkMax(driveSparkId, MotorType.kBrushless);
    turnSparkMax = new SparkMax(turnSparkId, MotorType.kBrushless);

    driveEncoder = driveSparkMax.getEncoder();
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(isDriveInverted);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.inverted(isTurnInverted);

    driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkMax.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Sürüş Verileri
    inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition());
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity());
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = driveSparkMax.getOutputCurrent();

    // Dönüş Verileri 
    inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsoluteEncoder.getPosition());
    inputs.turnPositionRad = Units.rotationsToRadians(turnAbsoluteEncoder.getPosition());
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnAbsoluteEncoder.getVelocity());
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    driveSparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnSparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}