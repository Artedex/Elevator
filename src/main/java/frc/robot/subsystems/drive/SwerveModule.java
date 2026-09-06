package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final ModuleIO io;
  private final ModuleIOInputs inputs = new ModuleIOInputs();
  private final int index;

  // Dönüş (Azimuth) motoru için PID Kontrolör
  private final PIDController turnFeedback = new PIDController(7.0, 0.0, 0.0);

  public SwerveModule(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Dönüş PID'sini sürekli bir çember (-PI ile +PI arası) üzerinde çalışacak şekilde ayarla
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Periodic döngüsünde çağrılır: Verileri okur ve AdvantageKit'e loglar */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Drive/Module" + index + "/TurnAbsolutePositionRad", inputs.turnAbsolutePositionRad);
    Logger.recordOutput("Drive/Module" + index + "/DrivePositionRad", inputs.drivePositionRad);
    Logger.recordOutput("Drive/Module" + index + "/DriveVelocityRadPerSec", inputs.driveVelocityRadPerSec);
  }

  /** Modüle hedef Hız ve Açı set etmek için kullanılır */
  public void runSetpoint(SwerveModuleState state) {
    // 1. Hedef durumu tekerleğin mevcut açısına göre optimize et (en kısa dönüş yolu)
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    // 2. Dönüş motoru için PID hesaplaması yap ve voltaj olarak uygula
    double turnVolts = turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians());
    io.setTurnVoltage(turnVolts);

    // 3. Sürüş motoruna hedef hıza orantılı voltaj uygula
    double driveVolts = optimizedState.speedMetersPerSecond * 2.0; 
    io.setDriveVoltage(driveVolts);
  }

  /** Modülün mevcut açısını döndürür */
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.turnAbsolutePositionRad);
  }

  /** Robotun sahadaki konum takibi (Odometry) için mevcut pozisyonu döndürür */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionRad, getAngle());
  }

  /** Modülün mevcut durumunu (Hız ve Açı) döndürür */
  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityRadPerSec, getAngle());
  }
}