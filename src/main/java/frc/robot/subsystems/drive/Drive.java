package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final SwerveModule[] modules = new SwerveModule[4];
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private Rotation2d simGyroAngle = new Rotation2d();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;

    modules[0] = new SwerveModule(flModuleIO, 0);
    modules[1] = new SwerveModule(frModuleIO, 1);
    modules[2] = new SwerveModule(blModuleIO, 2);
    modules[3] = new SwerveModule(brModuleIO, 3);

    kinematics = new SwerveDriveKinematics(
        new Translation2d(0.3, 0.3),    // FL 
        new Translation2d(0.3, -0.3),   // FR 
        new Translation2d(-0.3, 0.3),   // BL 
        new Translation2d(-0.3, -0.3)   // BR 
    );

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getRotation(),
        getModulePositions(),
        new Pose2d()
    );
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (SwerveModule module : modules) {
      module.periodic();
    }

    if (RobotBase.isSimulation()) {
      ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(getModuleStates());
      simGyroAngle = simGyroAngle.plus(Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * 0.02));
    }

    poseEstimator.update(getRotation(), getModulePositions());

    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleStates", getModuleStates());
  }

  public void runVelocity(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.5);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }

    Logger.recordOutput("Drive/DesiredModuleStates", states);
  }

  public Rotation2d getRotation() {
    if (RobotBase.isSimulation()) {
      return simGyroAngle;
    }
    return gyroInputs.connected ? gyroInputs.yawPosition : new Rotation2d();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }
}