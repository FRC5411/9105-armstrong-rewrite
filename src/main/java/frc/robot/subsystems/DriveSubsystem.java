
package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars.DriverProfiles;
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.ArcadeCommand;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import frc.robot.commands.TurnCommand;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private RelativeEncoder leftFrontEncoder;
  private RelativeEncoder leftBackEncoder;
  private RelativeEncoder rightFrontEncoder;
  private RelativeEncoder rightBackEncoder;

  private DifferentialDrive robotDrive;

  private AHRS navX;

  private DifferentialDriveOdometry odometry;

  private Field2d field;

  private Pose2d targetPose;

  public DriveSubsystem() {
    leftFrontMotor = new CANSparkMax(
      DrivebaseConstants.LF_MOTOR_CANID, 
      MotorType.kBrushless);

    leftBackMotor = new CANSparkMax(
      DrivebaseConstants.LB_MOTOR_CANID, 
      MotorType.kBrushless);

    rightFrontMotor = new CANSparkMax(
      DrivebaseConstants.RF_MOTOR_CANID, 
      MotorType.kBrushless);

    rightBackMotor = new CANSparkMax(
      DrivebaseConstants.RB_MOTOR_CANID, 
      MotorType.kBrushless);

    // rightBackMotor.setInverted(true);
    // rightFrontMotor.setInverted(true);

    // leftBackMotor.follow(leftFrontMotor);
    // rightBackMotor.follow(rightFrontMotor);

    leftFrontEncoder = leftFrontMotor.getEncoder();
    leftBackEncoder = leftBackMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    rightBackEncoder = rightBackMotor.getEncoder();

    leftFrontEncoder.setPositionConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    leftBackEncoder.setPositionConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    rightFrontEncoder.setPositionConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    rightBackEncoder.setPositionConversionFactor( 
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    leftFrontEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60);

    leftBackEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60);

    rightFrontEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60);

    rightBackEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60);

    leftMotors = new MotorControllerGroup(leftBackMotor, leftFrontMotor);
    rightMotors = new MotorControllerGroup(rightBackMotor, rightFrontMotor);

    robotDrive = new DifferentialDrive(rightMotors, leftMotors);

    navX = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

    field = new Field2d();

    SmartDashboard.putData("Field", field);

    navX.calibrate();
    navX.reset();

    resetEncoders();
    resetOdometry(getPose());

    targetPose = new Pose2d();
  }

  public Pose2d returnRobotPose(){
    return getPose();
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public void setTargetPose(Pose2d pose) {
    targetPose = pose;
  }

  public void arcadeDrive(double speed, double rotation) {
    if (speed < DriverProfiles.deadzoneValues && -DriverProfiles.deadzoneValues < speed) {
      speed = 0;
    }
    if (rotation < DriverProfiles.deadzoneValues && -DriverProfiles.deadzoneValues < rotation) {
      rotation = 0;
    }
    
    /*
     * If sniper mode is enabled, reduce to 40%
     * Otherwise reduce speed to 95% and rotation to 60%
     */
    if (SniperMode.driveSniperMode) {
      speed *= DrivebaseConstants.DRIVE_SNIPER_SPEED;
      rotation *= DrivebaseConstants.DRIVE_SNIPER_SPEED;
    }
    else {
      speed *= DrivebaseConstants.SPEED_REDUCTION;
      rotation *= DrivebaseConstants.ROTATION_REDUCTION;
    }

    robotDrive.arcadeDrive(rotation, speed, DriverProfiles.squareInputs);

    robotDrive.feed();
  }

  public void autonomousArcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("TURN OUTPUT", rotation);
  
    robotDrive.arcadeDrive(speed, rotation);

    System.out.println("ROTATION : " + rotation);
    
    robotDrive.feed();
  }

  public Command arcadeDriveCMD(double speed, double rotation) {
    return new ArcadeCommand(() -> speed, () -> rotation,  this);
  }

  public double getLeftFrontEncoderPosition() {
    return leftFrontEncoder.getPosition();
  }

  public double getRightFrontEncoderPosition() {
    return -rightFrontEncoder.getPosition();
  }

  public double getLeftFrontEncoderVelocity() {
    return -leftFrontEncoder.getVelocity();
  }

  public double getRightFrontEncoderVelocity() {
    return rightFrontEncoder.getVelocity();
  }

  public double getLefrtBackEncoderVelocity() {
    return -leftBackEncoder.getVelocity();
  }

  public double getRightBackEncoderVelocity() {
    return rightBackEncoder.getVelocity();
  }

  public double getGyroHeading() {
    return getRotation2d().getDegrees();
  }

  public double getLeftFrontMotorTemp() {
    return leftFrontMotor.getMotorTemperature();
  }

  public double getLeftBackMotorTemp() {
    return leftBackMotor.getMotorTemperature();
  }

  public double getRightFrontMotorTemp() {
    return rightFrontMotor.getMotorTemperature();
  }

  public double getRightBackMotorTemp() {
    return rightBackMotor.getMotorTemperature();
  }

  public double getGyroRoll() {
    return navX.getRoll();
  }

  public double getGyroPitch() {
    return navX.getPitch();
  }

  public double getGyroYaw() {
    return navX.getYaw();
  }

  public AHRS getGyro() {
    return navX;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftFrontEncoderVelocity(), getRightFrontEncoderVelocity());
  }

  public Field2d getField() {
    return field;
  }

  public void setTankDriveVolts(double rightVolts, double leftVolts) {
    // leftVolts *= -1;
    // rightVolts *= -1;

    leftFrontMotor.setVoltage(leftVolts);
    leftBackMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
    rightBackMotor.setVoltage(rightVolts);
    robotDrive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    robotDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    navX.calibrate();
  }


  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    // navX.reset();
    // navX.calibrate();
    odometry.resetPosition(getRotation2d(), getLeftFrontEncoderPosition(), getRightFrontEncoderPosition(), pose);
  }

  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }

 public Command followPath(PathPlannerTrajectory trajectory, boolean isFirstPath) {
   if (isFirstPath) {
     this.resetOdometry(trajectory.getInitialPose());
   }

   return new PPRamseteCommand (
     trajectory, 
     this::getPose, 
     new RamseteController(AutonomousConstants.RAMSETE_B, AutonomousConstants.RAMSETE_ZETA), 
     new SimpleMotorFeedforward(AutonomousConstants.VOLTS, AutonomousConstants.VOLT_SECONDS_PER_METER, AutonomousConstants.VOLT_SECONDS_SQUARED_PER_METER), 
     AutonomousConstants.DRIVE_KINEMATICS, 
     this::getWheelSpeeds, 
     new PIDController(0.0009, 0, 0),
     new PIDController(0.0009, 0, 0), //0.00075 //0.0007 //0.0008 //0.0009
     this::setTankDriveVolts,
     this
     );
 }

 public Command followPathGroup(List<PathPlannerTrajectory> trajectory, boolean AllianceColor, HashMap<String, Command> eventMap) {
    RamseteAutoBuilder bAutoBuilder = new RamseteAutoBuilder(
      this::getPose, 
      this::resetOdometry, 
    new RamseteController(AutonomousConstants.RAMSETE_B, 
                          AutonomousConstants.RAMSETE_ZETA),
                          AutonomousConstants.DRIVE_KINEMATICS, 
    new SimpleMotorFeedforward(AutonomousConstants.VOLTS, 
                            AutonomousConstants.VOLT_SECONDS_PER_METER, 
                            AutonomousConstants.VOLT_SECONDS_SQUARED_PER_METER), 
    this::getWheelSpeeds, 
    new PIDConstants(0.0012, 0, 0), // 0.0015 // 0.001 
    this::setTankDriveVolts, 
    eventMap, 
    AllianceColor, 
    this
    );

  return bAutoBuilder.fullAuto(trajectory);
}

  public Command turnCommand(double setpoint) {
    return new TurnCommand(
        new ProfiledPIDController(
          0.01, 0, 0, 
        new TrapezoidProfile.Constraints(200, 200)),
        () -> setpoint,
        () -> getPose().getRotation().getDegrees(),
        this
    );
  }

  public Command turnTo0CMD() {
    return turnCommand(0);
  }

  public Command turnTo180CMD() {
    return turnCommand(180);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(navX.getRotation2d().getRadians());
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("LEFT FRONT ENCODER POS: ", getLeftFrontEncoderPosition());
    SmartDashboard.putNumber("RIGHT FRONT ENCODER POS: ", getRightFrontEncoderPosition());
    SmartDashboard.putNumber("LEFT FRONT SPEED", -getLeftFrontEncoderVelocity());
    SmartDashboard.putNumber("RIGHT FRONT SPEED", getRightFrontEncoderVelocity());
    SmartDashboard.putNumber("RIGHT BACK SPEED", getRightBackEncoderVelocity());
    SmartDashboard.putNumber("LEFT BACK SPEED", -getLefrtBackEncoderVelocity());
    // SmartDashboard.putNumber("LEFT FRONT TEMP: ", getLeftFrontMotorTemp());
    // SmartDashboard.putNumber("LEFT BACK TEMP: ", getLeftBackMotorTemp());
    // SmartDashboard.putNumber("RIGHT FRONT TEMP: ", getRightFrontMotorTemp());
    // SmartDashboard.putNumber("RIGHT BACK TEMP: ", getRightBackMotorTemp());
    // SmartDashboard.putNumber("GYRO PITCH ", getGyroPitch());
    // SmartDashboard.putNumber("GYRO ROLL", getGyroRoll());
    SmartDashboard.putNumber("GYRO YAW", getGyroYaw());

    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());
  }
}