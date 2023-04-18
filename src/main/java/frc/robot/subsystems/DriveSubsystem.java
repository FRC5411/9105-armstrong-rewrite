
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController; // i like boys
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.ArcadeCommand;
import edu.wpi.first.wpilibj.SPI;

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

    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    leftFrontEncoder = leftFrontMotor.getEncoder();
    leftBackEncoder = leftBackMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    rightBackEncoder = rightBackMotor.getEncoder();

    leftFrontEncoder.setInverted(true);
    leftFrontEncoder.setInverted(true);
    leftFrontEncoder.setInverted(true);
    leftFrontEncoder.setInverted(true);

    leftFrontEncoder.setPositionConversionFactor(AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);
    rightFrontEncoder.setPositionConversionFactor(AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

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

    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

    field = new Field2d();

    SmartDashboard.putData("Field", field);

    navX.calibrate();
    navX.reset();

    resetEncoders();
    resetOdometry(getPose());
  }

  public Pose2d returnRobotPose(){
    return getPose();
  }

  public void arcadeDrive(double speed, double rotation) {
    /*
     * If LY is between 0.1 & -0.1, if so set to 0 to decrease sensitivity
     * Otherwise, square inputs accordingly
     */

    if (speed < DrivebaseConstants.DEADZONE && -DrivebaseConstants.DEADZONE < speed) {
      speed = 0;
    }
    else if (speed > 0) {
      speed *= speed;
    }
    else {
      speed *= -speed;
    }

    if (rotation < DrivebaseConstants.DEADZONE && -DrivebaseConstants.DEADZONE < rotation) {
      rotation = 0;
    }
    else if (rotation > 0) {
      rotation *= -rotation;
    }
    else {
      rotation *= rotation;
    }
    
    /*
     * If sniper mode is enabled, reduce to 40%
     * Otherwise reduce speed to 95% and rotation to 60%
     */
    if (SniperMode.driveSniperMode) {
      speed *= DrivebaseConstants.DRIVE_SNIPER_SPEED;
    }
    else {
      speed *= DrivebaseConstants.SPEED_REDUCTION;
    }

    if (SniperMode.driveSniperMode) {
      rotation *= DrivebaseConstants.DRIVE_SNIPER_SPEED;
    }
    else {
      rotation *= DrivebaseConstants.ROTATION_REDUCTION;
    }

    robotDrive.arcadeDrive(speed, rotation);

    robotDrive.feed();
  }

  public void autonomousArcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("TURN OUTPUT", rotation);
    
    rotation = rotation * 0.1;
    rotation = rotation * -1;

    robotDrive.arcadeDrive(speed, rotation);

    System.out.println("ROTATION : " + rotation);
    
    robotDrive.feed();
  }

  public Command arcadeDriveCMD(double speed, double rotation) {
    return new ArcadeCommand(() -> speed, () -> rotation,  this);
  }

  public double getLeftFrontEncoderPosition() {
    return -leftFrontEncoder.getPosition();
  }

  public double getRightFrontEncoderPosition() {
    return rightFrontEncoder.getPosition();
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
    return navX.getRotation2d().getDegrees();
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

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    leftFrontMotor.setVoltage(-leftVolts);
    leftBackMotor.setVoltage(-leftVolts);
    rightFrontMotor.setVoltage(-rightVolts);
    rightBackMotor.setVoltage(-rightVolts);
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
    navX.reset();
    navX.calibrate();
    odometry.resetPosition(navX.getRotation2d(), getLeftFrontEncoderPosition(), getGyroHeading(), pose);
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

    return new PPRamseteCommand(
      trajectory, 
      this::getPose, 
      new RamseteController(AutonomousConstants.RAMSETE_B, AutonomousConstants.RAMSETE_ZETA), 
      new SimpleMotorFeedforward(AutonomousConstants.VOLTS, AutonomousConstants.VOLT_SECONDS_PER_METER, AutonomousConstants.VOLT_SECONDS_SQUARED_PER_METER), 
      AutonomousConstants.DRIVE_KINEMATICS, 
      this::getWheelSpeeds, 
      new PIDController(0.001, 0, 0), 
      new PIDController(0.001, 0, 0), 
      this::setTankDriveVolts,
      this
      );
  }

  @Override
  public void periodic() {
    odometry.update(navX.getRotation2d(), -leftFrontEncoder.getPosition(), -rightFrontEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("LEFT FRONT ENCODER POS: ", getLeftFrontEncoderPosition());
    SmartDashboard.putNumber("RIGHT FRONT ENCODER POS: ", getRightFrontEncoderPosition());
    SmartDashboard.putNumber("LEFT FRONT TEMP: ", getLeftFrontMotorTemp());
    SmartDashboard.putNumber("LEFT BACK TEMP: ", getLeftBackMotorTemp());
    SmartDashboard.putNumber("RIGHT FRONT TEMP: ", getRightFrontMotorTemp());
    SmartDashboard.putNumber("RIGHT BACK TEMP: ", getRightBackMotorTemp());
    SmartDashboard.putNumber("GYRO PITCH ", getGyroPitch());
    SmartDashboard.putNumber("GYRO ROLL", getGyroRoll());
    SmartDashboard.putNumber("GYRO YAW", getGyroYaw());

    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
  }
}