
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars.SniperMode;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;

  private RelativeEncoder leftFrontEncoder;
  private RelativeEncoder leftBackEncoder;
  private RelativeEncoder rightFrontEncoder;
  private RelativeEncoder rightBackEncoder;

  private DifferentialDrive robotDrive;

  private Pose2d initialPose;

  private AHRS navX;

  private DifferentialDrivePoseEstimator odometry;

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

    leftFrontEncoder = leftFrontMotor.getEncoder();
    leftBackEncoder = leftBackMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    rightBackEncoder = rightBackMotor.getEncoder();

    leftFrontEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    leftBackEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    rightFrontEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    rightBackEncoder.setVelocityConversionFactor(
      AutonomousConstants.LINEAR_DIST_CONVERSION_FACTOR);

    robotDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);

    initialPose = new Pose2d();

    navX = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDrivePoseEstimator(
      AutonomousConstants.DRIVE_KINEMATICS, 
      navX.getRotation2d(), 
      leftFrontEncoder.getPosition(), 
      rightFrontEncoder.getPosition(), 
      initialPose);

    resetEncoders();
    zeroHeading();
    resetOdometry(getPose());
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
    // return navX.getRoll();
    return navX.getRoll();
  }

  public double getGyroPitch() {
    return navX.getRoll();
  }

  public double getGyroYaw() {
    return navX.getYaw();
  }

  public AHRS getGyro() {
    return navX;
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftFrontEncoderVelocity(), getRightFrontEncoderVelocity());
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    leftFrontMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
    robotDrive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    robotDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    navX.calibrate();
    //navX.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(navX.getRotation2d(), getLeftFrontEncoderPosition(), getGyroHeading(), pose);
  }

  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    odometry.update(navX.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

    SmartDashboard.putNumber("LEFT FRONT ENCODER POS: ", getLeftFrontEncoderPosition());
    SmartDashboard.putNumber("RIGHT FRONT ENCODER POS: ", getRightFrontEncoderPosition());
    SmartDashboard.putNumber("LEFT FRONT TEMP: ", getLeftFrontMotorTemp());
    SmartDashboard.putNumber("LEFT BACK TEMP: ", getLeftBackMotorTemp());
    SmartDashboard.putNumber("RIGHT FRONT TEMP: ", getRightFrontMotorTemp());
    SmartDashboard.putNumber("RIGHT BACK TEMP: ", getRightBackMotorTemp());
    SmartDashboard.putNumber("GYRO ROLL ", getGyroRoll());
    //SmartDashboard.putNumber("GYRO YAW: ", getGyroYaw());
    //SmartDashboard.putNumber("GYRO PITCH: ", getGyroPitch());

    //System.out.println(getGyroRoll());
  }
}