package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoEngageCommand extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private AHRS m_gyro;
  
  private double m_setpoint;
  private PIDController m_pid;
  
  private double gyroPitch;
  private double kP;
  private double kI;
  private double kD;

  private DriveSubsystem m_robotDrive;

  public AutoEngageCommand(AHRS gyro, DriveSubsystem robotDrive) {
    this.m_gyro = gyro;
    this.m_robotDrive = robotDrive;
    addRequirements(robotDrive);
  }

  @Override
  public void initialize() {
    m_setpoint = 0;

    // WARNING, must tune these
    kP = 0;
    kI = 0;
    kD = 0;

    m_pid = new PIDController(kP, kI, kD);

    m_pid.setTolerance(2.5);
    System.out.println("The AUTO ENGAGEMENT command has been INITIALIZED");
  }

  @Override
  public void execute() {
    gyroPitch = m_gyro.getPitch();

    double calc = m_pid.calculate(gyroPitch, m_setpoint);

    m_robotDrive.arcadeDrive(calc, 0.0);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("The AUTO ENGAGEMENT command has been ENDED");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
