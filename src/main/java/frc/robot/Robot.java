package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Command armCmd;
  private ArmSubsystem robotArm;

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Gyro Proportional", 0.035);
    SmartDashboard.putNumber("Gyro Integral", 0.0);
    SmartDashboard.putNumber("Gyro Derivitive", 0.005);
    SmartDashboard.putNumber("GYRO CALC", 0);

    m_robotContainer = new RobotContainer();
    robotArm = m_robotContainer.getRobotArm();
    armCmd = new ArmCommand(robotArm);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().schedule(armCmd);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
