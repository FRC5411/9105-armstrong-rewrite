package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.commands.Arm.HoldArmCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private HoldArmCommand holdArmCmd;
  
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("GYRO CALC", 0);
    SmartDashboard.putNumber("ARM OUTPUT", 0);
    SmartDashboard.putNumber("TURN OUTPUT", 0);

    m_robotContainer = new RobotContainer();

    DebugInfo.initialGyroPitch = m_robotContainer.getRobotDrive().getGyroPitch();

    holdArmCmd = new HoldArmCommand(m_robotContainer.getRobotArm());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().schedule(holdArmCmd);
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

    DebugInfo.profiledArmP = SmartDashboard.getNumber("ARM P", 0.01);
    DebugInfo.profiledArmI = SmartDashboard.getNumber("ARM I", 0.0);
    DebugInfo.profiledArmD = SmartDashboard.getNumber("ARM D", 0.0);
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
