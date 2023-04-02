
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

/* 
public class AutoEngageCommand extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private AHRS m_gyro;
  
  private double m_setpoint;
  private PIDController m_pid;
  
  private double gyroRoll;
  private double kP;
  private double kI;
  private double kD;

  private DriveSubsystem m_robotDrive;
  private ArcadeCommand arcadeCmd;

  public AutoEngageCommand(DriveSubsystem robotDrive) {
    this.m_robotDrive = robotDrive;
    this.m_gyro = robotDrive.getGyro();
  }

@Override
public void initialize() {
    m_setpoint = 0;

    // WARNING, must tune these
    kP = SmartDashboard.getNumber("Gyro Proportional", 0.035);
    kI = SmartDashboard.getNumber("Gyro Integral", 0.0);
    kD = SmartDashboard.getNumber("Gyro Derivitive", 0.005);

    m_pid = new PIDController(kP, kI, kD);

    m_pid.setTolerance(3);
    System.out.println("Command AUTO ENGAGE has started");
  }

  @Override
  public void execute() {
    gyroRoll = m_gyro.getRoll();

    double calc = m_pid.calculate(gyroRoll, m_setpoint);

    System.out.println("Gyro Pitch: " + gyroRoll + "\n" + "PID calculation: " + calc);

    arcadeCmd = new ArcadeCommand(() -> calc, () -> 0, m_robotDrive);
    CommandScheduler.getInstance().schedule(arcadeCmd);
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(arcadeCmd);
    System.out.println("Command AUTO ENGAGE has ended");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
} 
*/

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoEngageCommand extends ProfiledPIDCommand {

  /** Creates a new AutoBalance. */
  
  public AutoEngageCommand(DriveSubsystem robotDrive) {
    super(
        new ProfiledPIDController(
            0.038,
            0,
            0.001,
            new TrapezoidProfile.Constraints(1, 0.75)),
        robotDrive::getGyroRoll,
        // This should return the goal (can also be a constant)
        2.5,
        (output, setpoint) -> {
          SmartDashboard.putNumber("GYRO CALC", output);
          if (output < 0) {
            output = output * 1.05;
          }

          robotDrive.arcadeDrive(output, 0);
        });

    addRequirements(robotDrive);

    getController().setTolerance(1);
  }

  public void initialize() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void end() {}
}









