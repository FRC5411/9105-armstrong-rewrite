
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoEngageCommand extends ProfiledPIDCommand {
  
  public AutoEngageCommand(DriveSubsystem robotDrive) {
    super(
        new ProfiledPIDController(
            0.038,
            0,
            0.001,
            new TrapezoidProfile.Constraints(1, 0.5)),
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
