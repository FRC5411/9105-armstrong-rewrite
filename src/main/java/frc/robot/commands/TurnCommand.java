
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnCommand extends PIDCommand {
  
    public TurnCommand (DriveSubsystem robotDrive, double setpoint) {
      super(
        new PIDController(
          // Might work idk :T
          DrivebaseConstants.P_DRIVE_TURN, 
          DrivebaseConstants.I_DRIVE_TURN, 
          DrivebaseConstants.D_DRIVE_TURN), 
        robotDrive::getGyroYaw, 
        setpoint, 
        output -> robotDrive.autonomousArcadeDrive(0, output), 
        robotDrive);

      getController().enableContinuousInput(-180, 180);

      if(getController().atSetpoint()){
        System.out.println("❤AT SETPOINT❤");
      }
      // Tune these before moving to constants
      getController()
        .setTolerance(1.0, 10.0);
    }
  
    public void initialize() {}

    @Override
    public boolean isFinished() {
      return false;
    }
  
    public void end() {}
  }
