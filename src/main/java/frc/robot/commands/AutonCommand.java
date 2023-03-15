
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutonCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem robotArm;
    private DriveSubsystem robotDrive;
    private IntakeSubsystem robotIntake;

    private double autonomousStartTime;
    private double timeElapsed;

    private double scoringTime;
    private double outtakeTime;
    private double retractingTime;
    private double fullDriveBackTime;
    private double halfDriveBackTime;
    private double driveForwardsTime;
    private double dockingTime;

    public AutonCommand(DriveSubsystem robotDrive, ArmSubsystem robotArm, IntakeSubsystem robotIntake) {
      this.robotArm = robotArm;
      this.robotDrive = robotDrive;
      this.robotIntake = robotIntake;
    }

    @Override
    public void initialize() {
      System.out.println("STARTING TIMER...");
      autonomousStartTime = Timer.getFPGATimestamp();
      
      System.out.println("INITIALIZING VARIABLES...");
      scoringTime = 2.0;
      outtakeTime = 3.0;
      retractingTime = 6.0;
      fullDriveBackTime = 11.0;
      halfDriveBackTime = 9.0;
      driveForwardsTime = 13.0;
      dockingTime = 15.0;
      
      System.out.println("Command AUTONOMOUS has started");
    }


    public void stopAll() {
      robotArm.setArm(0);
      robotIntake.setspin(0);
      robotDrive.setLeftRightMotors(0, 0);
    }

    public void topAuton() {
      if (timeElapsed < scoringTime) {
        stopAll();
        new ArmCommand(robotArm, 175);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.spinout();
      }
      else if (timeElapsed < retractingTime) {
        stopAll();
        new ArmCommand(robotArm, 0);
      }
      else if (timeElapsed < halfDriveBackTime) {
        stopAll();
        robotDrive.setLeftRightMotors(-1, -1);
      }
    }

    public void centerAuton() {
      if (timeElapsed < scoringTime) {
        stopAll();
        new ArmCommand(robotArm, 175);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.spinout();
      }
      else if (timeElapsed < retractingTime) {
        stopAll();
        new ArmCommand(robotArm, 0);
      }
      else if (timeElapsed < fullDriveBackTime) {
        stopAll();
        robotDrive.setLeftRightMotors(-1, -1);
      }
      else if (timeElapsed < driveForwardsTime) {
        stopAll();
        robotDrive.setLeftRightMotors(1, 1);
      }
      else if (timeElapsed < dockingTime) {
        stopAll();
        new AutoEngageCommand(robotDrive);
      }
    }

    public void bottomAuton() {
      if (timeElapsed < scoringTime) {
        stopAll();
        new ArmCommand(robotArm, 175);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.spinout();
      }
      else if (timeElapsed < retractingTime) {
        stopAll();
        new ArmCommand(robotArm, 0);
      }
      else if (timeElapsed < fullDriveBackTime) {
        stopAll();
        robotDrive.setLeftRightMotors(-1, -1);
      }
    }

    @Override
    public void execute() {
      timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;


    }
    
    @Override
    public void end(boolean interrupted) {
      System.out.println("Command AUTONOMOUS has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}