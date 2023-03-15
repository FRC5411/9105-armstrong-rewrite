/*
 * NOTE: fullDriveBackwards = 3.60 metres
 *       driveForward = 1.40 metres
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.GlobalVars.GameStates;
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
      driveForwardsTime = 13.0;
      dockingTime = 15.0;
      
      System.out.println("Command AUTONOMOUS has started");
    }


    public void stopAll() {
      robotArm.setArm(0);
      robotIntake.setspin(0);
      robotDrive.setLeftRightMotors(0, 0);
    }

    /*
     * This auton is for the grid closest to the substations,
     * it will aim, outtake, retract, and move to community
     */
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
      else if (timeElapsed < fullDriveBackTime) {
        stopAll();
        robotDrive.setLeftRightMotors(-AutonomousConstants.DRIVE_SPEED, -AutonomousConstants.DRIVE_SPEED);
      }
    }

    /*
     * This auton is for the middle grid
     * it will aim, outtake, retract, and move to community
     * by crossing the charge station; then it will move
     * forward on to the charge station and attempt to dock
     * and engage
     */
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
        robotDrive.setLeftRightMotors(-AutonomousConstants.DRIVE_SPEED, -AutonomousConstants.DRIVE_SPEED);
      }
      else if (timeElapsed < driveForwardsTime) {
        stopAll();
        robotDrive.setLeftRightMotors(AutonomousConstants.DRIVE_SPEED, AutonomousConstants.DRIVE_SPEED);
      }
      else if (timeElapsed < dockingTime) {
        stopAll();
        new AutoEngageCommand(robotDrive);
      }
    }

    /*
     * This auton is for the grid closest to the wall,
     * it will aim, outtake, retract, and move to community
     */
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
        robotDrive.setLeftRightMotors(-AutonomousConstants.DRIVE_SPEED, -AutonomousConstants.DRIVE_SPEED);
      }
    }

    @Override
    public void execute() {
      timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

      if (GameStates.chosenAuton == 1) {
        topAuton();
      } 
      else if (GameStates.chosenAuton == 2) {
        centerAuton();
      }
      else if (GameStates.chosenAuton == 3) {
        bottomAuton();
      } 
      else {
        System.out.println("CRITICAL ERROR: NO AUTON CHOSEN! \nCURRENT AUTON: " + GameStates.chosenAuton);
      }
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