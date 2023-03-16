/*
 * NOTE: fullDriveBackwards = 3.60 metres - 3.2s
 *       driveForward = 1.40 metres - 1.48
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.*;

public class AutonCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem robotArm;
    private DriveSubsystem robotDrive;
    private IntakeSubsystem robotIntake;

    private ArmCommand armCmd;
    private ArmCommand retractCmd;
    private ArcadeCommand arcadeCmd;

    private double autonomousStartTime;
    private double timeElapsed;
    private int path;

    private double scoringTime;
    private double outtakeTime;
    private double retractingTime;
    private double fullDriveBackTime;
    private double driveForwardsTime;
    private double dockingTime;

    public AutonCommand(DriveSubsystem robotDrive, ArmSubsystem robotArm, IntakeSubsystem robotIntake, int path) {
      this.robotArm = robotArm;
      this.robotDrive = robotDrive;
      this.robotIntake = robotIntake;
      this.path = path;
    }

    @Override
    public void initialize() {
      System.out.println("STARTING TIMER...");
      autonomousStartTime = Timer.getFPGATimestamp();
      
      System.out.println("INITIALIZING VARIABLES...");
      scoringTime = 2.9;
      outtakeTime = 3.2;
      retractingTime = 6.2;
      fullDriveBackTime = 9.5;
      driveForwardsTime = 11.2;
      dockingTime = 14.0;
      GameStates.chosenAuton = path;

      armCmd = new ArmCommand(robotArm, 175);
      retractCmd = new ArmCommand(robotArm, 0);
      
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
        CommandScheduler.getInstance().schedule(armCmd);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.setspin(0.5);
      }
      else if (timeElapsed < retractingTime) {
        stopAll();
        CommandScheduler.getInstance().cancel(armCmd);
        CommandScheduler.getInstance().schedule(retractCmd);
      }
      else if (timeElapsed < fullDriveBackTime) {
        CommandScheduler.getInstance().cancel(retractCmd);
        robotArm.setArm(0);
        arcadeCmd = new ArcadeCommand(() -> 1, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(arcadeCmd);
      }
      else {
        CommandScheduler.getInstance().cancel(arcadeCmd);
        robotDrive.enableDriveMotorBrakes(true);
        stopAll();
      }
    }

    /*
     * This auton is for the middle grid
     * it will aim, outtake, retract, and move to community
     * by crossing the charge station; then it will move
     * forward on to the charge station and attempt to dock
     * and engage
     */

     /* 
    public void centerAuton() {
      if (timeElapsed < scoringTime) {
        stopAll();
        new ArmCommand(robotArm, 175);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.setspin(0.5);
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
    }*/

    public void centerAuton() {
      if (timeElapsed < scoringTime) {
        stopAll();
        CommandScheduler.getInstance().schedule(armCmd);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.setspin(0.5);
      }
      else if (timeElapsed < retractingTime) {
        stopAll();
        CommandScheduler.getInstance().cancel(armCmd);
        CommandScheduler.getInstance().schedule(retractCmd);
      }
      else if (timeElapsed < fullDriveBackTime) {
        CommandScheduler.getInstance().cancel(retractCmd);
        robotArm.setArm(0);
        arcadeCmd = new ArcadeCommand(() -> 1, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(arcadeCmd);
      }
      else {
        CommandScheduler.getInstance().cancel(arcadeCmd);
        robotDrive.enableDriveMotorBrakes(true);
        stopAll();
      }
    }

    /*
     * This auton is for the grid closest to the wall,
     * it will aim, outtake, retract, and move to community
     */
    public void bottomAuton() {
      if (timeElapsed < scoringTime) {
        stopAll();
        CommandScheduler.getInstance().schedule(armCmd);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.setspin(0.5);
      }
      else if (timeElapsed < retractingTime) {
        stopAll();
        CommandScheduler.getInstance().cancel(armCmd);
        CommandScheduler.getInstance().schedule(retractCmd);
      }
      else if (timeElapsed < fullDriveBackTime) {
        CommandScheduler.getInstance().cancel(retractCmd);
        robotArm.setArm(0);
        arcadeCmd = new ArcadeCommand(() -> 1, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(arcadeCmd);
      }
      else {
        CommandScheduler.getInstance().cancel(arcadeCmd);
        robotDrive.enableDriveMotorBrakes(true);
        stopAll();
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
      CommandScheduler.getInstance().cancel(arcadeCmd);
      CommandScheduler.getInstance().cancelAll();
      System.out.println("Command AUTONOMOUS has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}