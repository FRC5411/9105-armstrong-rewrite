
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.*;

public class AutonCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem robotArm;
    private DriveSubsystem robotDrive;
    private IntakeSubsystem robotIntake;

    private ArmCommand armCmd;
    private ArmCommand retractCmd;
    private ArmCommand armOutCmd;
    private ArcadeCommand arcadeCmd;
    private ArcadeCommand forwardArcadeCmd;
    //private AutoEngageCommand autoEngageCmd;

    private double autonomousStartTime;
    private double timeElapsed;
    private int path;

    private double scoringTime;
    private double outtakeTime;
    private double retractingTime;
    private double fullDriveBackTime;
    private double driveForwardsTime;
    //private double dockingTime;

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
      scoringTime = 1.9;
      outtakeTime = 2.2;
      retractingTime = 5.2;
      fullDriveBackTime = 8.5;
      driveForwardsTime = 12.0;

      /*
      scoringTime = 1.9;
      outtakeTime = 0.3;
      retractingTime = 3;
      fullDriveBackTime = 3.3;
      driveForwardsTime = 3.5
       */

      //dockingTime = 14.0;
      GameStates.chosenAuton = path;

      armCmd = new ArmCommand(robotArm, 175);
      retractCmd = new ArmCommand(robotArm, 0);
      armOutCmd = new ArmCommand(robotArm, 260.5);
      
      System.out.println("Command AUTONOMOUS has started");
    }

    public void stopAll() {
      robotArm.setArm(0);
      robotIntake.setspin(0);
    }

    /*
     * This auton is for the grid closest to the substations,
     * it will aim, outtake (cone), retract, and move to community
     */

    public void coneMob() {
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
        arcadeCmd = new ArcadeCommand(() -> 0.75, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(arcadeCmd);
      }
      else {
        CommandScheduler.getInstance().cancel(arcadeCmd);
        stopAll();
      }
    }

    /*
     * This auton is for the middle grid
     * it will aim, outtake (cone), retract, and move to community
     * by crossing the charge station; then it will move
     * forward on to the charge station and attempt to dock
     * and engage
     */

    public void coneMobDock() {
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
        arcadeCmd = new ArcadeCommand(() -> 0.75, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(arcadeCmd);
      }
      else if (timeElapsed < driveForwardsTime) {
        stopAll();
        CommandScheduler.getInstance().cancel(arcadeCmd);
        forwardArcadeCmd = new ArcadeCommand( () -> -0.75, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(forwardArcadeCmd);
      } /* 
      else if (timeElapsed < dockingTime) {
        stopAll();
        //autoEngageCmd = new AutoEngageCommand(robotDrive);
        //CommandScheduler.getInstance().schedule(autoEngageCmd);
      }*/
      else {
        CommandScheduler.getInstance().cancel(forwardArcadeCmd);
        //CommandScheduler.getInstance().cancel(autoEngageCmd);
        stopAll();
      }
    }

    /*
     * This auton will only aim, outtake (cone)
     * and will not move at all
     */

    public void coneOnly() {
      // new RunCommand(() -> intake.set(1), intake).withTimeout(1)
      // .andThen(new )
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
      }
      else {
        stopAll();
      }
    }

    public SequentialCommandGroup testAuton(){
      return new SequentialCommandGroup(
        new ArmCommand(robotArm, 175).repeatedly().withTimeout(1),
        new InstantCommand( () -> { robotIntake.setspin(-0.5); }).withTimeout(1),
        new ArmCommand(robotArm, 59).withTimeout(1),
        new ArcadeCommand(() -> 0.75, () -> 0.0, robotDrive).withTimeout(3)
      );
    }
    /*
     * This auton will only aim, outtake (cube)
     * and will not move at all
     */

     public void cubeOnly() {
      if (timeElapsed < scoringTime) {
        stopAll();
        CommandScheduler.getInstance().schedule(armCmd);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.setspin(-0.5);
      }
      else if (timeElapsed < retractingTime) {
        stopAll();
        CommandScheduler.getInstance().cancel(armCmd);
        CommandScheduler.getInstance().schedule(retractCmd);
      }
      else if (timeElapsed < fullDriveBackTime) {
        CommandScheduler.getInstance().cancel(retractCmd);
        robotArm.setArm(0);
      }
      else {
        stopAll();
      }
    }

    // Please be very very very very careful when using
    // this auton:

    /*
     * This auton will score, turn, then go and
     * extend arms to get ready for next cargo game piece
     */

     public void coneMobTurnExtend() {
      if (timeElapsed < scoringTime) {
        stopAll();
        CommandScheduler.getInstance().schedule(armCmd);
      }
      else if (timeElapsed < outtakeTime) {
        stopAll();
        robotIntake.setspin(0.5);
      }
      // Instead of retracting it will just turn
      else if (timeElapsed < 4.90) {
        stopAll();
        CommandScheduler.getInstance().cancel(armCmd);
        CommandScheduler.getInstance().schedule(retractCmd);
        forwardArcadeCmd = new ArcadeCommand(() -> 0.75, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(forwardArcadeCmd);
      }
      // Instead of going backwards it goes forward since it turns
      else if (timeElapsed < 6.75) {
        CommandScheduler.getInstance().cancel(forwardArcadeCmd);
        CommandScheduler.getInstance().cancel(retractCmd);
        robotArm.setArm(0);
        arcadeCmd = new ArcadeCommand(() -> 0, () -> 0.75, robotDrive);
        System.out.println("TURNING TIME: " + timeElapsed);
        CommandScheduler.getInstance().schedule(arcadeCmd);
      }
      else if (timeElapsed < 9.3) {
        CommandScheduler.getInstance().cancel(arcadeCmd);
        CommandScheduler.getInstance().schedule(armOutCmd);
        robotIntake.setspin(-0.5);
      }
      else if (timeElapsed < 9.85) {
        CommandScheduler.getInstance().schedule(armOutCmd);
        arcadeCmd = new ArcadeCommand(() -> -0.75, () -> 0, robotDrive);
        CommandScheduler.getInstance().schedule(arcadeCmd);
        robotIntake.setspin(-0.5);
      }
      else {
        CommandScheduler.getInstance().cancel(armOutCmd);
        CommandScheduler.getInstance().cancel(arcadeCmd);
        robotIntake.setspin(0);
        stopAll();
      }
    }

    @Override
    public void execute() {
      timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

      if (GameStates.chosenAuton == 1) {
        coneMob();
      } 
      else if (GameStates.chosenAuton == 2) {
        coneMobDock();
      }
      else if (GameStates.chosenAuton == 3) {
        coneOnly();
      }
      else if (GameStates.chosenAuton == 4) {
        cubeOnly();
      }
      else if (GameStates.chosenAuton == 5) {
        coneMobTurnExtend();
      }
      else if(GameStates.chosenAuton == 6){
        CommandScheduler.getInstance().schedule(testAuton());
      }
      else {
        System.out.println("CRITICAL ERROR: NO AUTON CHOSEN! \nCURRENT AUTON: " + GameStates.chosenAuton);
      }
    }
    
    @Override
    public void end(boolean interrupted) {
      CommandScheduler.getInstance().cancel(arcadeCmd);
      CommandScheduler.getInstance().cancel(testAuton());
      CommandScheduler.getInstance().cancelAll();
      System.out.println("Command AUTONOMOUS has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}