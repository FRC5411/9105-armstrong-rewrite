// In Java We Trust

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.AutoEngageCommand;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  private CommandXboxController controller;
  private CommandGenericHID buttonBoard;

  private DriveSubsystem robotDrive;
  private ArmSubsystem robotArm;
  private IntakeSubsystem robotIntake;
  private AutonSubsystem robotAuton;

  private PowerDistribution PDH;

  private SendableChooser<Command> autonChooser;

  Trigger chosenButton;

  // private Command PeriodicArmCommand;

  public RobotContainer() {

    
    controller = new CommandXboxController(DrivebaseConstants.CONTROLLER_PORT);
    buttonBoard = new CommandGenericHID(ButtonBoardConstants.BUTTON_BOARD_PORT);

    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();
    robotAuton = new AutonSubsystem(robotDrive, robotArm, robotIntake);

    PDH = new PowerDistribution(DrivebaseConstants.PDH_PORT_CANID, ModuleType.kRev);

    autonChooser = new SendableChooser<>();

    robotDrive.setDefaultCommand(new ArcadeCommand(
      () -> controller.getLeftY(),
      () -> controller.getRightX(),
      robotDrive
      ));

    Shuffleboard.getTab("Autonomous: ").add(autonChooser);

    autonChooser.addOption("CONE MOBILITY", 
      robotAuton.autonomousCmd(1));

    autonChooser.addOption("CONE MOBILITY DOCK", 
      robotAuton.autonomousCmd(2));

    autonChooser.addOption("CONE SCORE ONLY", 
      robotAuton.autonomousCmd(3));
    
    autonChooser.addOption("CUBE SCORE ONLY", 
      robotAuton.autonomousCmd(4));

    autonChooser.addOption("CONE MOBILITY EXTEND GRAB", 
      robotAuton.autonomousCmd(5));

    autonChooser.addOption("CONE MOBILITY TURN EXTEND", 
      robotAuton.autonomousCmd(6));

    autonChooser.addOption("RED CONE MOBILITY TURN", 
      robotAuton.autonomousCmd(7));

    configureBindings();
  }

  private void configureBindings() {

    //////////////////// XBOX CONTROLLER //////////////////// 
    controller.leftTrigger()
      .whileTrue(new InstantCommand( () -> { SniperMode.driveSniperMode = true; }))
      .whileFalse(new InstantCommand( () -> { SniperMode.driveSniperMode = false; }));
    
    controller.leftBumper()
      .whileTrue(new InstantCommand( () -> {
        if (GameStates.isCube) { robotIntake.spinout(); }
        else { robotIntake.spinin(); }
      }))
      .whileFalse(new InstantCommand( () -> { robotIntake.spinoff(); }));

    controller.rightBumper()
      .whileTrue(new InstantCommand( () -> {
        if (GameStates.isCube) { robotIntake.spinin(); }
        else { robotIntake.spinout(); }
      }))
      .whileFalse(new InstantCommand( () -> { robotIntake.spinoff(); }));

    controller.a()
      .whileTrue(new AutoEngageCommand(robotDrive))
      .whileFalse(new InstantCommand( () -> {}));

    controller.y()
      .onTrue(new InstantCommand(() -> robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));

    /*
     * NOTE: Leave this as onTrue, the timeout
     * will interrupt the command so no while
     * false or while true is needed here
     */
    controller.b()
    .whileTrue(new TurnCommand(robotDrive, 180));
    //.whileFalse(new InstantCommand( () -> { robotArm.setArm(0); }));

    // Test Button
    controller.y()
    .whileTrue(new InstantCommand( () -> { robotDrive.setTankDriveVolts(12, 12);}))
    .whileFalse(new InstantCommand( () -> {}));

    //////////////////// BUTTON BOARD ////////////////////

    pidArmInit(ButtonBoardConstants.SCORE_HIGH_BUTTON, "high");
    pidArmInit(ButtonBoardConstants.SCORE_MID_BUTTON, "mid");
    pidArmInit(ButtonBoardConstants.SCORE_LOW_BUTTON, "low");
    pidArmInit(ButtonBoardConstants.PICKUP_GROUND_BUTTON, "ground");
    pidArmInit(ButtonBoardConstants.PICKUP_SUBSTATION_BUTTON, "substation");
    pidArmInit(ButtonBoardConstants.RETURN_TO_IDLE_BUTTON, "idle");

    armMoveInit(ButtonBoardConstants.ARM_UP_BUTTON, 1);
    armMoveInit(ButtonBoardConstants.ARM_DOWN_BUTTON, -1);

    buttonBoard.button(ButtonBoardConstants.TOGGLE_INTAKE_BUTTON)
      .whileTrue(new InstantCommand( () -> {
        if (GameStates.isCube) robotIntake.spinin();
        else robotIntake.spinout();
      }))
      .whileFalse(new InstantCommand( () -> { robotIntake.spinoff(); }));

    buttonBoard.button(ButtonBoardConstants.TOGGLE_OUTAKE_BUTTON)
      .whileTrue(new InstantCommand( () -> {
        if (GameStates.isCube) robotIntake.spinout(); 
        else robotIntake.spinin(); 
      }))
      .whileFalse(new InstantCommand( () -> { robotIntake.spinoff(); }));

    buttonBoard.button(ButtonBoardConstants.TOGGLE_CONE_MODE_BUTTON)
      .toggleOnTrue(new InstantCommand( () -> { 
        GameStates.isCube = false; 
        PDH.setSwitchableChannel(true);
      }))
      .toggleOnFalse(new InstantCommand( () -> { 
        GameStates.isCube = true;
        PDH.setSwitchableChannel(false);
      }));
    
    buttonBoard.button(ButtonBoardConstants.TOGGLE_SNIPER_MODE_BUTTON)
      .toggleOnTrue(new InstantCommand( () -> { SniperMode.armSniperMode = true; }))
      .toggleOnFalse(new InstantCommand( () -> { SniperMode.armSniperMode = false; }));
  }


  // #region CUSTOM ABSTRACTION FUNCTIONS
  
  // Simply abstracts PID positions arm must go to when button pressed
  private void pidArmInit(int btnPort, String desiredAngle) {
    chosenButton = buttonBoard.button(btnPort);

    chosenButton
    .whileTrue(new TeleopArmCommand(robotArm, desiredAngle))
    .whileFalse(new InstantCommand( () -> robotArm.setArm(0)));
  }

  // Moves arm motor based on spee/d on button press
  private void armMoveInit(int btnPort, int speedPar) {
    chosenButton = buttonBoard.button(btnPort);

    chosenButton
    .whileTrue(new InstantCommand( () -> { 
      DebugInfo.currentArmSpeed = speedPar; 
      robotArm.setArm(DebugInfo.currentArmSpeed); 
    }))
    .whileFalse(new InstantCommand( () -> { robotArm.setArm(0); }));
  }

  // endregion

  public DriveSubsystem getRobotDrive() {
    return robotDrive;
  }

  public Command getAutonomousCommand() {
    PathConstraints trajectoryConstraints = new PathConstraints(AutonomousConstants.DRIVE_VELOCITY, AutonomousConstants.MAX_ACCELERATION);
    PathPlannerTrajectory mainTrajectory = PathPlanner.loadPath("Taha" , trajectoryConstraints);
    robotDrive.getField().getObject("Taha").setTrajectory(mainTrajectory);

    return robotDrive.followPath(mainTrajectory);
  }
}
