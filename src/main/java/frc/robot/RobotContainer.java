// In Java We Trust

package frc.robot;
import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.DriverProfiles;
import frc.robot.GlobalVars.GameStates;
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.AutoEngageCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.Arm.TeleopArmCommand;
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
  private SendableChooser<Command> driverChooser;

  Trigger chosenButton;
  
  Trigger toggleIntakeButton;
  Trigger toggleOutakeButton;
  Trigger toggleConeModeButton;
  Trigger toggleSniperModeButton;
  
  public RobotContainer() {
    
    controller = new CommandXboxController(DrivebaseConstants.CONTROLLER_PORT);
    buttonBoard = new CommandGenericHID(ButtonBoardConstants.BUTTON_BOARD_PORT);

    toggleSniperModeButton = buttonBoard.button(ButtonBoardConstants.TOGGLE_SNIPER_MODE_BUTTON);
    toggleOutakeButton = buttonBoard.button(ButtonBoardConstants.TOGGLE_OUTAKE_BUTTON);
    toggleIntakeButton = buttonBoard.button(ButtonBoardConstants.TOGGLE_INTAKE_BUTTON);
    toggleConeModeButton = buttonBoard.button(ButtonBoardConstants.TOGGLE_CONE_MODE_BUTTON);

    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();
    robotAuton = new AutonSubsystem(robotDrive, robotArm, robotIntake);

    PDH = new PowerDistribution(DrivebaseConstants.PDH_PORT_CANID, ModuleType.kRev);

    autonChooser = new SendableChooser<>();
    driverChooser = new SendableChooser<>();

    robotDrive.setDefaultCommand(new ArcadeCommand(
      () -> controller.getLeftY(),
      () -> - controller.getRightX(),
      robotDrive
      ));

    Shuffleboard.getTab("Autonomous: ").add(autonChooser);
    autonChooser.addOption("CONE MOBILITY", robotAuton.autonomousCmd(1));
    autonChooser.addOption("CONE MOBILITY DOCK", robotAuton.autonomousCmd(2));
    autonChooser.addOption("CONE SCORE ONLY", robotAuton.autonomousCmd(3));
    autonChooser.addOption("CUBE SCORE ONLY", robotAuton.autonomousCmd(4));
    autonChooser.addOption("CUBE MOBILITY EXTEND GRAB", robotAuton.autonomousCmd(5));
    autonChooser.addOption("(Exp*) CONE MOBILITY TURN EXTEND", robotAuton.autonomousCmd(6));
    autonChooser.addOption("(Exp*) RED CONE MOBILITY TURN", robotAuton.autonomousCmd(7));

//    PathPlannerServer.startServer(5811);

    configureBindings();
    initialiseDriverProfiles();
  }

  private void stopAll(){
    GameStates.shouldHoldArm = false;
    new TeleopArmCommand(robotArm, "idle");

  }

  private void configureBindings() {

    //////////////////// XBOX CONTROLLER ////////////////////
    
    //////// Toggle Drive Sniper mode
    whileHeld(controller.leftTrigger(), () -> SniperMode.driveSniperMode = true);
    whileUnheld(controller.leftTrigger(), () -> SniperMode.driveSniperMode = false);


    //////// Outake Button for cone, intake for cube
    whileHeld(controller.leftBumper(), () -> {
      if (GameStates.isCube) { robotIntake.spinout(); }
      else { robotIntake.spinin(); }});
    whileUnheld(controller.leftBumper(), () -> robotIntake.spinoff());


    //////// Outake Button for cube, intake for cone
    whileHeld(controller.rightBumper(), () -> {
      if (GameStates.isCube) robotIntake.spinin();
      else robotIntake.spinout();});
    whileUnheld(controller.rightBumper(), () -> robotIntake.spinoff());


    //////// Autobalance Drive
    controller.a().whileTrue(new AutoEngageCommand(robotDrive));


    //////// Reset Odometer
    whenClicked(controller.y(), () -> robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))));
      
      
    //////// Run Turn Command
    controller.b().onTrue(new TurnCommand(
        new ProfiledPIDController(
          0.015, 0, 0, 
        new TrapezoidProfile.Constraints(100, 100)),
        () -> 180,
        () -> robotDrive.getPose().getRotation().getDegrees(),
        robotDrive
    ).withTimeout(1.5)).onFalse(new InstantCommand(() -> {}, robotDrive));

    whileHeld(controller.x(), () -> stopAll());

    //////////////////// BUTTON BOARD ////////////////////
    
    pidArmInit(ButtonBoardConstants.SCORE_HIGH_BUTTON, "high");
    pidArmInit(ButtonBoardConstants.SCORE_MID_BUTTON, "mid");
    pidArmInit(ButtonBoardConstants.SCORE_LOW_BUTTON, "low");
    pidArmInit(ButtonBoardConstants.PICKUP_GROUND_BUTTON, "ground");
    pidArmInit(ButtonBoardConstants.PICKUP_SUBSTATION_BUTTON, "substation");
    pidArmInit(ButtonBoardConstants.RETURN_TO_IDLE_BUTTON, "idle");

    armMoveInit(ButtonBoardConstants.ARM_UP_BUTTON, 1);
    armMoveInit(ButtonBoardConstants.ARM_DOWN_BUTTON, -1);


    //////// Outake Button for cube, intake for cone
    whileHeld(toggleIntakeButton, () -> {
      if (GameStates.isCube) robotIntake.spinin();
      else robotIntake.spinout();});
    whileUnheld(toggleIntakeButton, () -> robotIntake.spinoff());
    

    //////// Intake Button for cube, outake for cone
    whileHeld(toggleOutakeButton, () -> {
      if (GameStates.isCube) robotIntake.spinout(); 
      else robotIntake.spinin();});
    whileUnheld(toggleOutakeButton, () -> robotIntake.spinoff());


    //////// Toggle cone mode button
    whenClicked(toggleConeModeButton, () -> { 
      GameStates.isCube = false; 
      PDH.setSwitchableChannel(true);});

    whenUnClicked(toggleConeModeButton, () -> { 
      GameStates.isCube = true;
      PDH.setSwitchableChannel(false);});
    

    //////// Toggle Sniper Mode Button
    whileHeld(toggleSniperModeButton,  () -> SniperMode.armSniperMode = true);
    whileUnheld(toggleSniperModeButton, () -> SniperMode.armSniperMode = false);
      
    }

    //////////////////
    // DONT PASS COMMANDS INTO THESE. ONLY INSTANT COMMANDS
    private void whileHeld(Trigger chosenButton, Runnable codeToRun){
      chosenButton
        .whileTrue(new InstantCommand(()-> codeToRun.run()));
    }

    private void whileUnheld(Trigger chosenButton, Runnable codeToRun){
      chosenButton
        .whileFalse(new InstantCommand(()-> codeToRun.run()));
    }

    private void whenClicked(Trigger chosenButton, Runnable codeToRun){
      chosenButton
        .onTrue(new InstantCommand(()-> codeToRun.run()));
    }

   

    private void whenUnClicked(Trigger chosenButton, Runnable codeToRun){
      chosenButton
        .onFalse(new InstantCommand(()-> codeToRun.run()));
    }

    
    // #region CUSTOM ABSTRACTION FUNCTIONS
    
    // Simply abstracts PID positions arm must go to when button pressed
    private void pidArmInit(int btnPort, String desiredAngle) {
      chosenButton = buttonBoard.button(btnPort);   
      chosenButton.whileTrue(new TeleopArmCommand(robotArm, desiredAngle));
      whileUnheld(chosenButton, () -> {
        GameStates.shouldHoldArm = true;
        robotArm.setArm(0);
      });
  }
  
  // Moves arm motor based on spee/d on button press
  private void armMoveInit(int btnPort, int speedPar) {
    chosenButton = buttonBoard.button(btnPort);

    whileHeld(chosenButton, () -> { 
      GameStates.shouldHoldArm = false;
      DebugInfo.currentArmSpeed = speedPar; 
      robotArm.setArm(DebugInfo.currentArmSpeed); 
    });
    whileUnheld(chosenButton, () -> {
      GameStates.shouldHoldArm = true;
      robotArm.setArm(0);
    });
  }

  // endregion

  public DriveSubsystem getRobotDrive() {
    return robotDrive;
  }


  public ArmSubsystem getRobotArm() {
    return robotArm;
  }

  public void initialiseDriverProfiles() {
    Shuffleboard.getTab("Driver Profiles").add(driverChooser);
    driverChooser.addOption("Fatemah", new InstantCommand(() -> { 
      DriverProfiles.deadzoneValues = 0.1;
      DriverProfiles.squareInputs = false;
    }));
    driverChooser.addOption("Rithvik", new InstantCommand(() -> { 
      DriverProfiles.deadzoneValues = 0.0;
      DriverProfiles.squareInputs = true;
    }));
    driverChooser.addOption("Aaron", new InstantCommand(() -> { 
      DriverProfiles.deadzoneValues = 0.1;
      DriverProfiles.squareInputs = true;
    }));
  }

  public Command getAutonomousCommand(String trajectoryName, Boolean alliance) {
    PathConstraints trajectoryConstraints = new PathConstraints(AutonomousConstants.DRIVE_VELOCITY, AutonomousConstants.MAX_ACCELERATION);
    List<PathPlannerTrajectory> mainTrajectory = PathPlanner.loadPathGroup("straight" , trajectoryConstraints);
    PathPlannerTrajectory mapTrajectory = PathPlanner.loadPath("straight", trajectoryConstraints);
    robotDrive.getField().getObject("straight").setTrajectory(mapTrajectory);

    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("GoConeHigh", robotAuton.armConeHigh());
    eventMap.put("SpitCone", robotAuton.inCubeOutCone());

    eventMap.put("GoCubeLow", robotAuton.armCubeGround());
    eventMap.put("TakeCube", robotAuton.inCubeOutCone());

    eventMap.put("GoCubeHigh", robotAuton.armCubeHigh());
    eventMap.put("SpitCube", robotAuton.inConeOutCube());

    eventMap.put("Idle", robotAuton.armToIdle(1.9));

    eventMap.put("TurnTo0", robotDrive.turnTo0CMD());
    eventMap.put("TurnTo180", robotDrive.turnTo180CMD());

    return new SequentialCommandGroup(
        robotDrive.followPathGroup(mainTrajectory, alliance, eventMap), 
        robotDrive.turnTo180CMD().withTimeout(2),
        robotDrive.turnTo0CMD().withTimeout(2));
  }
}