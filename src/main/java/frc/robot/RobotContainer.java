// In Java We Trust

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.DynamicArmAngles;
import frc.robot.GlobalVars.GameStates;
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoEngageCommand;
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

    autonChooser.addOption("CONE MOBILITY EXTEND", 
      robotAuton.autonomousCmd(5));

    configureBindings();
  }

  private void configureBindings() {

    //////////////////// XBOX CONTROLLER //////////////////// 
    controller.leftTrigger()
      .whileTrue(new InstantCommand( () -> { SniperMode.driveSniperMode = true; }))
      .whileFalse(new InstantCommand( () -> { SniperMode.driveSniperMode = false; }));
    
    // Intake
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

    controller.x()
    .whileTrue(new InstantCommand( () -> { 
      DebugInfo.currentArmSpeed = -1; 
      robotArm.setArm(DebugInfo.currentArmSpeed); 
    }))
    .whileFalse(new InstantCommand( () -> { robotArm.setArm(0); }));

    controller.b()
    .whileTrue(new InstantCommand( () -> { 
      DebugInfo.currentArmSpeed = 1; 
      robotArm.setArm(DebugInfo.currentArmSpeed); 
    }))
    .whileFalse(new InstantCommand( () -> { robotArm.setArm(0); }));

    //Test Button
    controller.y()
    .whileTrue(new InstantCommand( () -> {
      GameStates.shouldRunPID = false;
      GameStates.armSetpoint = DynamicArmAngles.scoreHighAngle;
      GameStates.shouldRunPID = true;
    }))
    .whileFalse(new InstantCommand( () -> {
      holdPos();
      //  robotArm.setArm(0);
      }));

    //////////////////// BUTTON BOARD ////////////////////
    pidArmInit(ButtonBoardConstants.SCORE_HIGH_BUTTON, DynamicArmAngles.scoreHighAngle);
    pidArmInit(ButtonBoardConstants.SCORE_MID_BUTTON, DynamicArmAngles.scoreMidAngle);
    pidArmInit(ButtonBoardConstants.SCORE_LOW_BUTTON, DynamicArmAngles.scoreLowAngle);
    pidArmInit(ButtonBoardConstants.PICKUP_SUBSTATION_BUTTON, DynamicArmAngles.fetchSubstationAngle);
    pidArmInit(ButtonBoardConstants.PICKUP_GROUND_BUTTON, DynamicArmAngles.fetchGroundAngle);
    pidArmInit(ButtonBoardConstants.RETURN_TO_IDLE_BUTTON, ArmConstants.IDLE);

    armMoveInit(ButtonBoardConstants.ARM_UP_BUTTON, 1);
    armMoveInit(ButtonBoardConstants.ARM_DOWN_BUTTON, -1);

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
  
  private void holdPos(){
    GameStates.shouldRunPID = false;
    GameStates.armSetpoint = robotArm.getBicepEncoderPosition();
    GameStates.shouldRunPID = true;
  }

  // Simply abstracts PID positions arm must go to when button pressed
  private void pidArmInit(int btnPort, Double passedSetpointPar) {
    buttonBoard.button(btnPort)
    .whileTrue(
      new InstantCommand( () -> {
        GameStates.shouldRunPID = false;
        GameStates.armSetpoint = passedSetpointPar;
        GameStates.shouldRunPID = true;
      }))
    .whileFalse(new InstantCommand( () -> { 
      // robotArm.setArm(0);
      holdPos();
    }));
  }
  

  // Moves arm motor based on spee/d on button press
  private void armMoveInit(int btnPort, int speedPar) {
    buttonBoard.button(btnPort)
    .whileTrue(new InstantCommand( () -> { 
      GameStates.shouldRunPID = false;
      DebugInfo.currentArmSpeed = speedPar; 
      robotArm.setArm(DebugInfo.currentArmSpeed); 
    }))
    .whileFalse(new InstantCommand( () -> {
      //  robotArm.setArm(0); 
      holdPos();
    }));
  }
  // endregion

  public DriveSubsystem getRobotDrive() {
    return robotDrive;
  }

  public ArmSubsystem getRobotArm() {
    return robotArm;
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
