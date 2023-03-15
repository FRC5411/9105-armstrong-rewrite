// In Java We Trust

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoEngageCommand;
import frc.robot.commands.AutonCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  private CommandXboxController controller;
  private CommandGenericHID buttonBoard;

  private DriveSubsystem robotDrive;
  private ArmSubsystem robotArm;
  private IntakeSubsystem robotIntake;

  private PowerDistribution PDH;

  private SendableChooser<Command> autonChooser;

  public RobotContainer() {
    controller = new CommandXboxController(DrivebaseConstants.CONTROLLER_PORT);
    buttonBoard = new CommandGenericHID(ButtonBoardConstants.BUTTON_BOARD_PORT);

    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();

    PDH = new PowerDistribution(DrivebaseConstants.PDH_PORT_CANID, ModuleType.kRev);

    autonChooser = new SendableChooser<>();

    robotDrive.setDefaultCommand(new ArcadeCommand(
      () -> controller.getLeftY(),
      () -> controller.getRightX(),
      robotDrive
      ));

    autonChooser.addOption("PATH TOP", 
      new AutonCommand(robotDrive, robotArm, robotIntake, 1));

    autonChooser.addOption("PATH MID", 
      new AutonCommand(robotDrive, robotArm, robotIntake, 2));

    autonChooser.addOption("PATH BOTTOM", 
      new AutonCommand(robotDrive, robotArm, robotIntake, 3));

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
      .whileTrue(new InstantCommand( () -> { new AutoEngageCommand(robotDrive); }))
      .whileFalse(new InstantCommand( () -> { CommandScheduler.getInstance().cancel(new AutoEngageCommand(robotDrive)); }));

    //////////////////// BUTTON BOARD ////////////////////
    pidArmInit(ButtonBoardConstants.SCORE_HIGH_BUTTON, 1.0);
    pidArmInit(ButtonBoardConstants.SCORE_MID_BUTTON, 2.0);
    pidArmInit(ButtonBoardConstants.SCORE_LOW_BUTTON, 3.0);
    pidArmInit(ButtonBoardConstants.PICKUP_SUBSTATION_BUTTON, 4.0);
    pidArmInit(ButtonBoardConstants.PICKUP_GROUND_BUTTON, 5.0);
    pidArmInit(ButtonBoardConstants.RETURN_TO_IDLE_BUTTON, ArmConstants.IDLE);

    armMoveInit(ButtonBoardConstants.ARM_UP_BUTTON, 1);
    armMoveInit(ButtonBoardConstants.ARM_DOWN_BUTTON, -1);

    buttonBoard.button(ButtonBoardConstants.TOGGLE_CUBE_MODE_BUTTON)
      .toggleOnTrue(new InstantCommand( () -> { 
        GameStates.isCube = true;
        PDH.setSwitchableChannel(false);
      }))
      .toggleOnFalse(new InstantCommand( () -> { 
        GameStates.isCube = false; 
        PDH.setSwitchableChannel(true);
      }));

    buttonBoard.button(ButtonBoardConstants.TOGGLE_SNIPER_MODE_BUTTON)
      .toggleOnTrue(new InstantCommand( () -> { SniperMode.armSniperMode = true; }))
      .toggleOnFalse(new InstantCommand( () -> { SniperMode.armSniperMode = false; }));
  }

  // Custom Functions for Abstraction
  // #region CUSTOM ABSTRACTION FUNCTIONS
  // Simply abstracts PID positions arm must go to when button pressed
  private void pidArmInit(int btnPort, Double passedSetpointPar) {
    buttonBoard.button(btnPort)
    .whileTrue(new ArmCommand(robotArm, passedSetpointPar))
    .whileFalse(new InstantCommand( () -> { robotArm.setArm(0); }));
  }

  // Moves arm motor based on speed on button press
  private void armMoveInit(int btnPort, int speedPar) {
    buttonBoard.button(btnPort)
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
    return autonChooser.getSelected();
  }
}
