// In Java We Trust

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutonoumousConstants;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  private CommandXboxController controller;
  private CommandGenericHID buttonBoard;

  private DriveSubsystem robotDrive;
  private ArmSubsystem robotArm;
  private IntakeSubsystem robotIntake;

  SendableChooser <Command> autonChooser;

  public RobotContainer() {
    controller = new CommandXboxController(DrivebaseConstants.CONTROLLER_PORT);
    buttonBoard = new CommandGenericHID(ButtonBoardConstants.BUTTON_BOARD_PORT);

    robotDrive = new DriveSubsystem();
    robotArm = new ArmSubsystem();
    robotIntake = new IntakeSubsystem();

    robotDrive.setDefaultCommand(new ArcadeCommand(
      () -> controller.getLeftY(),
      () -> controller.getRightX(),
      robotDrive
      ));

    autonChooser = new SendableChooser<>();
    PathConstraints trajectoryConstraints = new PathConstraints(AutonoumousConstants.DRIVE_VELOCITY, AutonoumousConstants.MAX_ACCELERATION);
    PathPlannerTrajectory mainTrajectory = PathPlanner.loadPath("/Users/k2so/Desktop/9105-armstrong-rewrite/src/main/deploy/pathplanner/generatedJSON/Test Path.wpilib.json" , trajectoryConstraints);

    autonChooser.addOption("Test Path", robotDrive.followPath(
      mainTrajectory,
     true));

    Shuffleboard.getTab("Autonomous: ").add(autonChooser);
    configureBindings();
  }

  private void configureBindings() {
    controller.leftTrigger()
      .whileTrue(new InstantCommand( () ->  SniperMode.driveSniperMode = true ))
      .whileFalse(new InstantCommand( () -> SniperMode.driveSniperMode = false ));

    controller.leftBumper()
      .whileTrue(new InstantCommand( () -> {
        if (GameStates.isCube) {
          robotIntake.spinout();
        }
        else {
          robotIntake.spinin();
        }
      }))
      .whileFalse(new InstantCommand( () -> robotIntake.spinoff() ));

    controller.rightBumper()
      .whileTrue(new InstantCommand( () -> {
        if (GameStates.isCube) {
          robotIntake.spinin();
        }
        else {
          robotIntake.spinout();
        }
      }))
      .whileFalse(new InstantCommand( () -> robotIntake.spinoff() ));

    buttonBoard.button(ButtonBoardConstants.SCORE_HIGH_BUTTON)
      .whileTrue(new ArmCommand(robotArm, 1))
      .whileFalse(new InstantCommand( () -> robotArm.setArm(0) ));

    buttonBoard.button(ButtonBoardConstants.SCORE_MID_BUTTON)
      .whileTrue(new ArmCommand(robotArm, 2))
      .whileFalse(new InstantCommand( () -> robotArm.setArm(0) ));

    buttonBoard.button(ButtonBoardConstants.SCORE_LOW_BUTTON)
      .whileTrue(new ArmCommand(robotArm, 3))
      .whileFalse(new InstantCommand( () -> robotArm.setArm(0) ));
      
    buttonBoard.button(ButtonBoardConstants.PICKUP_SUBSTATION_BUTTON)
      .whileTrue(new ArmCommand(robotArm, 4))
      .whileFalse(new InstantCommand( () -> robotArm.setArm(0) ));
    
    buttonBoard.button(ButtonBoardConstants.PICKUP_GROUND_BUTTON)
      .whileTrue(new ArmCommand(robotArm, 5))
      .whileFalse(new InstantCommand( () -> robotArm.setArm(0) ));

    buttonBoard.button(ButtonBoardConstants.RETURN_TO_IDLE_BUTTON)
      .whileTrue(new ArmCommand(robotArm, ArmConstants.IDLE))
      .whileFalse(new InstantCommand( () -> robotArm.setArm(0) ));

    buttonBoard.button(ButtonBoardConstants.ARM_UP_BUTTON)
      .whileTrue(new InstantCommand( () -> { 
        robotArm.setArm(1); 
        DebugInfo.currentArmSpeed = 1; 
      }))
      .whileFalse(new InstantCommand( () -> {
        robotArm.setArm(0);
      }));

    buttonBoard.button(ButtonBoardConstants.ARM_DOWN_BUTTON)
      .whileTrue(new InstantCommand( () -> { 
        robotArm.setArm(-1); 
        DebugInfo.currentArmSpeed = -1; 
      }))
      .whileFalse(new InstantCommand( () -> {
        robotArm.setArm(0);
      }));

    buttonBoard.button(ButtonBoardConstants.TOGGLE_CUBE_MODE_BUTTON)
      .toggleOnTrue(new InstantCommand( () -> {
        GameStates.isCube = true;
      }))
      .toggleOnFalse(new InstantCommand( () -> {
        GameStates.isCube = false;
      }));

    buttonBoard.button(ButtonBoardConstants.TOGGLE_SNIPER_MODE_BUTTON)
      .toggleOnTrue(new InstantCommand( () -> SniperMode.armSniperMode = true ))
      .toggleOnFalse(new InstantCommand( () -> SniperMode.armSniperMode = false) );
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
