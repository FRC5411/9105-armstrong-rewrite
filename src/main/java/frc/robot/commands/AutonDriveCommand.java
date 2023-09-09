// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonDriveCommand extends CommandBase {
  /** Creates a new AutonDriveCommand. */
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DoubleSupplier speed;
  private DoubleSupplier rotation;
  private DriveSubsystem robotDrive;

  public AutonDriveCommand(DoubleSupplier speed, DoubleSupplier rotation, DriveSubsystem robotDrive) {
    this.speed = speed;
    this.rotation = rotation;
    this.robotDrive = robotDrive;
    addRequirements(robotDrive);
  }

  @Override
  public void initialize() {
    System.out.println("Command ARCADE has started");
  }

  @Override
  public void execute() {
    robotDrive.autonomousArcadeDriveFix(speed.getAsDouble(), rotation.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Command ARCADE has ended");
  }

  @Override
  public boolean isFinished() {
    
    return false;
  }
}
