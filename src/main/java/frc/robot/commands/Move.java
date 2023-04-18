// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Move extends CommandBase {
  ProfiledPIDController controller;
  double setpoint;
  DoubleSupplier measure;
  DriveSubsystem subsystem;

  public Move(ProfiledPIDController controller, double setpoint, DoubleSupplier measure, DriveSubsystem subsystem) {
    this.controller = controller;
    this.setpoint = setpoint;
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.arcadeDrive(controller.calculate(measure.getAsDouble(), setpoint), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
