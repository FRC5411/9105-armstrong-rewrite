package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private IntakeSubsystem robotIntake;

    private double spin;

    public IntakeCommand(IntakeSubsystem robotIntake, double spin) {
        this.robotIntake = robotIntake;
        this.spin = spin;
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        robotIntake.setspin(spin);
    }
  
    @Override
    public void end(boolean interrupted) {
        robotIntake.setspin(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
    
}
