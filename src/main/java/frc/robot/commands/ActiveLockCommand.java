
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class ActiveLockCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem robotArm;
    private double error;
 

    public ActiveLockCommand(ArmSubsystem robotArm) {
      this.robotArm = robotArm;
    }

    @Override
    public void initialize() {
      System.out.println("Command ACTIVELOCK has started");

    }

    @Override
    public void execute() {
      if(GameStates.shouldLock) {
        error = robotArm.getBicepEncoderPosition() - GameStates.pidArmAngle;
        robotArm.moveTicks((int) error);
      }
    }
    
    @Override
    public void end(boolean interrupted) {
      System.out.println("Command ACTIVELOCK has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}