
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class PeriodicArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;
    private PIDController pid;

    private double kP;
    private double kI;
    private double kD;

    public PeriodicArmCommand(ArmSubsystem robotArm) {
        this.robotArm = robotArm;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      kP = 0.066;
      kI = 0;
      kD = 0;

      pid = new PIDController(kP, kI, kD);
      pid.setTolerance(2);

      System.out.println("Command PERIODIC ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
      if (GameStates.shouldHoldArm) {
        double calc = pid.calculate (
          robotArm.getBicepEncoderPosition(),
          GameStates.armSetpoint
        ); 

        robotArm.setArm(calc);
        DebugInfo.currentArmSpeed = calc;
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      robotArm.setArm(0);
      System.out.println("Command PERIODIC ARM ALIGN has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}