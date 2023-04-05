
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class Men extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;
    private ProfiledPIDController pid;

    private double kP;
    private double kI;
    private double kD;

    public Men(ArmSubsystem robotArm) {
        this.robotArm = robotArm;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      kP = 0.066;
      kI = 0;
      kD = 0;

      pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(1, 1));
      pid.setTolerance(2);
      pid.reset(robotArm.getBicepEncoderPosition());

      System.out.println("Command PERIODIC ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
      double calc = pid.calculate(robotArm.getBicepEncoderPosition(), 100);

      robotArm.setArm(calc);
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