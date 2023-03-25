
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;
    private double setpoint;
    private PIDController pid;

    private double kP;
    private double kI;
    private double kD;

    public ArmCommand(ArmSubsystem robotArm, double setpoint) {
        this.robotArm = robotArm;
        this.setpoint = setpoint;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      kP = 0.065;
      kI = 0;
      kD = 0;

      pid = new PIDController(kP, kI, kD);
      pid.setTolerance(1);

      System.out.println("Command ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
        double calc = pid.calculate(robotArm.getBicepEncoderPosition(), setpoint);
        //calc *= ArmConstants.ARM_REDUCED_SPEED;
        
        robotArm.setArm(calc);

        DebugInfo.currentArmSpeed = calc;
    }
  
    @Override
    public void end(boolean interrupted) {
      robotArm.setArm(0);
      System.out.println("Command ARM ALIGN has ended");
    }
  
    @Override
    public boolean isFinished() {
      return pid.atSetpoint();
    }
}