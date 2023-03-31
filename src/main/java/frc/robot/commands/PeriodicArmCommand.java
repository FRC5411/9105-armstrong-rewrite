
package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

    private double kS;
    private double kG;
    private double kV;
    private double kA;
    private ArmFeedforward feedforward;

    public PeriodicArmCommand(ArmSubsystem robotArm) {
        this.robotArm = robotArm;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      kP = 0.063;
      kI = 0;
      kD = 0;

      kS = 0.002;
      kG = 0.81;
      kV = 3.74;
      kA = 0.13;

      feedforward = new ArmFeedforward(kS, kG, kV, kA);
      pid = new PIDController(kP, kI, kD);
      pid.setTolerance(2);
      pid.enableContinuousInput(0, 360);

      System.out.println("Command PERIODIC ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
        double currentPosition = robotArm.getBicepEncoderPosition();

        currentPosition = currentPosition - (180 - Constants.ArmConstants.FLAT);

        if(currentPosition < 0 ){
          currentPosition += 360;
        }

      if (GameStates.shouldHoldArm) {
        double calc = pid.calculate(
          currentPosition, 
          GameStates.armSetpoint
        ) +   
          
          feedforward.calculate(Math.toRadians(GameStates.armSetpoint + Constants.ArmConstants.SETPOINT_OFFSET), 
          robotArm.getEncoderVelocity()
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