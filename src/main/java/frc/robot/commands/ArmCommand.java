
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;
    private double setpoint;
    private double passedSetpoint;
    private PIDController pid;

    private double kP;
    private double kI;
    private double kD;

    public ArmCommand(ArmSubsystem robotArm, double passedSetpoint) {
        this.robotArm = robotArm;
        this.passedSetpoint = passedSetpoint;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
        if (GameStates.isCube == true) {
            if (passedSetpoint == 1) this.setpoint = 100;
            else if (passedSetpoint == 2) this.setpoint = ArmConstants.MID_CUBE_ANG;
            else if (passedSetpoint == 3) this.setpoint = ArmConstants.LOW_CUBE_ANG;
            else if (passedSetpoint == 4) this.setpoint = ArmConstants.FETCH_CUBE_SUBSTATION;
            else if (passedSetpoint == 5) this.setpoint = ArmConstants.FETCH_CUBE_GROUND;
            else this.setpoint = 0;
        }
        else {
            if (passedSetpoint == 1) this.setpoint = 200;
            else if (passedSetpoint == 2) this.setpoint = ArmConstants.MID_CONE_ANG;
            else if (passedSetpoint == 3) this.setpoint = ArmConstants.LOW_CONE_ANG;
            else if (passedSetpoint == 4) this.setpoint = ArmConstants.FETCH_CONE_SUBSTATION;
            else if (passedSetpoint == 5) this.setpoint = ArmConstants.FETCH_CONE_GROUND;
            else this.setpoint = 0;
        }

      kP = 0.034;
      kI = 0;
      kD = 0;

      pid = new PIDController(kP, kI, kD);

      pid.setTolerance(1);

      System.out.println("Command ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
        double calc = pid.calculate(robotArm.getBicepEncoderPosition(), setpoint);
        
        robotArm.setArm(calc);

        DebugInfo.currentArmSpeed = calc;
        System.out.println("Setpoint:" + setpoint);
    }
  
    @Override
    public void end(boolean interrupted) {
      System.out.println("Command ARM ALIGN has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}