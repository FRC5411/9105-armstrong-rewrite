package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class HoldArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;

    private ArmFeedforward FF;
    private double ks;
    private double kg;
    private double kv;
    private double ka;

    public HoldArmCommand(ArmSubsystem robotArm) {
        this.robotArm = robotArm;
    }

    @Override
    public void initialize() {
      ks = 0;
      kg = 0.03;
      kv = 0;
      ka = 0;

      FF = new ArmFeedforward(ks, kg, kv, ka);
      System.out.println("Command HOLD ARM COMMAND has started");
    }
  
    @Override
    public void execute() {
      if(GameStates.shouldHoldArm){
        double calc = FF.calculate(
          Math.toRadians(robotArm.getBicepEncoderPosition() - ArmConstants.FLAT), 
          Math.toRadians(robotArm.neoVelocity()) / 12
        );
  
        robotArm.setManualArm(calc);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      robotArm.setArm(0);
      System.out.println("Command HOLD ARM COMMAND has ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}