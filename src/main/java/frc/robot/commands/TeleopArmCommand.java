
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVars.GameStates;
// import frc.robot.GlobalVars.GameStates;
// import frc.robot.GlobalVars.DebugInfo;
// import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class TeleopArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;
    private ProfiledPIDController pid;
    private String strSetpoint;

    private double kP;
    private double kI; 
    private double kD;

    public TeleopArmCommand(ArmSubsystem robotArm, String strSetpoint) {
        this.robotArm = robotArm;
        this.strSetpoint = strSetpoint;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      kP = 0.066;
      kI = 0;
      kD = 0;

      pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(ArmConstants.ARM_VELOCITY, ArmConstants.ARM_ACCELERATION));
      pid.setTolerance(2);
      pid.reset(robotArm.getBicepEncoderPosition());

      System.out.println("Command PERIODIC ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
      double calc = pid.calculate(robotArm.getBicepEncoderPosition(), returnAngle(strSetpoint));
      robotArm.setManualArm(calc);
    }
  
    @Override
    public void end(boolean interrupted) {
      robotArm.setArm(0);
      System.out.println("Command PERIODIC ARM ALIGN has ended");
    }

  private double returnAngle(String pos){
    switch(pos){
      case "high":
        if (GameStates.isCube) return ArmConstants.CUBE_HIGH_ANGLE;
        else return ArmConstants.CONE_HIGH_ANGLE;
        
      case "mid":
        if (GameStates.isCube) return ArmConstants.CUBE_MID_ANGLE;
        else return ArmConstants.CONE_MID_ANGLE;
        
      case "low":
        if (GameStates.isCube) return ArmConstants.CUBE_LOW_ANGLE;
        else return ArmConstants.CONE_LOW_ANGLE;
        
      case "ground":
        if (GameStates.isCube) return ArmConstants.CUBE_GROUND_ANGLE;
        else return ArmConstants.CONE_GROUND_ANGLE;
        
      case "substation":
        if (GameStates.isCube) return ArmConstants.CUBE_SUBSTATION_ANGLE;
        else return ArmConstants.CONE_SUBSTATION_ANGLE;

      case "idle":
        return ArmConstants.IDLE;
        
      default:
        System.out.println("CODE ERROR! INVALID POSITION! CHECK ROBOTCONTAINER!");
        return 0;
    }
  }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}