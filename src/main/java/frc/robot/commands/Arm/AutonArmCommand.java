package frc.robot.commands.Arm;

// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
// import frc.robot.GlobalVars.GameStates;
// import frc.robot.GlobalVars.DebugInfo;
// import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class AutonArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;
    private ProfiledPIDController pid;
    // private ArmFeedforward FF;
    private String setpoint;
    private String cargoType;
    

    private double kP;
    private double kI; 
    private double kD;

    public AutonArmCommand(ArmSubsystem robotArm, String setpoint, String cargoType) {
        this.robotArm = robotArm;
        this.setpoint = setpoint;
        this.cargoType = cargoType;
        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      kP = 0.066;
      kI = 0;
      kD = 0;

      // FF = new ArmFeedforward(0.04, kP, kI, kD);
      pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(ArmConstants.ARM_VELOCITY, ArmConstants.ARM_ACCELERATION));
      pid.setTolerance(2);
      pid.reset(robotArm.getBicepEncoderPosition());

      System.out.println("Command AUTON ARM ALIGN has started");
    }
  
    @Override
    public void execute() {
      double calc = pid.calculate(robotArm.getBicepEncoderPosition(), returnAngle(setpoint)); 
      // + FF.calculate(Math.toRadians(robotArm.getBicepEncoderPosition()), Math.toRadians(robotArm.getEncoderVelocity())/12);
      robotArm.setManualArm(calc);
    }
  

    
    @Override
    public void end(boolean interrupted) {
      robotArm.setArm(0);
      System.out.println("Command AUTON ARM ALIGN has ended");
    }

    private double returnAngle(String pos){
      switch(pos){
        case "high":
          if (cargoType == "cube") return ArmConstants.CUBE_HIGH_ANGLE;
          else if (cargoType == "cone") return ArmConstants.CONE_HIGH_ANGLE;
          System.out.println("CODE ERROR, CHECK AUTON ARM COMMAND");
          return 0;
        case "mid":
          if (cargoType == "cube") return ArmConstants.CUBE_MID_ANGLE;
          else if (cargoType == "cone") return ArmConstants.CONE_MID_ANGLE;
          System.out.println("CODE ERROR, CHECK AUTON ARM COMMAND");
          return 0;
        case "low":
          if (cargoType == "cube") return ArmConstants.CUBE_LOW_ANGLE;
          else if (cargoType == "cone") return ArmConstants.CONE_LOW_ANGLE;
          System.out.println("CODE ERROR, CHECK AUTON ARM COMMAND");
          return 0;
        case "ground":
          if (cargoType == "cube") return ArmConstants.CUBE_GROUND_ANGLE;
          else if (cargoType == "cone") return ArmConstants.CONE_GROUND_ANGLE;
          System.out.println("CODE ERROR, CHECK AUTON ARM COMMAND");
          return 0;
        case "substation":
          if (cargoType == "cube") return ArmConstants.CUBE_SUBSTATION_ANGLE;
          else if (cargoType == "cone") return ArmConstants.CONE_SUBSTATION_ANGLE;
          System.out.println("CODE ERROR, CHECK AUTON ARM COMMAND");
          return 0;
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