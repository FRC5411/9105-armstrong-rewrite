
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.subsystems.ArmSubsystem;

public class ProfiledArmCommand extends ProfiledPIDCommand {
  
    public ProfiledArmCommand(ArmSubsystem robotArm, double goal) {
      super(
          new ProfiledPIDController(
              DebugInfo.profiledArmP,//SmartDashboard.getNumber("ARM P", 0.034),
              DebugInfo.profiledArmI,//SmartDashboard.getNumber("ARM I", 0.0),
              DebugInfo.profiledArmD,//SmartDashboard.getNumber("ARM D", 0.0),
              new TrapezoidProfile.Constraints(1, 0.5)),
          robotArm::getBicepEncoderPosition,
          200,
          (output, setpoint) -> {
            SmartDashboard.putNumber("ARM OUTPUT", output);

            robotArm.setArm(output);
        });
      GameStates.shouldHoldArm = false;
      addRequirements(robotArm);
  
      getController().setTolerance(1);
    }
  
    public void initialize() {

    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
  
    public void end() {}
  }
