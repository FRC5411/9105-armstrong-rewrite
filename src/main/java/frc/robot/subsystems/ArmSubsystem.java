
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.DynamicArmAngles;
import frc.robot.GlobalVars.GameStates;
import frc.robot.GlobalVars.SniperMode;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax biscep;
    private Encoder armBoreEncoder;

    public ArmSubsystem() {
      biscep = new CANSparkMax(
        ArmConstants.ARM_MOTOR_CANID, 
        MotorType.kBrushless); 
      
      biscep.setIdleMode(IdleMode.kBrake);

      biscep.setInverted(false);

      biscep.setSmartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);

      armBoreEncoder = new Encoder(0, 1);
    }

    public void setArm(double speed) {
      if (SniperMode.armSniperMode) {
        speed *= ArmConstants.ARM_SNIPER_SPEED;
      }
      biscep.set(speed);
    }

    public double getBiscepEncoderPosition() {
      return armBoreEncoder.getDistance() / 22.755;
    }

    public double getArmCurrent() {
      return biscep.getOutputCurrent();
    }

    public void limitArmSpeedOut() {
      if (getBiscepEncoderPosition() > 277) {
        if (DebugInfo.currentArmSpeed > 0) {
          setArm(0);
        }
      }
    }

    public void limitArmSpeedDown() {
      if (getBiscepEncoderPosition() < 3) {
        if (DebugInfo.currentArmSpeed < 0) {
          setArm(0);
        }
      }
    }

    @Override  
    public void periodic() {
      limitArmSpeedOut();
      limitArmSpeedDown();
      
      SmartDashboard.putBoolean("GAME MODE", GameStates.isCube);
      SmartDashboard.putNumber("DEBUG INFO", DynamicArmAngles.scoreHighAngle);
    }
   
    @Override  public void simulationPeriodic() {}
}