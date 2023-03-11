
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

    private CANSparkMax bicep;
    private Encoder armBoreEncoder;

    public ArmSubsystem() {
      bicep = new CANSparkMax(
        ArmConstants.ARM_MOTOR_CANID, 
        MotorType.kBrushless); 
      
      bicep.setIdleMode(IdleMode.kBrake);

      bicep.setInverted(false);

      bicep.setSmartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);

      armBoreEncoder = new Encoder(0, 1);
    }

    public void setArm(double speed) {
      if (SniperMode.armSniperMode) speed *= ArmConstants.ARM_SNIPER_SPEED;
      bicep.set(speed);
    }

    public double getBicepEncoderPosition() {
      return armBoreEncoder.getDistance() / 22.755;
    }

    public double getArmCurrent() {
      return bicep.getOutputCurrent();
    }
    
    public void limitArmSpeed() {
      double bicepEncoderPos = getBicepEncoderPosition(); // I put it in var so it doesn't have to fetch twice (helps with runtime)
      if (
        (bicepEncoderPos > 277 && DebugInfo.currentArmSpeed > 0) || 
        (bicepEncoderPos < 3 && DebugInfo.currentArmSpeed < 0)
      ) setArm(0);
    }

    @Override  
    public void periodic() {
      limitArmSpeed();
      
      SmartDashboard.putBoolean("GAME MODE", GameStates.isCube);
      SmartDashboard.putNumber("DEBUG INFO", DynamicArmAngles.scoreHighAngle);
    }
   
    @Override  public void simulationPeriodic() {}
}