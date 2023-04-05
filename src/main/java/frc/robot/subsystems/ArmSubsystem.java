
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVars.DebugInfo;
import frc.robot.GlobalVars.GameStates;
import frc.robot.GlobalVars.SniperMode;
import frc.robot.commands.PeriodicArmCommand;

public class ArmSubsystem extends SubsystemBase {

  private PeriodicArmCommand periodicArmCommand;
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

      periodicArmCommand = new PeriodicArmCommand(this);
    }

    public void setArm(double speed) {
      if (SniperMode.armSniperMode) { speed *= ArmConstants.ARM_SNIPER_SPEED; }
      bicep.set(speed * ArmConstants.ARM_REDUCED_SPEED);
    }

    /*
     * NOTE: This method is to be used carefully 
     * since there is no speed reduction that
     * will slow the arm down. Expect arm to go
     * brr
     */
    public void setManualArm(double speed) {
      bicep.set(speed);
    }

    public double getBicepEncoderPosition() {
      return armBoreEncoder.getDistance() / 22.755;
    }

    public double getArmCurrent() {
      return bicep.getOutputCurrent();
    }

    public PeriodicArmCommand getPeriodicArmCommand() {
      return periodicArmCommand;
    }
    
    public void limitArmSpeed() {
      double bicepEncoderPos = getBicepEncoderPosition();
      if (
        (bicepEncoderPos > 277 && DebugInfo.currentArmSpeed > 0) || 
        (bicepEncoderPos < 3 && DebugInfo.currentArmSpeed < 0)
      ) { setArm(0); }
    }

    @Override  
    public void periodic() {
      CommandScheduler.getInstance().schedule(periodicArmCommand);

      limitArmSpeed();
      
      SmartDashboard.putBoolean("GAME MODE", GameStates.isCube);
      SmartDashboard.putNumber("ARM ENCODER", getBicepEncoderPosition());
    }
   
    @Override  public void simulationPeriodic() {}

    public double getEncoderVelocity() {
        return 0;
    }
}