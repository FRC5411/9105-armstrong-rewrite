
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.GlobalVars.GameStates;

public class IntakeSubsystem extends SubsystemBase{
  private CANSparkMax grabber;
  private LinearFilter filter;

  public IntakeSubsystem() {
    grabber = new CANSparkMax(
      ArmConstants.GRABBER_MOTOR_CANID, 
      MotorType.kBrushless
    );
  }

  public void setspin(double speed) {
    grabber.set(speed);
  }

  public void spinin() {
    if (GameStates.isCube) {
      grabber.set(0.5);
    }
    else {
      grabber.set(1);
    }
  }

  public void spinout() {
    if (GameStates.isCube) {
      grabber.set(-0.5);
    }
    else {
      grabber.set(-1);
    }
  }

  public void spinoff() {
    grabber.set(0);
  }

  public double getIntakeCurrent() {
    return grabber.getOutputCurrent();
  }

  public Command setIntakeSpin(double speed, IntakeSubsystem robotIntake) {
    return new InstantCommand( () -> robotIntake.setspin(speed) );
  } 

  @Override  
  public void periodic() {
      SmartDashboard.putNumber("INTAKE CURRENT: ", getIntakeCurrent());

      filter = LinearFilter.movingAverage(25);
      double calc = filter.calculate(getIntakeCurrent());

      if (calc > ArmConstants.GRABBER_MOTOR_CURRENT_LIMIT) {
        spinoff();
      }
  }
}