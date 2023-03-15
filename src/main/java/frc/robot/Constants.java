
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  static double convertToLinDist(double GEAR_RATIO, double WHEEL_RADIUS){
    return Units.inchesToMeters (
        1 / GEAR_RATIO
        * 2 * Math.PI 
        * Units.inchesToMeters(WHEEL_RADIUS)
        * 10
        );
  }

  public static class DrivebaseConstants {
    public static final int LF_MOTOR_CANID = 11;
    public static final int LB_MOTOR_CANID = 12;
    public static final int RF_MOTOR_CANID = 13;
    public static final int RB_MOTOR_CANID = 14;

    public static final int PDH_PORT_CANID = 1;

    public static final int CONTROLLER_PORT = 0;

    public static final int MOTOR_AMP_LIMIT = 80;
    public static final double DRIVE_SNIPER_SPEED = 0.4;
    public static final double DEADZONE = 0.1;
    public static final double SPEED_REDUCTION = 0.95;
    public static final double ROTATION_REDUCTION = 0.6;
  }
  
  public static class LEDsConstants {
    public static final int LED_PORT = 0;
    public static final int LED_NUMBER = 60;
  }

  public static class AutonomousConstants {
    public static final double VOLTS = 0.12547;
    public static final double VOLT_SECONDS_PER_METER = 2.0508;
    public static final double VOLT_SECONDS_SQUARED_PER_METER = 0.57404;
    public static final double DRIVE_VELOCITY = 1.1216;
    
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(6);
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new 
      DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    
    public static final double MAX_SPEED_METERS_PER_SECOND = 3; 
    public static final double MAX_ACCELERATION = 3; 
    
    public static final double RAMSETE_B = 2; 
    public static final double RAMSETE_ZETA = 0.7; 
    
    public static final double GEAR_RATIO = 7.9;
    public static final double WHEEL_RADIUS = 3; 
    
    public static final double LINEAR_DIST_CONVERSION_FACTOR = 
      (convertToLinDist(GEAR_RATIO, WHEEL_RADIUS));

    public static final double DRIVE_SPEED = 0.5;
  }

  public static class ArmConstants { 
    public static final double ARM_SNIPER_SPEED = 0.3;

    public static final int ARM_MOTOR_CANID = 21;
    public static final int GRABBER_MOTOR_CANID = 22;

    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final int GRABBER_MOTOR_CURRENT_LIMIT = 50;

    public static final double LOW_CUBE_ANG = 280; 
    public static final double MID_CUBE_ANG = 220; 
    public static final double HIGH_CUBE_ANG = 175; 

    public static final double LOW_CONE_ANG = 280; 
    public static final double MID_CONE_ANG = 200; 
    public static final double HIGH_CONE_ANG = 175; 

    public static double FETCH_CONE_SUBSTATION = 116.3;
    public static double FETCH_CUBE_SUBSTATION = 125.4;

    public static final double FETCH_CONE_GROUND = 264; 
    public static final double FETCH_CUBE_GROUND = 265; 

    public static final double FRONT = 178; 
    public static final double STRAIGHT = 180;
    public static final double HOLD = 75; 
    public static final double IDLE = 3; 
   }

  public static class ButtonBoardConstants {
    public static final int BUTTON_BOARD_PORT = 1;

    public static final int SCORE_HIGH_BUTTON = 1;
    public static final int SCORE_MID_BUTTON = 2;
    public static final int SCORE_LOW_BUTTON = 3;

    public static final int TOGGLE_CUBE_MODE_BUTTON = 6;
    public static final int TOGGLE_CONE_MODE_BUTTON = 5;

    public static final int PICKUP_SUBSTATION_BUTTON = 8;
    public static final int PICKUP_GROUND_BUTTON = 7;

    public static final int TOGGLE_SNIPER_MODE_BUTTON = 4;

    public static final int RETURN_TO_IDLE_BUTTON = 9;

    public static final int ARM_UP_BUTTON = 10;
    public static final int ARM_DOWN_BUTTON = 11;

  }
}