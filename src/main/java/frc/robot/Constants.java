
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  static double convertToLinDist(double GEAR_RATIO, double WHEEL_RADIUS){
    return Units.inchesToMeters (
        1 / GEAR_RATIO
        * 2 * Math.PI 
        * 1
        * Units.inchesToMeters(WHEEL_RADIUS)
        )
        * (2.16/0.548);
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

    public static final double AUTO_ENGAGE_DRIVE_BOOST = 1.0; // does nothing but may be useful later

    /* PID Constants for Turning */

    public static final double P_DRIVE_TURN = 0.03;
    public static final double I_DRIVE_TURN = 0.0;
    public static final double D_DRIVE_TURN = 0.0005;
  }

  public static class AutonomousConstants {
    public static final double VOLTS = 0.14592; //kS
    public static final double VOLT_SECONDS_PER_METER = 2.0809; //kV
    public static final double VOLT_SECONDS_SQUARED_PER_METER = 0.83925; //kA
    public static final double DRIVE_VELOCITY = 0.1;//0.001//0.0001//0.014026; //kP
    
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(6);
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new 
      DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    
    public static final double MAX_SPEED_METERS_PER_SECOND = 3; 
    public static final double MAX_ACCELERATION = 1; 
    
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7; 
    
    public static final double GEAR_RATIO = 7.89;
    public static final double WHEEL_RADIUS = 3; 
    
    public static final double LINEAR_DIST_CONVERSION_FACTOR = (convertToLinDist(GEAR_RATIO, WHEEL_RADIUS));

    public static final double DRIVE_SPEED = 0.25;
  }

  public static class ArmConstants {
    public static final double ARM_REDUCED_SPEED = 0.6;
    public static final double ARM_SNIPER_SPEED = 0.3;

    public static final int ARM_MOTOR_CANID = 21;
    public static final int GRABBER_MOTOR_CANID = 22;

    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final int GRABBER_MOTOR_CURRENT_LIMIT = 50;

    public static final double CONE_HIGH_ANGLE = 172;
    public static final double CONE_MID_ANGLE = 193;
    public static final double CONE_LOW_ANGLE = 112; 
    public static final double CONE_SUBSTATION_ANGLE = 174;
    public static final double CONE_GROUND_ANGLE = 257;

    public static final double CUBE_HIGH_ANGLE = 173;
    public static final double CUBE_MID_ANGLE = 142;
    public static final double CUBE_LOW_ANGLE = 112; 
    public static final double CUBE_SUBSTATION_ANGLE = 178;
    public static final double CUBE_GROUND_ANGLE = 263;
    
    
    public static final double FRONT = 178; 
    public static final double STRAIGHT = 180;
    public static final double HOLD = 75; 
    public static final double IDLE = 0; 
    public static final double FLAT = 33.43;

    public static final double ARM_VELOCITY = 250;
    public static final double ARM_ACCELERATION = 100;
    //public static final double TUCKED = 59; 
    //public static final double FLAT = 29;

    //public static final double SETPOINT_OFFSET = -32; 
   }

  public static class ButtonBoardConstants {
    public static final int BUTTON_BOARD_PORT = 1;

    public static final int SCORE_HIGH_BUTTON = 1;
    public static final int SCORE_MID_BUTTON = 2;
    public static final int SCORE_LOW_BUTTON = 3;

    //public static final int TOGGLE_CUBE_MODE_BUTTON = 6;
    public static final int TOGGLE_CONE_MODE_BUTTON = 5;

    public static final int PICKUP_SUBSTATION_BUTTON = 8;
    public static final int PICKUP_GROUND_BUTTON = 7;

    public static final int TOGGLE_SNIPER_MODE_BUTTON = 4;

    public static final int RETURN_TO_IDLE_BUTTON = 9;

    public static final int ARM_UP_BUTTON = 10;
    public static final int ARM_DOWN_BUTTON = 11;

    public static final int TOGGLE_INTAKE_BUTTON = 12; 
    public static final int TOGGLE_OUTAKE_BUTTON = 6; 

  }
}