
package frc.robot;

public class GlobalVars {

    public static class SniperMode {
        public static boolean driveSniperMode = false;
        public static boolean armSniperMode = false;
    }

    public static class DebugInfo {
        public static double currentArmSpeed = 0;
    }

    public static class GameStates {
        public static boolean isCube = true;
        public static int chosenAuton = 0;

        public static double armSetpoint = 3;
    }

    public static class DynamicArmAngles {
        public static double scoreHighAngle = 167;
        public static double scoreMidAngle = 187;
        public static double scoreLowAngle = 110;

        public static double fetchSubstationAngle = 167;
        public static double fetchGroundAngle = 251;

        public static double gyroP = 0;
    }
}
