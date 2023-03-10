
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
    }

    public static class DynamicArmAngles {
        public static double scoreHighAngle = 1;
        public static double scoreMidAngle = 2;
        public static double scoreLowAngle = 3;

        public static double fetchSubstationAngle = 4;
        public static double fetchGroundAngle = 5;
    }
}