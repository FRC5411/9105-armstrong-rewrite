
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
        public static double scoreHighAngle = 0.0;
        public static double scoreMidAngle = 0.0;
        public static double scoreLowAngle = 0.0;

        public static double fetchSubstationAngle = 0.0;
        public static double fetchGroundAngle = 0.0;
        public static double idle = 0.0;

        public static void checkGamePieceMode() {
            if (GameStates.isCube) {
                scoreHighAngle = 100;
            }
            else {
                scoreHighAngle = 170;
            }
        }
    }
}