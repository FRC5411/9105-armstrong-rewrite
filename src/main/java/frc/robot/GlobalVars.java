
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

        public static boolean shouldHoldArm = false;
        public static double armSetpoint = 3;
    }
}
