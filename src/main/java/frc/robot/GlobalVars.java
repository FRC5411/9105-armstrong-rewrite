
package frc.robot;

public class GlobalVars {

    public static class SniperMode {
        public static boolean driveSniperMode = false;
        public static boolean armSniperMode = false;
    }

    public static class DebugInfo {
        public static double currentArmSpeed = 0;
        public static double initialGyroPitch = 0;

        public static double profiledArmP = 0;
        public static double profiledArmI = 0;
        public static double profiledArmD = 0;
    }

    public static class GameStates {
        public static boolean isCube = true;
        public static int chosenAuton = 0;

        public static boolean shouldHoldArm = false;

        // Current angles to go to, get updated (this is for dynamic setpoints)
        // public static double curHighAng = 0;
        // public static double curMiddleAng = 0;
        // public static double curLowAng = 0;
        // public static double curSubstationAng = 0;
        // public static double curGroundAng = 0;
    }

    public static class DriverProfiles {
        public static boolean squareInputs = false;
        public static double deadzoneValues = 0.0;
    }
}
