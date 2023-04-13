package frc.robot.Libs;

public class SProfile {
    public double maxVel;
    public double maxAccel;
    public double maxJerk;
    public double goal;
    public double currentPos;
    public double t_half;
    public double dist_half;
    public double t_end;
    public double t_max;
    public double dist_max;
    public double t_total;


    public SProfile(double maxVel, double maxAccel, double maxJerk) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    public void setGoal(double currentPos, double goal) {
        this.goal = goal;
        this.currentPos = currentPos;
        congfigs();
    }

    public void  congfigs() {
        double t_half = Math.sqrt((goal - currentPos) / (2 * maxAccel));
        double dist_half = 0.5 * maxAccel * t_half * t_half;
        double t_end = Math.sqrt((goal - currentPos - 2 * dist_half) / (2 * maxAccel));
        double t_max = (goal - currentPos - 2 * dist_half) / maxVel;
        double dist_max = maxVel * t_max;
        double t_total = 2 * t_half + t_end + t_max;
        this.t_half = t_half;
        this.dist_half = dist_half;
        this.t_end = t_end;
        this.t_max = t_max;
        this.dist_max = dist_max;
        this.t_total = t_total;
    }

    public double calculate(double t) {
        return currentPos +
        (goal - currentPos) * (10 * Math.pow(t / t_total, 3))
        - (15 * Math.pow(t / t_total, 4))
        + (6 * Math.pow(t / t_total, 5));
    }

    public double getTotalTime() {
        return t_total;
    }
}