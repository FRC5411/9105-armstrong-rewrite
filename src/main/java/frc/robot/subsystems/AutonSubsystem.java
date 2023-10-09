
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVars.GameStates;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.AutoEngageCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Arm.AutonArmCommand;

public class AutonSubsystem {

    public DriveSubsystem robotDrive;
    public ArmSubsystem robotArm;
    public IntakeSubsystem robotIntake;

    public AutonSubsystem(DriveSubsystem robotDrive, ArmSubsystem robotArm, IntakeSubsystem robotIntake) {
        this.robotDrive = robotDrive;
        this.robotArm = robotArm;
        this.robotIntake = robotIntake;
    }

    public Command autonomousCmd(int auton) {
        switch(auton) {
            /* CONE > MOBILITY */
            case 1:
            return new SequentialCommandGroup (
                armConeHigh(),
                inCubeOutCone(),
                armToIdle(1.5),
                driveBack(2.6)
            );

            /* CONE > MOBILITY > DOCK */
            case 2:
            return new SequentialCommandGroup (
                armConeHigh(),
                inCubeOutCone(),
                armToIdle(1.7),
                driveBack(3.25),  // This time is longer because it has to go over the charge station   
                driveFront(2.25),
                enableAutoEngage()
            );

            /* CONE ONLY */
            case 3:
            return new SequentialCommandGroup(
                armConeHigh(),
                inCubeOutCone(),
                armToIdle(1.5)
            );

            /* CUBE ONLY */
            case 4:
            return new SequentialCommandGroup(
                armCubeHigh(),
                inConeOutCube(),
                armToIdle(1.5)
            );

            /* CUBE > MOBILITY > GRAB CUBE */
            case 5:
            return new SequentialCommandGroup(
                armCubeHigh(),
                inConeOutCube(),
                armToIdle(1.5),
                driveBack(2.3),
                startTurn(),
                armCubeGround(),
                enableHoldArm(),
                new ParallelCommandGroup(
                    rawInCubeOutCone(),
                    rawDriveFront()
                ).withTimeout(1.12)
            );

            /* CONE > MOBILITY > TURN > EXTEND */
            case 6:
            return new SequentialCommandGroup(
                armConeHigh(),
                inCubeOutCone(),
                armToIdle(1.5),
                driveBack(3.3),
                startTurn(),
                armConeGround(),
                enableHoldArm()
            );
            

            /* TEST TURN */
            case 7:
            return new SequentialCommandGroup(
                armConeHigh(),
                inCubeOutCone(),
                armToIdle(1.5),
                driveBack(2.3),
                startTurn()
            );

            default: 
            return new SequentialCommandGroup (
                new InstantCommand( () -> System.out.println("ERROR: Autonomous Failure."))
            );
        }
    } 

    public Command rawDriveFront(){
        return new ArcadeCommand(() -> -0.75, () -> 0.0, robotDrive);
    }

    public Command rawInCubeOutCone(){
        return new IntakeCommand(robotIntake, 0.5);
    }

    public Command armConeHigh(){
        System.out.println("\n\nARM CONE HIGH\n\n");
        return new AutonArmCommand(robotArm, "high", "cone").withTimeout(1.9);
    }

    public Command armCubeHigh(){
        System.out.println("\n\nARM CUBE HIGH\n\n");
        return new AutonArmCommand(robotArm, "high", "cube").withTimeout(1.9);
    }

    public Command inCubeOutCone(){
        System.out.println("\n\nIN CUBE OUT CONE\n\n");
        return new IntakeCommand(robotIntake, 0.5).withTimeout(0.5);
    }

    public Command armToIdle(double time){
        System.out.println("\n\nARM TO IDLE\n\n");
        return new AutonArmCommand(robotArm, "idle", "").withTimeout(time);
    }

    public Command driveBack(double time){
        return new ArcadeCommand(() -> -0.6, () -> 0.0, robotDrive).withTimeout(time);
    }

    public Command driveFront(double time){
        return new ArcadeCommand(() -> 0.6, () -> 0.0, robotDrive).withTimeout(time);
    }

    public Command startTurn(){
        return new ArcadeCommand(() -> 0.0, () -> 0.775, robotDrive).withTimeout(1.5);
    }

    public Command armConeGround(){
        return new AutonArmCommand(robotArm, "ground", "cone").withTimeout(3.4);
    }

    public Command armCubeGround(){
        System.out.println("\n\nARM CUBE GROUND\n\n");
       return new AutonArmCommand(robotArm, "ground", "cube").withTimeout(3.4);
    }
    
    public Command inConeOutCube(){
        System.out.println("\n\nSPIT CUBE\n\n");
       return new IntakeCommand(robotIntake, -0.5).withTimeout(0.5);
    }

    public Command enableAutoEngage(){
        return new AutoEngageCommand(robotDrive).withTimeout(5);
    }

    public Command enableHoldArm(){
        return new InstantCommand(() -> GameStates.shouldHoldArm = true);
    }

    // public Command disableHoldArm(){
    //     return new InstantCommand(() -> GameStates.shouldHoldArm = false);
    // }
}
