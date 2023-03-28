
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoEngageCommand;
import frc.robot.commands.IntakeCommand;

public class AutonSubsystem {

    private DriveSubsystem robotDrive;
    private ArmSubsystem robotArm;
    private IntakeSubsystem robotIntake;

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
                new ArmCommand(robotArm, 175).withTimeout(1.9),
                new IntakeCommand(robotIntake, 0.5).withTimeout(0.5),
                new ArmCommand(robotArm, 59).withTimeout(1.5),
                new ArcadeCommand(() -> 0.75, () -> 0.0, robotDrive).withTimeout(3.3)       
            );

            /* CONE > MOBILITY > DOCK */
            case 2:
            return new SequentialCommandGroup (
                new ArmCommand(robotArm, 175).withTimeout(1.9),
                new IntakeCommand(robotIntake, 0.5).withTimeout(0.5),
                new ArmCommand(robotArm, 3).withTimeout(1.7),
                new ArcadeCommand(() -> 0.75, () -> 0.0, robotDrive).withTimeout(3.5),     
                new ArcadeCommand(() -> -0.75, () -> 0.0, robotDrive).withTimeout(2.25),
                //new ArcadeCommand(() -> -0.5, () -> 0.0, robotDrive).withTimeout(1),
                //new ArcadeCommand(() -> -0.25, () -> 0.0, robotDrive).withTimeout(4.5)
                new AutoEngageCommand(robotDrive).withTimeout(5)
            );

            /* CONE ONLY */
            case 3:
            return new SequentialCommandGroup(
                new ArmCommand(robotArm, 175).withTimeout(1.9),
                new IntakeCommand(robotIntake, 0.5).withTimeout(0.5),
                new ArmCommand(robotArm, 59).withTimeout(1.5)
            );

            /* CUBE ONLY */
            case 4:
            return new SequentialCommandGroup(
                new ArmCommand(robotArm, 175).withTimeout(1.9),
                new IntakeCommand(robotIntake, -0.5).withTimeout(0.5),
                new ArmCommand(robotArm, 59).withTimeout(1.5)
            );

            /* CONE > MOBILITY > GRAB CUBE */
            case 5:
            return new SequentialCommandGroup(
                new ArmCommand(robotArm, 175).withTimeout(1.9),
                new IntakeCommand(robotIntake, 0.5).withTimeout(0.5),
                new ArmCommand(robotArm, 59).withTimeout(1.5),
                new ArcadeCommand(() -> 0.75, () -> 0.0, robotDrive).withTimeout(3.3),
                new ArcadeCommand(() -> 0, () -> 0.75, robotDrive).withTimeout(1.85),
                new ArcadeCommand(() -> -0.75, () -> 0.0, robotDrive).withTimeout(3.1),
                new ArmCommand(robotArm, 260.5).withTimeout(0.25),
                new IntakeCommand(robotIntake, -0.5).withTimeout(0.5)
            );

            default: 
            return new SequentialCommandGroup (
                new InstantCommand( () -> { System.out.println("ERROR: Autonomous Failure."); })
            );
        }
    } 
}
