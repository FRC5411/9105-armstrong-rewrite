package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.DriveSubsystem;

public class TurnCommand extends CommandBase {
    private ProfiledPIDController controller;
    private DoubleSupplier setPointSupplier;
    private DoubleSupplier measureSupplier;
    private DriveSubsystem system;

    public TurnCommand (ProfiledPIDController controller, DoubleSupplier setPointSupplier, DoubleSupplier measureSupplier, DriveSubsystem system) {
      this.controller = controller;
      this.setPointSupplier = setPointSupplier;
      this.measureSupplier = measureSupplier;

      this.system = system;

      addRequirements(system);
    }
    
    @Override
    public void initialize() {
      controller.reset(measureSupplier.getAsDouble());
    }

    @Override
    public void execute() {
      system.autonomousArcadeDrive(0, 
      MathUtil.clamp(controller.calculate(
        measureSupplier.getAsDouble(),
        setPointSupplier.getAsDouble()), -6, 6));
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  
    public void end() {}
  }
