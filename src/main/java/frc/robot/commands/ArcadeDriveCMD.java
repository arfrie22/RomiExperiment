package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import java.util.function.Supplier;

public class ArcadeDriveCMD extends CommandBase {
    private final RomiDrivetrain drivetrain;
    private final Supplier<Double> speedSupplier;
    private final Supplier<Double> rotateSupplier;

    public ArcadeDriveCMD(RomiDrivetrain drivetrain, Supplier<Double> speedSupplier, Supplier<Double> rotateSupplier) {
        this.drivetrain = drivetrain;
        this.speedSupplier = speedSupplier;
        this.rotateSupplier = rotateSupplier;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.arcadeDrive(speedSupplier.get(), rotateSupplier.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
