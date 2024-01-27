package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subSwerve;

public class cmdSwerve_TeleOp extends Command {
  private final subSwerve swerve;
  private final DoubleSupplier XSupplier, YSupplier, rotationSupplier;

  public cmdSwerve_TeleOp(subSwerve swerve, DoubleSupplier XSupplier, DoubleSupplier YSupplier, DoubleSupplier rotationSupplier) {
    this.swerve = swerve;
    this.XSupplier = XSupplier;
    this.YSupplier = YSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = XSupplier.getAsDouble()*Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeed = YSupplier.getAsDouble()*Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double rotationSpeed = rotationSupplier.getAsDouble()*Constants.DriveConstants.kMaxAngularSpeed;
    swerve.drive(xSpeed, ySpeed, rotationSpeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() { return false; }
}
