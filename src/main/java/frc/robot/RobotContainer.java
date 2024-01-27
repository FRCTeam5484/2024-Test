package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdSwerve_TeleOp;
import frc.robot.subsystems.subSwerve;

public class RobotContainer {
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  private final subSwerve swerve = new subSwerve();
  SendableChooser<Command> chooser = new SendableChooser<>();
  public RobotContainer() {
    configureDriverOne();
    addAutoOptions();
  }

  private void configureDriverOne() {
    swerve.setDefaultCommand(
      new cmdSwerve_TeleOp(
          swerve,
          () -> MathUtil.applyDeadband(-driverOne.getLeftY(), 0.01),
          () -> MathUtil.applyDeadband(driverOne.getLeftX(), 0.01),
          () -> MathUtil.applyDeadband(-driverOne.getRightX(), 0.01)));
  }

  private void addAutoOptions(){
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    SmartDashboard.putData("Auto Options", chooser);
  }

  public Command getAutonomousCommand() {
    try { return chooser.getSelected(); } 
    catch (NullPointerException ex) { 
      DriverStation.reportError("auto choose NULL somewhere in getAutonomousCommand in RobotContainer.java", null);
      return new InstantCommand();
    }
  }
}
