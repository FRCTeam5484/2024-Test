package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int DriverOne = 0;
    public static final int DriverTwo = 1;
  }
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
  }
}