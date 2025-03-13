package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.AprilTagHeightDB;

public class AutoAlign extends Command {
    private final DriveSubsystem m_swerve;
    // Corrected table name
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private final PIDController rotationPID = new PIDController(0.05, 0, 0); // Tune
    private final PIDController distancePID = new PIDController(0.1, 0, 0); // Tune
    private final double rotationTolerance = 0.20; // Degrees
    private final double distanceTolerance = 0.1; // Meters

    private final double limelightHeight = 0.2032; // Meters - Adjust to your Limelight height
    private final double limelightAngle = 0.0; // Degrees - Adjust to your Limelight angle

    public AutoAlign(DriveSubsystem swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(0);
        distancePID.setSetpoint(1.0); // Set the desired distance (example: 1 meter)
    }

    @Override
    public void execute() {
        double tv = table.getEntry("tv").getDouble(0.0);
        double tx = table.getEntry("tx").getDouble(0.0);
        double tid = table.getEntry("tid").getDouble(0.0);
        double targetHeight = AprilTagHeightDB.getHeight(tid);

        System.out.println("tv: " + tv + ", tx: " + tx); // Debugging output

        if (tv == 1.0) {
            double rotationOutput = rotationPID.calculate(tx);

            if (targetHeight != -1) {
                double distanceOutput = distancePID.calculate(calculateDistance(targetHeight));

                rotationOutput = Math.max(Math.min(rotationOutput, 0.2), -0.2); // Limit speed
                distanceOutput = Math.max(Math.min(distanceOutput, 0.2), -0.2); // Limit speed

                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        distanceOutput, // Forward/backward
                        0.0, // No left/right
                        Math.toRadians(rotationOutput), // Angular velocity
                        m_swerve.getRotation2d()
                );

                m_swerve.drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, true);
                System.out.println("rotationOutput: " + rotationOutput + ", distanceOutput: " + distanceOutput + ", omega: " + chassisSpeeds.omegaRadiansPerSecond + ", vx: " + chassisSpeeds.vxMetersPerSecond); // More debugging
            } else {
                m_swerve.drive(0.0, 0.0, 0.0, false);
            }
        } else {
            m_swerve.drive(0.0, 0.0, 0.0, false);
        }
    }

    @Override
    public boolean isFinished() {
        double tv = table.getEntry("tv").getDouble(0.0);
        if (tv == 0.0) {
            // Stop if the target is no longer visible
            return true;
        }

        double tx = table.getEntry("tx").getDouble(0.0);
        double tid = table.getEntry("tid").getDouble(0.0);
        double targetHeight = AprilTagHeightDB.getHeight(tid);

        if (targetHeight == -1) {
            // Stop if we can't get the target height.
            return true;
        }

        double distanceError = Math.abs(calculateDistance(targetHeight) - distancePID.getSetpoint());

        // Check if both rotation and distance are within tolerances
        boolean finished = Math.abs(tx) < rotationTolerance && distanceError < distanceTolerance;
        System.out.println("Finished: " + finished);
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0.0, 0.0, 0.0, false);
    }

    private double calculateDistance(double targetHeight) {
        double ty = table.getEntry("ty").getDouble(0.0);
        double angleToTarget = ty + limelightAngle;
        return (targetHeight - limelightHeight) / Math.tan(Math.toRadians(angleToTarget));
    }
}