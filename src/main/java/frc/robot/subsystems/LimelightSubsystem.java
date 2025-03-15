package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagHeightDB;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;


public class LimelightSubsystem extends SubsystemBase {

    public static DriveSubsystem m_robotDrive;
    
        // If an AprilTag is seen/in the camera's range, light up the Limelight
        public static void lightUpLimelight() {
            final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            double tagID =  table.getEntry("tid").getDouble(0); //stupid syntax error that makes no sense; ill fix later
            if(tagID > 0 ){
            // Turn on light 
            table.getEntry("ledMode").setNumber(3);  // 3 means turn on LEDs
            }else{
            // Turn off light
            table.getEntry("ledMode").setNumber(1);  // 1 means turn off LEDs
            }
        }
    
        // If an AprilTag is seen/in the camera's range, return true/false
        public static boolean aprilTagUpdate(){
            final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            double TagFound =  table.getEntry("tv").getDouble(0);
            if (TagFound == 1) {
                return true;
            } else {
                return false;
            }
        }
    
        private static double getAprilTagHeight(){
            final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            final double ID = table.getEntry("tid").getDouble(0);
            return AprilTagHeightDB.getHeight(ID);
        }
    
    
        public static double getAprilTagDistance(){       
            final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            final double targetAngleOffset = table.getEntry("ty").getDouble(0.0);
    
            final double tagHeight = getAprilTagHeight();
    
            final double targetHeightOffset = tagHeight - Constants.LimelightConstants.MOUNT_HEIGHT;
            final double totalAngleDegrees = targetAngleOffset + Constants.LimelightConstants.MOUNT_ANGLE;
    
            final double totalAngleRadians = totalAngleDegrees*(3.14159/180.0);
            
            return targetHeightOffset/Math.tan(totalAngleRadians);
        }
    
        // In radians
        public static double getAprilTagAngle(){
            final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            final double angle = table.getEntry("tx").getDouble(0);
            return angle*(Math.PI/180);
        }

        public void alignChassis() {

                }
        
                  public Command alignChassisCommand() {
            return this.startEnd(
                () -> this.alignChassis(), () -> this.alignChassis());
  }
    
    }
