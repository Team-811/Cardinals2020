
package frc.robot.subsystems;

import frc.robot.Kinematics;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Twist2d;

import edu.wpi.first.wpilibj.command.Subsystem;

public class RobotStateEstimator extends Subsystem {
    private RobotState robot_state_ = RobotState.getInstance();
    private Drivetrain drive = Robot.drivetrain;
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double strafe_encoder_prev_distance_ = 0.0;

    public RobotStateEstimator() 
    {
    
        left_encoder_prev_distance_ = drive.getLeftEncoder();
        right_encoder_prev_distance_ = drive.getRightEncoder();
        strafe_encoder_prev_distance_ = drive.getStrafeEncoder();

    }

    
    public void runRobotStateEstimator(double timestamp)  {
        final double left_distance = drive.getLeftEncoder();
        final double right_distance = drive.getRightEncoder();
        final double strafe_distance = drive.getStrafeEncoder();
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final double delta_strafe = strafe_distance - strafe_encoder_prev_distance_;
        final Rotation2d gyro_angle = Rotation2d.fromDegrees(drive.getGyroAngle());
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                delta_left, delta_right, delta_strafe, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive.getLeftLinearVelocity(),
                drive.getRightLinearVelocity());
        robot_state_.addObservations(timestamp, odometry_velocity,
                predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

        

        @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

}
