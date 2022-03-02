// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.field.AdvancedField2d;
import java.util.List;


public class RomiDrivetrain extends SubsystemBase
{
    private static final double COUNTS_PER_REVOLUTION = 1440.0;
    private static final double WHEEL_DIAMETER_METER = 0.07;
    
    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    private final Spark leftMotor = new Spark(0);
    private final Spark rightMotor = new Spark(1);
    
    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder leftEncoder = new Encoder(4, 5);
    private final Encoder rightEncoder = new Encoder(6, 7);
    
    // Set up the differential drive controller
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);
    private final RomiGyro gyro = new RomiGyro();
    private final AdvancedField2d field = new AdvancedField2d();

    DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
            new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.
    
    
    /** Creates a new RomiDrivetrain. */
    public RomiDrivetrain()
    {
        // Use inches as unit for encoder distances
        leftEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_METER) / COUNTS_PER_REVOLUTION);
        rightEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_METER) / COUNTS_PER_REVOLUTION);
        resetEncoders();
        
        // Invert right side since motor is flipped
        rightMotor.setInverted(true);
        estimator.resetPosition(new Pose2d(getLeftDistanceInch(), getRightDistanceInch(), gyro.getRotation2d()), gyro.getRotation2d());
        Shuffleboard.getTab("Field").add(field).withPosition(0, 0).withSize(5, 4);
        field.getObject("a").setPoses(new Pose2d(), new Pose2d(1, 2, new Rotation2d()));
        TrajectoryConfig config = new TrajectoryConfig(
            1,
            2);


        field.getObject("t").setTrajectory(TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(),
            new Pose2d(5, 2, Rotation2d.fromDegrees(30)),
            config
        ));
    }
    
    
    public void arcadeDrive(double xAxisSpeed, double zAxisRotate)
    {
        diffDrive.arcadeDrive(xAxisSpeed, zAxisRotate);
    }
    
    
    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    private void resetPose() {
        estimator.resetPosition(estimator.getEstimatedPosition(), gyro.getRotation2d());
    }

    public void setPose(Pose2d pose) {
        resetEncoders();
        estimator.resetPosition(pose, gyro.getRotation2d());
    }
    
    
    public double getLeftDistanceInch()
    {
        return leftEncoder.getDistance();
    }
    
    
    public double getRightDistanceInch()
    {
        return rightEncoder.getDistance();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }
    
    @Override
    public void periodic()
    {
        estimator.update(gyro.getRotation2d(), getWheelSpeeds(), getLeftDistanceInch(), getRightDistanceInch());

        field.setRobotPose(estimator.getEstimatedPosition());
        field.getObject("test").setPose(1, 2, Rotation2d.fromDegrees(30));
    }
    
    
    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
