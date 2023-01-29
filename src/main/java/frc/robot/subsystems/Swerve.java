package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.SimpleFeedforwardConstants;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.api.ModuleConfig;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.HeadingController;
import frc.robot.util.TrajectoryController;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase {

    public static final double maxVelocity = 4.5; // m / s
    public static final double maxRotationalVelocity = 1.5 * Math.PI; // rads/s

    // The center of the robot is the origin point for all locations
    private static final double driveBaseWidth = 0.5969; // m
    private static final double driveBaseLength = 0.9017; // m

    private static final Translation2d flModuleLocation =
            new Translation2d(driveBaseWidth / 2.0, driveBaseLength / 2.0);
    private static final Translation2d frModuleLocation =
            new Translation2d(driveBaseWidth / 2.0, -driveBaseLength / 2.0);
    private static final Translation2d blModuleLocation =
            new Translation2d(-driveBaseWidth / 2.0, driveBaseLength / 2.0);
    private static final Translation2d brModuleLocation =
            new Translation2d(-driveBaseWidth / 2.0, -driveBaseLength / 2.0);

    private static final int flWheelMotorID = 10;
    private static final int flSteerMotorID = 11; // Done
    private static final int flSteerEncoderID = 10;
    private static final int frWheelMotorID = 16;
    private static final int frSteerMotorID = 17;
    private static final int frSteerEncoderID = 16;
    private static final int blWheelMotorID = 12;
    private static final int blSteerMotorID = 13;
    private static final int blSteerEncoderID = 12;
    private static final int brWheelMotorID = 14;
    private static final int brSteerMotorID = 15;
    private static final int brSteerEncoderID = 14;

    private static final double flAngleOffset = -292.23; // -290.12
    private static final double frAngleOffset = -239.76; // -238.00
    private static final double blAngleOffset = -198.54; // -196.69
    private static final double brAngleOffset = -97.82; // -95.80

    private static final int gyroID = 0;

    public static final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    flModuleLocation, frModuleLocation, blModuleLocation, brModuleLocation);

    public final SwerveDrivePoseEstimator poseEstimator;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    // Config injections into SDSlib to allow us to directly modify hardware objects (motors)
    private static final ModuleConfig DRIVE_CONFIG = motor -> config -> feedforwardConstants -> {
        motor.config_kP(0, 0.001);
        motor.selectProfileSlot(0, 0);
        feedforwardConstants.ks = 0.319185544;
        feedforwardConstants.kv = 2.2544;
        feedforwardConstants.ka = 0.063528;
    };

    private static final ModuleConfig STEER_CONFIG = motor -> config -> feedforwardConstants -> {
        motor.config_kP(0, 0.2);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.1);
    };

    private final SwerveModule flModule =
            Mk4iSwerveModuleHelper.createFalcon500(
                    tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                            .withSize(2, 2)
                            .withPosition(0, 0),
                    GearRatio.L3,
                    flWheelMotorID,
                    DRIVE_CONFIG,
                    flSteerMotorID,
                    STEER_CONFIG,
                    flSteerEncoderID,
                    Math.toRadians(flAngleOffset));

    private final SwerveModule frModule =
            Mk4iSwerveModuleHelper.createFalcon500(
                    tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                            .withSize(2, 2)
                            .withPosition(2, 0),
                    GearRatio.L3,
                    frWheelMotorID,
                    DRIVE_CONFIG,
                    frSteerMotorID,
                    STEER_CONFIG,
                    frSteerEncoderID,
                    Math.toRadians(frAngleOffset));

    private final SwerveModule blModule =
            Mk4iSwerveModuleHelper.createFalcon500(
                    tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0),
                    GearRatio.L3,
                    blWheelMotorID,
                    DRIVE_CONFIG,
                    blSteerMotorID,
                    STEER_CONFIG,
                    blSteerEncoderID,
                    Math.toRadians(blAngleOffset));

    private final SwerveModule brModule =
            Mk4iSwerveModuleHelper.createFalcon500(
                    tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                            .withSize(2, 2)
                            .withPosition(6, 0),
                    GearRatio.L3,
                    brWheelMotorID,
                    DRIVE_CONFIG,
                    brSteerMotorID,
                    STEER_CONFIG,
                    brSteerEncoderID,
                    Math.toRadians(brAngleOffset));

    public final PigeonIMU gyro = new PigeonIMU(gyroID);

    private final HeadingController headingController =
            new HeadingController(
                    0.005, // Stabilization kP
                    0.0, // Stabilization kD
                    1.75, // Lock kP
                    0.0, // Lock kI
                    0.0, // Lock kD
                    2.0, // Turn in place kP
                    0.0, // Turn in place kI
                    0.0 // Turn in place kD
            );
    private final TrajectoryController trajectoryController = new TrajectoryController(kinematics);
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private Rotation2d gyroAngleAdjustment = Rotation2d.fromDegrees(0.0);

    // Last ChassisSpeeds
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    // Current acceleration
    private Translation2d acceleration = new Translation2d();

    // Field2d Instance
    private final Field2d field2d = new Field2d();

    // Localized flag
    private boolean localized = false;

    /** Constructs the Swerve subsystem. */
    public Swerve() {
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        kinematics,
                        getHeading(),
//                        kinematics,
//                        VecBuilder.fill(0.05, 0.05, Math.toRadians(5.0)),
//                        VecBuilder.fill(Math.toRadians(0.01)),
//                        VecBuilder.fill(0.5, 0.5, Math.toRadians(30.0))
                );

        gyro.setFusedHeading(0.0);
        this.register();
        this.setName("Swerve Drive");

        // Put the Field2d instance on the SmartDashboard
        SmartDashboard.putData("Field", this.field2d);
    }

    @Override
    public void periodic() {
        poseEstimator.update(
                getHeading(),
                flModule.getState(),
                frModule.getState(),
                blModule.getState(),
                brModule.getState());

        // Update Field2d with the latest pose estimate
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());

        // If we are currently running a trajectory
        if (trajectoryController.isFinished()) {
            headingController.update(desiredSpeeds, getHeading());
        } else {
            desiredSpeeds = trajectoryController.calculate(getPose());
        }
        setSwerveStates(desiredSpeeds);

        // Calculate current acceleration
        acceleration =
                new Translation2d(
                        (getSpeeds().vxMetersPerSecond - lastSpeeds.vxMetersPerSecond),
                        (getSpeeds().vyMetersPerSecond - lastSpeeds.vyMetersPerSecond));

        SmartDashboard.putNumber("Target vX", desiredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Target vY", desiredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Target vR", desiredSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        SmartDashboard.putNumber("Robot Heading", getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("Localized", this.localized);
        SmartDashboard.putNumber("Acceleration", acceleration.getNorm());
    }

    public void drive(
            double xVelocity, double yVelocity, double rotationVelocity, boolean fieldRelative) {

        headingController.stabiliseHeading();

        if (!trajectoryController.isFinished()) trajectoryController.stop();

        if (fieldRelative) {
            desiredSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity, yVelocity, rotationVelocity, getHeading());
        } else {
            desiredSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
        }
    }

    public void drive(double xVelocity, double yVelocity, Rotation2d rotation) {
        if (!trajectoryController.isFinished()) trajectoryController.stop();
        headingController.lockHeading(rotation);
        // We don't set the rotation speeds as the heading controller will take care of that.
        desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, 0.0, getHeading());
    }

    public void stop() {
        drive(0.0, 0.0, 0.0, true);
    }

    private void setSwerveStates(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    /**
     * Sets all the swerve module states sequentially.
     *
     * @param states What to set the states to
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
        flModule.setState(states[0]);
        frModule.setState(states[1]);
        blModule.setState(states[2]);
        SmartDashboard.putNumber("Back Left dV", states[2].speedMetersPerSecond);
        brModule.setState(states[3]);
    }

    /**
     * Get the states of all the current modules.
     *
     * @return A list of SwerveModulesState(s)
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = flModule.getState();
        states[1] = frModule.getState();
        states[2] = blModule.getState();
        states[3] = brModule.getState();
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
    }

    /** Zeros the gyro heading. */
    public void zeroHeading() {
        final var zeroRotation = new Rotation2d();
        final var currentTranslation = poseEstimator.getEstimatedPosition().getTranslation();
        setHeading(zeroRotation);
        poseEstimator.resetPosition(new Pose2d(currentTranslation, zeroRotation), zeroRotation);
    }

    private void setHeading(Rotation2d heading) {
        gyroAngleAdjustment = heading.rotateBy(getRawHeading().unaryMinus());
    }

    private Rotation2d getRawHeading() {
        // For the Pigeon 1 use getFusedHeading the Pigeon 2 uses getYaw. See the Pigeon 2 user guide.
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Returns the swerve drive's heading as a Rotation2d.
     *
     * @return A Rotation2d representing the swerve drive's heading
     */
    public Rotation2d getHeading() {
        return gyroAngleAdjustment.rotateBy(getRawHeading());
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Translation2d getAcceleration() {
        return this.acceleration;
    }

    /**
     * Gets the velocity of the drivetrain in m/s
     *
     * @return The velocity of the drivetrain
     */
    public double getVelocity() {
        return Math.sqrt(
                Math.pow(getSpeeds().vxMetersPerSecond, 2) + Math.pow(getSpeeds().vyMetersPerSecond, 2));
    }

    /**
     * Gets the angular velocity of the drivetrain in rads/s
     *
     * @return The angular velocity of the drivetrain in rads/s
     */
    public double getAngularVelocity() {
        return getSpeeds().omegaRadiansPerSecond;
    }

    /**
     * Get the current pose of the robot.
     *
     * @return A Pose2d that represents the position of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose, Rotation2d heading) {
        this.localized = true;

        setHeading(heading);
        poseEstimator.resetPosition(pose, heading);
    }

    public void resetSensors() {
        zeroHeading();
        poseEstimator.resetPosition(new Pose2d(), new Rotation2d());
    }

    /** Returns whether the swerve has been localized or not */
    public boolean hasBeenLocalized() {
        return this.localized;
    }

    /**
     * Start following a trajectory.
     *
     * @param trajectory The trajectory to follow.
     */
    public void followTrajectory(PathPlannerTrajectory trajectory) {
        trajectoryController.startTrajectory(trajectory);
    }

    /**
     * Returns if the trajectory controller is finished.
     *
     * @return True if the trajectory is finished. False otherwise.
     * @see TrajectoryController
     */
    public boolean isTrajectoryFinished() {
        return trajectoryController.isFinished();
    }
}