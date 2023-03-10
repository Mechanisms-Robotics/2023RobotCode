package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.util.Field2dWrapper;
import frc.robot.util.HeadingController;

public class Swerve extends SubsystemBase {

    public static final double ANGULAR_VELOCITY_RANGE = Math.PI / 2;

    private final Field2dWrapper field;

    private final HeadingController headingController =
            new HeadingController(
                    0.08, // Stabilization kP
                    0.0, // Stabilization kD
                    1.75, // Lock kP
                    0.0, // Lock kI
                    0.0, // Lock kD
                    2.0, // Turn in place kP
                    0.0, // Turn in place kI
                    0.0 // Turn in place kD
            );

    private final Pose2d[] swerveModulePoses = new Pose2d[4];

    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public WPI_Pigeon2 gyro;

    private Rotation2d simYaw = new Rotation2d();

    private boolean runningTrajectory = false;
    private PathPlannerTrajectory currentTrajectory;

    private Rotation2d upAngle;

    private double maxVelocityRange = 4.5; // m/s
    private Rotation2d swerveRobotRelativeHeading = new Rotation2d();

    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();

    public Swerve() {
        gyro = new WPI_Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.FrontLeftMod.constants),
                new SwerveModule(1, Constants.Swerve.FrontRightMod.constants),
                new SwerveModule(2, Constants.Swerve.BackLeftMod.constants),
                new SwerveModule(3, Constants.Swerve.BackRightMod.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());

        field = new Field2dWrapper();

        upAngle = Rotation2d.fromDegrees(gyro.getRoll());

        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates;

        if (fieldRelative) {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw());
        } else {
            currentSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(currentSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        headingController.stabiliseHeading();
    }

    public void stop() {
        drive(new Translation2d(0.0, 0.0), 0.0, false, true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d heading) {
        poseEstimator.resetPosition(heading, getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public Rotation2d getUpAngle() {
        return upAngle;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getMaxVelocity() {
        return maxVelocityRange;
    }

    public ChassisSpeeds getVelocity() {
        return currentSpeeds;
    }

    public Field2dWrapper getField() {
        return field;
    }

    public void setRunningTrajectory(boolean isRunning) {
        runningTrajectory = isRunning;
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        this.currentTrajectory = trajectory;
    }

    public PathPlannerTrajectory getTrajectory() {
        return currentTrajectory;
    }

    @Override
    public void periodic(){
        poseEstimator.update(getYaw(), getModulePositions());

        if (!runningTrajectory) {
            headingController.update(currentSpeeds, getYaw());
        }

        swerveRobotRelativeHeading =
                new Rotation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        maxVelocityRange = 2.5 * (0.25 * Math.cos(2 * swerveRobotRelativeHeading.getRadians()) + 0.75);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        for (int i = 0; i < 4; i++) {
            Translation2d updatedPositions =
                    Constants.Swerve.swerveModuleTranslations[i]
                            .rotateBy(getPose().getRotation())
                            .plus(getPose().getTranslation());

            swerveModulePoses[i] =
                    new Pose2d(
                            updatedPositions,
                            new Rotation2d(mSwerveMods[i].getCanCoder().getRadians()).plus(getYaw()));
        }

        field.setRobotPose(poseEstimator.getEstimatedPosition());
        field.getObject("aSwerve Modules").setPoses(swerveModulePoses);
    }

    @Override
    public void simulationPeriodic() {
        simYaw =
                simYaw.plus(
                        new Rotation2d(
                                currentSpeeds.omegaRadiansPerSecond * Constants.LOOP_TIME));

        gyro.setYaw(simYaw.getDegrees());

        poseEstimator.update(simYaw, getModulePositions());

        field.setRobotPose(
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), simYaw));

        field.updateSims(getPose());
    }
}
