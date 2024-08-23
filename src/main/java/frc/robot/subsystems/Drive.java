// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.SwerveUtils;

public class Drive extends SubsystemBase {
  private static Drive m_drive;
  private final Field2d m_field;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      RobotMap.D_FRONT_LEFT_DRIVE,
      RobotMap.D_FRONT_LEFT_TURNING,
      Constants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      RobotMap.D_FRONT_RIGHT_DRIVE,
      RobotMap.D_FRONT_RIGHT_TURNING,
      Constants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      RobotMap.D_BACK_LEFT_DRIVE,
      RobotMap.D_BACK_LEFT_TURNING,
      Constants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      RobotMap.D_BACK_RIGHT_DRIVE,
      RobotMap.D_BACK_RIGHT_TURNING,
      Constants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private ChassisSpeeds m_lastSpeeds = new ChassisSpeeds();

  // SwerveDrivePoseEstimator class for tracking robot pose
  SwerveDrivePoseEstimator m_DrivePoseEstimator;

  // testing dashboard instances
  TDNumber TDxSpeedCommanded;
  TDNumber TDySpeedCommanded;
  TDNumber TDrotSpeedCommanded;
  
  TDNumber TDxSpeedMeasured;
  TDNumber TDySpeedMeasured;
  TDNumber TDrotSpeedMeasured;

  TDNumber TDPoseX;
  TDNumber TDPoseY;
  TDNumber TDPoseAngle;

  ChassisSpeeds m_requestedSpeeds = new ChassisSpeeds();
  ChassisSpeeds m_limitSpeeds = new ChassisSpeeds();
  double m_driveTime = 0;

  TDNumber m_DLeftFrontCurrentOutput;
  TDNumber m_DRightFrontCurrentOutput;
  TDNumber m_DLeftBackCurrentOutput;
  TDNumber m_DRightBackCurrentOutput;

  TDNumber m_TLeftFrontCurrentOutput;
  TDNumber m_TRightFrontCurrentOutput;
  TDNumber m_TLeftBackCurrentOutput;
  TDNumber m_TRightBackCurrentOutput;

  /** Creates a new Drive. */
  private Drive() {
    super("Drive");
    m_DrivePoseEstimator = new SwerveDrivePoseEstimator(
        Constants.kDriveKinematics, 
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
      }, new Pose2d());

    m_field = new Field2d();
    new TDSendable(this, "Field", "Position", m_field);

    TDxSpeedCommanded = new TDNumber(this, "Drive Input", "XInputSpeed");
    TDySpeedCommanded = new TDNumber(this, "Drive Input", "YInputSpeed");
    TDrotSpeedCommanded = new TDNumber(this, "Drive Input", "RotInputSpeed");

    TDxSpeedMeasured = new TDNumber(this, "Drive Speed", "XMeasuredSpeed");
    TDySpeedMeasured = new TDNumber(this, "Drive Speed", "YMeasuredSpeed");
    TDrotSpeedMeasured = new TDNumber(this, "Drive Speed", "RotMeasuredSpeed");
    
    TDPoseX = new TDNumber(this, "Drive Pose", "PoseX");
    TDPoseY = new TDNumber(this, "Drive Pose", "PoseY");
    TDPoseAngle = new TDNumber(this, "Drive Pose", "PoseAngle");

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getMeasuredSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        Constants.kPathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

  m_DLeftFrontCurrentOutput = new TDNumber(this, "Current", "Drive Front Left Output", m_frontLeft.getDriveOutputCurrent());
  m_DRightFrontCurrentOutput = new TDNumber(this, "Current", "Drive Front Right Output", m_frontRight.getDriveOutputCurrent());
  m_DLeftBackCurrentOutput = new TDNumber(this, "Current", "Drive Back Left Output", m_rearLeft.getDriveOutputCurrent());
  m_DRightBackCurrentOutput = new TDNumber(this, "Current", "Drive Back Right Output", m_rearRight.getDriveOutputCurrent());

  m_TLeftFrontCurrentOutput = new TDNumber(this, "Current", "Turning Front Left Output", m_frontLeft.getTurningOutputCurrent());
  m_TRightFrontCurrentOutput = new TDNumber(this, "Current", "Turning Front Right Output", m_frontRight.getTurningOutputCurrent());
  m_TLeftBackCurrentOutput = new TDNumber(this, "Current", "Turning Back Left Output", m_rearLeft.getTurningOutputCurrent());
  m_TRightBackCurrentOutput = new TDNumber(this, "Current", "Turning Back Right Output", m_rearRight.getTurningOutputCurrent());
  }

  public static Drive getInstance() {
    if (m_drive == null) {
      m_drive = new Drive();
    }
    return m_drive;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_DrivePoseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    updateTD();
    super.periodic();
  }

  public void simulationPeriodic()
  {
    double now = Timer.getFPGATimestamp();

    Pose2d lastPose = getPose();
    if (m_driveTime != 0 && m_limitSpeeds != null)
    {
      ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_limitSpeeds, lastPose.getRotation());
      double deltax = fieldRelativeSpeeds.vxMetersPerSecond * (now - m_driveTime);
      double deltay = fieldRelativeSpeeds.vyMetersPerSecond * (now - m_driveTime);
      Rotation2d rot = new Rotation2d(fieldRelativeSpeeds.omegaRadiansPerSecond * (now - m_driveTime));
      Translation2d translation = new Translation2d(deltax, deltay);
      Pose2d newPose = new Pose2d(lastPose.getTranslation().plus(translation),
                                  lastPose.getRotation().plus(rot));
      resetOdometry(newPose);
    }

    m_driveTime = now;
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return Current robot pose.
   */
  public Pose2d getPose() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getMeasuredSpeeds(){
    if(RobotBase.isReal()){
      SwerveModuleState[] moduleStates = new SwerveModuleState[4];
      moduleStates[0] = m_frontLeft.getState();
      moduleStates[1] = m_frontRight.getState();
      moduleStates[2] = m_rearLeft.getState();
      moduleStates[3] = m_rearRight.getState();

      return Constants.kDriveKinematics.toChassisSpeeds(moduleStates);
    } else {
      return m_limitSpeeds;
    }
  }

  /**
   * Updates pose estimator with estimate from the Vision subsystem.
   * 
   * @param pose Current pose of the robot.
   * @param timestamp When the vision measurement was taken.
   * @param stdDevs Confidence.
   */
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    m_DrivePoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_DrivePoseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.kMaxSpeedMetersPerSecond;

    if (Constants.INPUT_MODE == Constants.INPUT_MODE_TYPE.DemoControllerSolo) {
      Pose2d currentPose = getPose();
      xSpeedDelivered = (xSpeedDelivered > 0 & currentPose.getX() > Constants.DEMO_BOX_WIDTH) ? 0 : xSpeedDelivered;
      xSpeedDelivered = (xSpeedDelivered < 0 & currentPose.getX() <-Constants.DEMO_BOX_WIDTH) ? 0 : xSpeedDelivered;

      ySpeedDelivered = (ySpeedDelivered > 0 & currentPose.getY() > Constants.DEMO_BOX_WIDTH) ? 0 : ySpeedDelivered;
      ySpeedDelivered = (ySpeedDelivered < 0 & currentPose.getY() <-Constants.DEMO_BOX_WIDTH) ? 0 : ySpeedDelivered;
    }
    double rotDelivered = m_currentRotation * Constants.kMaxAngularSpeed;
    Rotation2d fieldAngle = m_DrivePoseEstimator.getEstimatedPosition().getRotation()
                                  .plus(FieldUtils.getInstance().getRotationOffset());//Compensates for alliance

    drive(
      fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldAngle)
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    TDxSpeedCommanded.set(speeds.vxMetersPerSecond);
    TDySpeedCommanded.set(speeds.vyMetersPerSecond);
    TDrotSpeedCommanded.set(speeds.omegaRadiansPerSecond);
    m_requestedSpeeds = speeds;
    ChassisSpeeds limitedSpeeds = limitRates(speeds);
    m_limitSpeeds = limitedSpeeds;
    var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(limitedSpeeds);
    setModuleStates(swerveModuleStates);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees.
   */
  public double getHeading() {
    return getPose().getRotation().getDegrees();
  }

  public double getGyroAngle() {
    return m_gyro.getAngle(IMUAxis.kZ);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  private void updateTD(){

    m_field.setRobotPose(getPose());

    ChassisSpeeds measuredSpeeds = getMeasuredSpeeds();
    TDxSpeedMeasured.set(measuredSpeeds.vxMetersPerSecond);
    TDySpeedMeasured.set(measuredSpeeds.vyMetersPerSecond);
    TDrotSpeedMeasured.set(measuredSpeeds.omegaRadiansPerSecond);

    Pose2d currentPose = getPose();
    TDPoseX.set(currentPose.getX());
    TDPoseY.set(currentPose.getY());
    TDPoseAngle.set(currentPose.getRotation().getDegrees());

    m_DLeftFrontCurrentOutput.set(m_frontLeft.getDriveOutputCurrent());
    m_DRightFrontCurrentOutput.set(m_frontRight.getDriveOutputCurrent());
    m_DLeftBackCurrentOutput.set(m_rearLeft.getDriveOutputCurrent());
    m_DRightBackCurrentOutput.set(m_rearRight.getDriveOutputCurrent());

    m_TLeftFrontCurrentOutput.set(m_frontLeft.getTurningOutputCurrent());
    m_TRightFrontCurrentOutput.set(m_frontRight.getTurningOutputCurrent());
    m_TLeftBackCurrentOutput.set(m_rearLeft.getTurningOutputCurrent());
    m_TRightBackCurrentOutput.set(m_rearRight.getTurningOutputCurrent());
  }
  
  public ChassisSpeeds limitRates(ChassisSpeeds commandedSpeeds) {
    //Apply Speed Limits
    double linearSpeed = Math.hypot(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
    Rotation2d direction = new Rotation2d(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
    double limitedSpeed = Math.min(linearSpeed, Constants.kMaxSpeedMetersPerSecond);
    double limitedTheta = MathUtil.clamp(commandedSpeeds.omegaRadiansPerSecond,
                                         -Constants.kMaxAngularSpeed, Constants.kMaxAngularSpeed);

    Translation2d linearVelocity = new Pose2d(new Translation2d(), direction)
                  .transformBy(new Transform2d(new Translation2d(limitedSpeed, 0.0), new Rotation2d()))
                  .getTranslation();
    
    //Apply Acceleration Limits
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double accelerationDif = Constants.kMaxAccelerationMetersPerSecondSquared * (currentTime-m_prevTime);
    double xSpeed = MathUtil.clamp(linearVelocity.getX(),
                                   m_lastSpeeds.vxMetersPerSecond - accelerationDif, 
                                  m_lastSpeeds.vxMetersPerSecond + accelerationDif);
    double ySpeed = MathUtil.clamp(linearVelocity.getY(), 
                                  m_lastSpeeds.vyMetersPerSecond - accelerationDif,
                                  m_lastSpeeds.vyMetersPerSecond + accelerationDif);

    double thetaAccelDif = Constants.kMaxAngularSpeedRadiansPerSecondSquared * (currentTime - m_prevTime);
    double thetaSpeed = MathUtil.clamp(limitedTheta, 
                                      m_lastSpeeds.omegaRadiansPerSecond - thetaAccelDif, 
                                      m_lastSpeeds.omegaRadiansPerSecond + thetaAccelDif);

    ChassisSpeeds limitedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    m_lastSpeeds = limitedSpeeds;
    m_prevTime = currentTime;
    return limitedSpeeds;
  }
}