// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //auto
  private final String option1Auto = "Option1";
  private final String option2Auto = "Option2";

   String optionChosenAuto;

  private final SendableChooser<String> autoChooser = new SendableChooser<>();


  //controllers
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  //variables for swerve drive
  ChassisSpeeds speed;
  SwerveSubsystem swerveSubsystem;

  //sparkMax variables
  CANSparkMax sparkMax;
  SparkMaxPIDController sparkMaxPidController;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { // initialize stuff
    autoChooser.setDefaultOption("Option1", option1Auto);
    autoChooser.addOption("Option1", option1Auto);
    autoChooser.addOption("Option2", option2Auto);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerveSubsystem = new SwerveSubsystem(); //initialize swerve

    sparkMax = new CANSparkMax(MechanismConstants.kSparkMaxPort, MotorType.kBrushless); //initialize motor
    sparkMaxPidController = sparkMax.getPIDController(); //initialize motor PID controller
    
    sparkMaxPidController.setP(MechanismConstants.kPSparkMax); //set PID's P
    sparkMaxPidController.setI(MechanismConstants.kISparkMax); //set PID's I
    sparkMaxPidController.setD(MechanismConstants.kDSparkMax); //set PID's D
    sparkMaxPidController.setOutputRange(MechanismConstants.kMinSparkMax, MechanismConstants.kMaxSparkMax); //set range of PID
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    optionChosenAuto = autoChooser.getSelected();
    System.out.println("Auto selected: " + autoChooser);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (optionChosenAuto) {
      case option1Auto:
        // Put custom auto code here
        break;
      case option2Auto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //this is for swerve driving
    joystickDrive(
      driverController.getRawAxis(OIConstants.kDriverXAxis), 
      driverController.getRawAxis(OIConstants.kDriverYAxis), 
      driverController.getRawAxis(OIConstants.kDriverRotAxis), 
      driverController.getRawButton(OIConstants.kResetDirectionButton), 
      driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
    );
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  //custom functions
  public void sparkMaxPosition(double angle) { //this function is for setting the spark max to a specific angle (degrees)
    sparkMaxPidController.setReference(-1*(angle/360)*MechanismConstants.kReductionSparkMax, CANSparkMax.ControlType.kPosition);
  }

  public void sparkMaxManual(double speed) { //this function is for setting a power to the spark max (-1 - 1)
    sparkMax.set(speed);
  }

  public void joystickDrive(double x, double y, double theta, boolean resetDirection, boolean fieldOriented) {
        SlewRateLimiter xLimiter, yLimiter, turningLimiter;
        xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        // 1. Get real-time joystick inputs
        double xSpeed = x;
        double ySpeed = y;
        double turningSpeed = theta;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        
        if(resetDirection) {
            swerveSubsystem.zeroHeading();
        }

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        
        if(fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
  }
}
