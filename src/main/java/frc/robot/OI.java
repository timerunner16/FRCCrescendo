/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Sensors.ResetAllSensors;
import frc.robot.commands.Sensors.ToggleSensorsOnOff;
import frc.robot.commands.Shooter.SpinUpShooter;
import frc.robot.subsystems.BarrelPivot;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.SwerveDriveInputs;
import frc.robot.commands.MoveNoteBackward;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.MoveNoteForward;
import frc.robot.commands.MoveNoteForwardNS;
import frc.robot.commands.PrepareToAmp;
import frc.robot.commands.PrepareToFerry;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.PrepareToShootClose;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.AmpAddOn.AmpPivotToIntake;
import frc.robot.commands.AmpAddOn.AmpPivotUp;
import frc.robot.commands.BarrelPivot.PivotUpwards;
import frc.robot.commands.BarrelPivot.PivotDownwards;
import frc.robot.commands.BarrelPivot.PivotDOWNDOWNDOWN;
import frc.robot.commands.BarrelPivot.PivotToSpeaker;
import frc.robot.commands.Drive.DriveToPose;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private static OI m_oi;

  private static XboxController m_DriverXboxController;
  private static XboxController m_OperatorXboxController;

  private SwerveDriveInputs m_driveInputs;

  /**
   * Used outside of the OI class to return an instance of the class.
   * @return Returns instance of OI class formed from constructor.
   */
  public static OI getInstance() {
    if (m_oi == null) {
      m_oi = new OI();
    }
    return m_oi;
  }

  public OI() {
    // User Input
    // TODO: Tune deadband
    m_DriverXboxController = new XboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER);
    m_OperatorXboxController = new XboxController(RobotMap.U_OPERATOR_XBOX_CONTROLLER);

    // Set up drive translation and rotation inputs
    XboxController driveController = m_DriverXboxController;
    Supplier<Double> xInput;
    Supplier<Double> yInput;
    Supplier<Double> rotInput;
    if(RobotBase.isReal()){
      if (Constants.INPUT_MODE == "DriveSim") {
        xInput = ()->(!m_DriverXboxController.getAButton() ? (1.0-(driveController.getLeftY()*0.5+0.5))-(1.0-(driveController.getRightX()*0.5+0.5)) : 0.0);
        yInput = ()->(!m_DriverXboxController.getAButton() ? 0.0 : (1.0-(driveController.getLeftY()*0.5+0.5))-(1.0-(driveController.getRightX()*0.5+0.5)));
        rotInput = ()->(driveController.getLeftX());
      } else if (Constants.INPUT_MODE == "DemoControllerSolo") {
        xInput = ()->driveController.getLeftY();
        yInput = ()->driveController.getLeftX();
        rotInput = ()->driveController.getRightX();
      } else {
        xInput = ()->-driveController.getLeftY();
        yInput = ()->driveController.getLeftX();
        rotInput = ()->driveController.getRightX();
      }
    } else {
      xInput = ()->-driveController.getLeftX();
      yInput = ()->driveController.getLeftY();
      rotInput = ()->driveController.getRightX();
    }
    m_driveInputs = new SwerveDriveInputs(xInput, yInput, rotInput);
  }

  public void bindControls() {
    if (Constants.INPUT_MODE == "DriveSim") {
      new Trigger(()->{return (m_DriverXboxController.getRightTriggerAxis() > 0.5);}).whileTrue(new GroundIntake());
      new Trigger(()->{return (m_DriverXboxController.getLeftTriggerAxis() > 0.5);}).whileTrue(new SpinUpShooter());
      new JoystickButton(m_DriverXboxController, Button.kRightBumper.value).whileTrue(new PivotUpwards());
      new JoystickButton(m_DriverXboxController, Button.kLeftBumper.value).whileTrue(new PivotDownwards());
      new JoystickButton(m_DriverXboxController, Button.kB.value).whileTrue(new MoveNoteBackward());
      new JoystickButton(m_DriverXboxController, Button.kY.value).whileTrue(new MoveNoteForwardNS());
      new JoystickButton(m_DriverXboxController, Button.kStart.value).onTrue(new InstantCommand(()->Drive.getInstance().zeroHeading()));
    } else if (Constants.INPUT_MODE == "DemoControllerSolo") {
      new Trigger(()->{return (m_DriverXboxController.getRightTriggerAxis() > 0.5);}).whileTrue(new PivotUpwards());
      new Trigger(()->{return (m_DriverXboxController.getLeftTriggerAxis() > 0.5);}).whileTrue(new PivotDownwards());
      new JoystickButton(m_DriverXboxController, Button.kRightBumper.value).whileTrue(new GroundIntake());
      new JoystickButton(m_DriverXboxController, Button.kLeftBumper.value).whileTrue(new SpinUpShooter());
      new JoystickButton(m_DriverXboxController, Button.kB.value).whileTrue(new MoveNoteBackward());
      new JoystickButton(m_DriverXboxController, Button.kY.value).whileTrue(new MoveNoteForwardNS());
      new JoystickButton(m_DriverXboxController, Button.kBack.value).onTrue(new InstantCommand(()->Drive.getInstance().zeroHeading()));
    } else {
      ////////////////////////////////////////////////////
      // Now Mapping Commands to XBoxz
      ////////////////////////////////////////////////////

      // Driver Grease Man's Special Abilities(OP)
      new JoystickButton(m_DriverXboxController, Button.kRightBumper.value).whileTrue(new PrepareToShoot());
      new JoystickButton(m_DriverXboxController, Button.kLeftBumper.value).whileTrue(new PrepareToFerry());
      new Trigger(()->{return (m_DriverXboxController.getRightTriggerAxis() > 0.5);}).whileTrue(new PivotToSpeaker());
      new Trigger(()->{return (m_DriverXboxController.getRightTriggerAxis() > 0.5);}).whileTrue(new AmpPivotUp());
      new JoystickButton(m_DriverXboxController, Button.kBack.value).onTrue(new InstantCommand(()->Drive.getInstance().zeroHeading()));

      new Trigger(()->{return (m_DriverXboxController.getLeftTriggerAxis() > 0.5);}).whileTrue(new SpinUpShooter());
      
      
      // Drive to locations on the field
      new JoystickButton(m_DriverXboxController, Button.kA.value).whileTrue(new DriveToPose(FieldUtils.getInstance()::getAmpScorePose));
      // new JoystickButton(m_DriverXboxController, Button.kX.value).whileTrue(new DriveToPose(FieldUtils.getInstance()::getSource1Pose));
      // new JoystickButton(m_DriverXboxController, Button.kB.value).whileTrue(new DriveToPose(FieldUtils.getInstance()::getSource3Pose));
      // new JoystickButton(m_DriverXboxController, Button.kY.value).whileTrue(new DriveToPose(FieldUtils.getInstance()::getSpeakerScorePose));

      // Operator Cookie Monster Special Abilities(MEGA OP)
      new JoystickButton(m_OperatorXboxController, Button.kB.value).whileTrue(new SourceIntake());
      new JoystickButton(m_OperatorXboxController, Button.kA.value).whileTrue(new MoveNoteForward());
      new JoystickButton(m_OperatorXboxController, Button.kX.value).whileTrue(new GroundIntake());
      new JoystickButton(m_OperatorXboxController, Button.kY.value).whileTrue(new MoveNoteBackward());

      new JoystickButton(m_OperatorXboxController, Button.kBack.value).whileTrue(new AmpPivotToIntake());
      new JoystickButton(m_OperatorXboxController, Button.kStart.value).whileTrue(new PivotDOWNDOWNDOWN());
      new JoystickButton(m_OperatorXboxController, Button.kStart.value).whileTrue(new AmpPivotToIntake());

      new JoystickButton(m_OperatorXboxController, Button.kRightBumper.value).whileTrue(new PrepareToShootClose());
      new Trigger(()->{return (m_OperatorXboxController.getRightTriggerAxis() > 0.5);}).whileTrue(new ShootSpeaker());

      new JoystickButton(m_OperatorXboxController, Button.kLeftBumper.value).whileTrue(new PrepareToAmp());
      new Trigger(()->{return (m_OperatorXboxController.getLeftTriggerAxis() > 0.5);}).whileTrue(new ScoreAmp());

      new Trigger(m_OperatorXboxController.povDown(CommandScheduler.getInstance().getDefaultButtonLoop())).onTrue(new InstantCommand(()->{
        TDNumber speakerHeightOffset = new TDNumber(BarrelPivot.getInstance(), "Auto Pivot", "Speaker Height Offset (meters)");
        speakerHeightOffset.set(speakerHeightOffset.get() + Constants.SPEAKER_ADJUSTMENT_INCREMENT_M);
      }));

      new Trigger(m_OperatorXboxController.povUp(CommandScheduler.getInstance().getDefaultButtonLoop())).onTrue(new InstantCommand(()->{
        TDNumber speakerHeightOffset = new TDNumber(BarrelPivot.getInstance(), "Auto Pivot", "Speaker Height Offset (meters)");
        speakerHeightOffset.set(speakerHeightOffset.get() - Constants.SPEAKER_ADJUSTMENT_INCREMENT_M);
      }));
    }
  }

  /**
   * Returns the Xbox Controller
   * @return the Xbox Controller
   */
  public XboxController getDriverXboxController() {
      return m_DriverXboxController;
  }

  public XboxController getOperatorXboxController() {
    return m_OperatorXboxController;
  }

  public SwerveDriveInputs getDriveInputs() {
    return m_driveInputs;
  }
}
