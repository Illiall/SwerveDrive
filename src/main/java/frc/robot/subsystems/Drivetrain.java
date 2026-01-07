// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  PigeonIMU Gyro;

  SwerveDrivePoseEstimator PoseEstimator;

  RobotConfig config;

  PPHolonomicDriveController Controller;

  PIDConstants translationConstants;

  PIDConstants rotationConstants;

  private SwerveModule[] drive;

  public Drivetrain() {

    Gyro = new PigeonIMU(10);

    Gyro.configFactoryDefault();

    SetYaw(0);

    this.drive = new SwerveModule[] {
      new SwerveModule(0, Constants.Drivetrain.SwerveModule0.turn_id, Constants.Drivetrain.SwerveModule0.drive_id, Constants.Drivetrain.SwerveModule0.CAN_coder_id, Constants.Drivetrain.SwerveModule0.turn_offset),
      new SwerveModule(1, Constants.Drivetrain.SwerveModule1.turn_id, Constants.Drivetrain.SwerveModule1.drive_id, Constants.Drivetrain.SwerveModule1.CAN_coder_id, Constants.Drivetrain.SwerveModule1.turn_offset),
      new SwerveModule(2, Constants.Drivetrain.SwerveModule2.turn_id, Constants.Drivetrain.SwerveModule2.drive_id, Constants.Drivetrain.SwerveModule2.CAN_coder_id, Constants.Drivetrain.SwerveModule2.turn_offset),
      new SwerveModule(3, Constants.Drivetrain.SwerveModule3.turn_id, Constants.Drivetrain.SwerveModule3.drive_id, Constants.Drivetrain.SwerveModule3.CAN_coder_id, Constants.Drivetrain.SwerveModule3.turn_offset)
    };

    //Creates translation constants for the controller 
    translationConstants = new PIDConstants(
      Constants.PathPlanner.translation_kP,
      Constants.PathPlanner.translation_kI,
      Constants.PathPlanner.translation_kD);
    
    //Creates rotation constants for the controller
    rotationConstants = new PIDConstants(
      Constants.PathPlanner.rotation_kP,
      Constants.PathPlanner.rotation_kI,
      Constants.PathPlanner.rotation_kD);
    
    //Creates the Controller to use in the AutoBuilder
    Controller = new PPHolonomicDriveController(translationConstants, rotationConstants);

    //Pose Estimator to estimate robots current position
    PoseEstimator = new SwerveDrivePoseEstimator(
      Constants.Drivetrain.swerve_kinematics, 
      getYaw(),
      new SwerveModulePosition[]{
        this.drive[0].getModulePosition(),
        this.drive[1].getModulePosition(),
        this.drive[2].getModulePosition(),
        this.drive[3].getModulePosition()
      },
      new Pose2d());

      //Tries to recieve information from PathPlanner GUI settings to load into the autobuilder
      try{
        config = RobotConfig.fromGUISettings();
      //If Failed will return an error
      }catch(Exception exception){
        exception.printStackTrace();
      }

      //Configures the AutoBuilder
      AutoBuilder.configure(
        this::get_Pose, 
        this::reset_pose, 
        this::getRobotRelativeSpeed, 
        (speed, feedforward) -> DriveRobotRelative(speed), 
        Controller, 
        config, 
        () -> flipTeam(), 
        this);
  }

  public void SetYaw(double yaw){
    Gyro.setYaw(yaw);
  }

  public void Reset_Gyro(){
    Gyro.setYaw(0);
  }

  public Rotation2d getYaw(){
    return Rotation2d.fromDegrees(Gyro.getYaw());
  }

  public void drive(Translation2d translation, double rotation, boolean isFieldRelative){
    SwerveModuleState[] swerveModuleState = Constants.Drivetrain.swerve_kinematics.toSwerveModuleStates(
      isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()):
      new ChassisSpeeds(translation.getX(), translation.getY(), rotation)); {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState, Constants.Drivetrain.max_speed);
      
      for(SwerveModule module: this.drive){
        module.setDesiredState(swerveModuleState[module.module_number]);
      }
      }
  }

  //Pathplanner Code

  //Function that returns a Estimated Position as a Pose2D variable from the Pose Estimator
  public Pose2d get_Pose(){
      return this.PoseEstimator.getEstimatedPosition();
  }

  //Resets the pose of the Robot
  //Uses current rotation, Module positions, and the current pose
  public void reset_pose(Pose2d pose){
    this.PoseEstimator.resetPosition(getYaw(), swerveModulePositions(), pose);
  }

  //Returns the RelativeSpeed of the Robot using the Swerve_Kinematic to access the module state
  public ChassisSpeeds getRobotRelativeSpeed(){
    return Constants.Drivetrain.swerve_kinematics.toChassisSpeeds(getModuleState());
  }

  //Flips the path's for different team colors
  public boolean flipTeam(){
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      return alliance.get() == DriverStation.Alliance.Red;
    }else{
      return false;
    }
  }

  //Pathplanner Drive
  //Uses speed(ChassisSpeed) to rotate the motors
  public void DriveRobotRelative(ChassisSpeeds speed){
    SwerveModuleState[] ModuleStates = Constants.Drivetrain.swerve_kinematics.toSwerveModuleStates(speed);
    SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, Constants.Drivetrain.max_speed);
    for(SwerveModule module:this.drive){
      module.setDesiredState(ModuleStates[module.module_number]);
    }
  }

  //Returns the SwerveModulePositions
  public SwerveModulePosition[] swerveModulePositions(){
    return new SwerveModulePosition[] {
      this.drive[0].getModulePosition(),
      this.drive[1].getModulePosition(),
      this.drive[2].getModulePosition(),
      this.drive[3].getModulePosition()
    };
  }

  //Returns the SwerveModuleState 
  public SwerveModuleState[] getModuleState(){
    return new SwerveModuleState[]{
      this.drive[0].getState(),
      this.drive[1].getState(),
      this.drive[2].getState(),
      this.drive[3].getState()};
  }

  @Override
  public void periodic() {
    //Updates the Pose Estimator
    PoseEstimator.update(
      getYaw(), 
      new SwerveModulePosition[]{
        this.drive[0].getModulePosition(),
        this.drive[1].getModulePosition(),
        this.drive[2].getModulePosition(),
        this.drive[3].getModulePosition()
      });
  }
}
