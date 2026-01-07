// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  Drivetrain drive;

  Joystick left_joystick;
  Joystick right_joystick;

  SlewRateLimiter x_limiter = new SlewRateLimiter(3);
  SlewRateLimiter y_limiter = new SlewRateLimiter(3);
  SlewRateLimiter rotation_limiter = new SlewRateLimiter(3);
  

  public Drive(Drivetrain drive_imported, Joystick left_joystick_imported, Joystick right_joystick_imported) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive_imported;
    this.left_joystick = left_joystick_imported;
    this.right_joystick = right_joystick_imported;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = x_limiter.calculate(MathUtil.applyDeadband(this.left_joystick.getX(), 0.1));
    double y = -y_limiter.calculate(MathUtil.applyDeadband(this.left_joystick.getY(), 0.1));
    double rotation = rotation_limiter.calculate(MathUtil.applyDeadband(this.right_joystick.getX(), 0.1))*Constants.Drivetrain.max_angular_speed;

    Translation2d translation = new Translation2d(x,y).times(Constants.Drivetrain.max_speed);

    this.drive.drive(translation, rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
