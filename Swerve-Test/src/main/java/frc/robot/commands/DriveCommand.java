package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Dimensoes;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class DriveCommand extends Command {
     private final Swerve swerve;
     private final DoubleSupplier vX, vY, heading;
     private boolean initRotation = false;
     private SwerveController controller;
     double velocity;
     private PIDController pid = new PIDController(1.5, 0, 0);

     XboxController control;

     public DriveCommand(Swerve swerve, DoubleSupplier vY, DoubleSupplier vX, DoubleSupplier heading, XboxController control) {
          this.swerve = swerve;
          this.vX = vX;
          this.vY = vY;
          this.heading = heading;
          this.control = control;
          controller = swerve.getSwerveController();

          addRequirements(swerve);
     }

     @Override
     public void initialize() {
          initRotation = true;
     }

     private double setMax() {
          if (control.getRawButton(1)) {
               return velocity = 1.5;
          } else {
               return velocity = 4.0;
          }
     }

     private double getLock() {
          if (control.getBButtonPressed()) {
               return swerve.getHeading().getRadians();
          }

          return 0.0;
     }

     @Override
     public void execute() {
          double angle = heading.getAsDouble() * 0.8;

          ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), setMax());
          Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
          translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                    Dimensoes.LOOP_TIME, Dimensoes.ROBOT_MASS, List.of(Dimensoes.CHASSIS),
                    swerve.getSwerveDriveConfiguration());
          SmartDashboard.putNumber("LimitedTranslation", translation.getX());
          SmartDashboard.putString("Translation", translation.toString());
          SmartDashboard.putNumber("VelocityMax", setMax());
          double rotation = controller.config.maxAngularVelocity * angle;

          double output = pid.calculate(swerve.getHeading().getRadians(), getLock());
          if (control.getLeftTriggerAxis() != 0.3) {
               swerve.drive(translation, output);
          }

          swerve.drive(translation, rotation);

     }

     @Override
     public void end(boolean interrupted) {
     }

     @Override
     public boolean isFinished() {
          return false;
     }

}
