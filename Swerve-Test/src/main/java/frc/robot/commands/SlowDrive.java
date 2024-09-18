package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Dimensoes;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class SlowDrive extends Command {
     private final Swerve swerve;
     private final DoubleSupplier vX, vY, heading;
     private boolean initRotation = false;
     private SwerveController controller;
     // private int velocity = 0;

     CommandXboxController control = new CommandXboxController(0);
     // private double speed;

     public SlowDrive(Swerve swerve, DoubleSupplier vY, DoubleSupplier vX, DoubleSupplier heading) {
          this.swerve = swerve;
          this.vX = vX;
          this.vY = vY;
          this.heading = heading;
          // this.velocity = velocity;
          // this.control = control;
          controller = swerve.getSwerveController();

          addRequirements(swerve);
     }

     @Override
     public void initialize() {
          initRotation = true;
     }

     @Override
     public void execute() {
          double angle = heading.getAsDouble() * 0.8;

          ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), 1.5);

          // Limit velocity to prevent tippy
          Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
          translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                    Dimensoes.LOOP_TIME, Dimensoes.ROBOT_MASS, List.of(Dimensoes.CHASSIS),
                    swerve.getSwerveDriveConfiguration());
          SmartDashboard.putNumber("LimitedTranslation", translation.getX());
          SmartDashboard.putString("Translation", translation.toString());
          double rotation = controller.config.maxAngularVelocity * angle;

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
