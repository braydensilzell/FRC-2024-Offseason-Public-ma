package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
    private ShooterSubsystem shooter;
    //private Boolean auto;
    private int x;
    


  /** Creates a new Shoot. */
  public Shoot(ShooterSubsystem shooter) {

    this.shooter = shooter;
    //this.auto = auto;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int x =0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putNumber("TargetAngle", SpeakerShotRegression.calculateWristAngle(targetDistance).getDegrees());
    // SmartDashboard.putNumber("TargetDistance", targetDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    x=0;

  }

  // Returns true when the command should end.
}