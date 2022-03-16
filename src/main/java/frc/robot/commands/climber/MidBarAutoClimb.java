package frc.robot.commands.climber;

public class MidBarAutoClimb {
    private final HookClimberSubsystem climber;
    
    public MidBarAutoClimb(HookClimberSubsystem climber, BooleanSupplier overrideButtonIsPressed) {
        this.climber = climber;

        addRequirements(this.climber);

        this.addCommands(
            new ExtendClimberCommand(climber, () -> false).withInterrupt(climber::getIsAtMaxHeight),
            new RetractClimberCommand(climber, () -> true, 0.8).withInterrupt(() -> climber.getEncoderPosition() <= -2000)
        );
}
