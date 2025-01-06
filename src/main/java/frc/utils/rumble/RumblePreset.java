package frc.utils.rumble;

public enum RumblePreset {
    /**
     * single pulse
    */
    TAP,
    /**
     * two taps
     */
    DOUBLE_TAP,
    /**
     * simaler to a tap, but longer
     */
    RING;

    public RumbleSequence load(){
        RumblePreset rumble = this;
        switch (rumble) {
            case TAP:
                return new RumbleSequence(new Rumble[]{
                    new Rumble(0.2, 1),
                });
            case RING:
                return new RumbleSequence(new Rumble[]{
                    new Rumble(0.5, 0.75),
                });
            case DOUBLE_TAP:
                return new RumbleSequence(new Rumble[]{
                    new Rumble(0.2, 1),
                    new Rumble(0.1, 0),
                    new Rumble(0.2, 1),
                });
        
            default:
                return new RumbleSequence(new Rumble[]{});
        }
    }
}
