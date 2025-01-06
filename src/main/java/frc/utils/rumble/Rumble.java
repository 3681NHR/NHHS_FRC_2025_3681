package frc.utils.rumble;

public class Rumble implements RumbleBase{
    private double duration = 0.0;
    private double strength = 0.0;
    public Rumble(double time, double strength){
        this.strength = strength;
        this.duration = time;
    }
    @Override
    public void subtractTime(double time) {
        duration -= time;
    }
    @Override
    public double getTime() {
        return duration;
    }
    @Override
    public void setTime(double time) {
        duration = time;
    }
    @Override
    public double getStrength() {
        return strength;
    }
    @Override
    public void update(){

    }
}