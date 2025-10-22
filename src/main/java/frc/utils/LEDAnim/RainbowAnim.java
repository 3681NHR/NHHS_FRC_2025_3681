package frc.utils.LEDAnim;

import edu.wpi.first.wpilibj.util.Color;

public class RainbowAnim extends LEDAnim {

    /**
     * Rainbow animation
     * @param length length of LED strip
     * @param speed speed of rainbow, 1 is full cycle per second
     */
    public RainbowAnim(int length){
        super(length);

        for(int i=0; i<leds.length; i++){
            int hue = ((i * 180) / leds.length) % 180;
            leds[i] = Color.fromHSV(hue, 255, 255);
        }
        initLeds = leds.clone();
    }
    
}
