# time capsule
this document is meant to save as much info compiled over time by members of the nathan hale robotics programming subteam as possible. the main reson of this is to provide
 - a referance of knowledge for current teams encountering problems
 - information to new members

## preface
 - all links in text are also in 3. links
 - feel free to add any challenging problems and the solutions you found in the referance page **this file is meant to be edited**
 - dont remove something without a good reason
 - try to avoid adding things to referance that can be solved with a google search
 - version-specific info should include a version(ex. the dashboard wont work with negitive numbers in **v2024.1.5**)
## 1. referance
 - ### YAGSL
    - https://yagsl.gitbook.io/
    - not offloading offsets to encoders causes some problems, idk how, but i think its internal. seen in YAGSL v2024.7.0
    - loctite the magnets on swerve modules. this is very importent
 - ### photonvision
    - can connect to simulator if on the same network and the computer running the sim is named "RoboRIO-3681-frc", team must be 3681 in camera settings
    - calibration requires as much data as posible in a wide range. make sure to get images with points close to the edge of view, and in varius camera positions and angles
    - #### apriltags
       - 16h5: larger grid. larger range, more false positives, ~32 ids
       - 36h11: smaller grid. less range, less false positives. ~256 ids

 - complex and abstract systems/lockouts need an override in case of failure(ex. a lockout that prevents launching something until the launcher is up to speed needs a button or toggle in dashboard that allows launching whenever. otherwise if an encoder fails, you *cant* score)
 - build failing with error "cant find symbol" and filenames, class names, packages, imports, and autocorrect are right. deleting .class files under build directory might fix
 - ### CamelCase
   - variables, methods, and package names are lowercase first letter. first letter of following words are uppercase
   - class names are uppercase first letter. first letter of following words are uppercase
   - constants are all caps with underscore seperating words
   #### examples
      - class ArmSubsystem
      - variable angleTarget
      - constant MAX_SPEED
## 2. teachings

## 3. links
 - https://yagsl.gitbook.io/

 ------ 
### contributing members
  - Evan Orchard (2024-2027)