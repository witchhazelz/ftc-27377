package org.firstinspires.ftc.teamcode.opmode;

public class SkeletonMainTeleOp {
    // The teleop contains the main structure of the program being run on the robot
    /* objectives/ goals that the program needs to fullfill:
    >>>>>>assuming starting position has the slide locked perpundicular to the floor, and teh claw should
    be facing down<<<<<<<

    * Score sample samples in baskets(low and high):
        * Pick up samples from a set position on the ground:
              * Open the claw
              > Close claw
              > extend the slides to a set position(extended1)
              * Possibly:
              * rotate the wrist

           * Pick up samples form inside the chamber
           to get inside the chamber:
                > drop pitch to a certain angle
                > Open claw
                * make sure the wrist is in the initial position
                wiat a couple seconds
                > extend the slides horizontially(extend1)

            to pick and leave the chamber:
                > close claw
                then
                > retract the slide
                then
                move the drive train back
           * Score the samples in the baskets(high and low):
                > Rotate sprocket to set the slide vertically raise(initial psoition)
                >  raise slide to the height of the basket(high or low)"find these values"
                > rotate wrist so the sample falls from between the claws arms


                > open claw to allow for sample drop in
                reset to collection/initial position


    * Score specimens on chambers:
           * Pick up samples from a set position on the ground:
              > Open the claw
              > Close claw
              * Possibly:
              * rotate the wrist and/or extend slides
           * Pick up samples form inside the chamber
                > Open claw
                > extend the slides horizontially
                > rotate the wrist for better grabbing
                > close claw
                > retract the slide
           * Retrieve specimens from specified zones
                > same code as for from inside the chamber
                *Possibly:
                * adjust slide extending

           * score the specimens on the chambers(high and low):
                > Rotate sprocket to vertical position
                > move to be in front of chamber
                > raise slide to the height of the bar(high and low) "find these values"
                > rotate the wrist so the claw's arms are on the right and left of the sample( DETAILS TO BE DECIDED)
                > lower slides to pull the specimens down
                > open claw

    * Hang minimum level 2 ascent: TO BE DECIDED
     */
}
