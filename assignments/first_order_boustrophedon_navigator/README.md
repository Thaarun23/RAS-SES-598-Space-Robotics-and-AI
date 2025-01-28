# Assignment 1:First-Order Boustrophedon Navigator


This assignments goal was to obtain the optimal PID values for the Boustrophedon navigator also known as the lawnmower pattern in turtle sim acquiring the correct kp and kd values for linear and angular motion. the assignment encompassed obtaining the correct motion, pattern and efficiency in coverage of space 
and the cross track error which shows how much the path deviates from the actual path its supposed to take
![My result](https://github.com/Thaarun23/RAS-SES-598-Space-Robotics-and-AI/blob/0da165639a14ef1dfe5174a17dbc155dedceddab/Lawnmower%20pattern%20Thaarun.png)
## Tuning Methodology and Results

the tuning methodology used was a heuristic approach of trial and error. by setting the Kp_linear and Kp_angular values to the extreme to see how it affects the system. this allows us to learn the behaviour of the robot and how it affects the pattern in the end.and from the above experiment its showed that

- Kp_linear and Kd_linear - controlled the overshooting of the model near the turns which when improperly tuned causes the course correction to turn above the line it turned from and then proceeded to cross itself. Kd_linear controlled the slowing of the system as it approached the turning point this allowed the Kp_angular to make sharper turns but not too sharp 

- Kp_angular and Kd_angular - Kp_angular and Kd_angular changes to these values showed that the higher the value of Kp_angular the sharper the turn this was the primary requirement to change for the pattern as it not only controlled the tightness of the spacing but also the straightness of the path with which it approached the next waypoints.

after which the parameters were returned to more suitable values and initially tested with a spacing of 0.5. through knowing the behaviours of the system with respect to the parameters , deducing that the system needed to make sharper turns the PID values were adjusted with the above in mind and the values obtained were
the spacing was decreased to allow the robot to cover more ground and an interesting observation was the lower the spacing the higher the Kp_angular value
-  Kp_linear - 4.0
-  Kd_linear - 0.03
-  Kp_angular - 8.0
-  Kd_angular - 0.01
-  spacing-0.5


## Performance Metrics:

The system produced different average values in each run but close in values
- average cross-track error perfomance was less than <0.2 with the values ranging from 0.106-0.112
- the maximum cross-track error was 0.233

was unable to obtain the plots due to some random and unexpected error with topics not publishing the raw data and only the message that the publishers published which could not be read by the rqt_plots

