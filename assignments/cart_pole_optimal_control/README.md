# Cart-Pole Optimal Control Assignment

the cart-pole optimal control assignment revolves around a inverted pendulum attached to a cart with a earthquake simulator that shakes the ground constantly. the state of the cart is controlled by the carts x position its velocity and the angle theta of the inverted pendulum the cart and pole are affected by a normal gravity of 9.8 m/s.

### LQR controller

the balance of the cart pole is controlled by adjusting the gain of the controller using an LQR controller (linear quadratic controller). the control matrix has 2 cost matrices Q with the cost function for the states [x,x_dot ,Theta,Theta_dot] and the other R matrix which governs the cost function of the effort of the actuator. the higher the cost the more important that that parameter should be in the optimal section. having it all high would be the assumption but not all states are more important and having a higher cost function for another state would cause the system to collapse. the effect the cost function R is directly related to what the actuator is able to produce and having too low of a cost function sets up and impossible standard for the actuator


### Experiments

The system was run for 2 minutes on average with 7 different Q and R matrices and the maximum deviation between them was noted in the each of the runs.

- Q = [100 10 100 100] R = [0.5]

 ![100 10 100 100](https://github.com/user-attachments/assets/271ad21e-df58-4a0a-a913-834a22c63a17)

this run was setup to see how having extreme amount in the cost functions affected the values in the system , the system was stable in the 5 minutes it ran but had a higher deviation in the angle and the position of the system with a maximum angle deviation of = 0.8 and a maximum position deviation = 0.11 the R was setup to have 0.5 cost for the actuator effort

- Q = [10 1 10 1] R = [0.5] 

this run was setup was to see how an system with a more forgiving cost function for the actuator but less cost function would work but the entire system fell unstable and would tip over within the 2 minutes

- Q = [10 1 10 10] R = [0.1]

 ![10 1 10 10 0 1](https://github.com/user-attachments/assets/11d18189-0610-4c06-b51a-cd5597fc9920)

this run was setup trying to limit both the angular velocity and the position and velocity of the system. this system was unstable  due to not allowing the system to achieve higher angular velocities the system had to compensate by increasing the linear velocity moving the cart beyond the deviation and eventually falling over. this system however lasted longer than the default values provided in the assignment

- Q = [10 1 10 1] R = [0.1] time = 5 mins

![10 1 10 1 0 1 2](https://github.com/user-attachments/assets/36b4aff5-e63c-44e4-aeac-956f3bcb3e9b)

this run was setup to prove my above assumption of allowing the cart to move more faster with allowing to achieve a higher angular velocity would provide a more stable system. the system was stable allowing the system to produce a higher angular velocity and linear velocity it had a easier time balancing itself. the system was allowed to run for 5 minutes and the maximum angle deviation = 0.06 and maximum position deviation = 0.21

- Q = [10 10 1 1] R = [0.1]

![10 10 1 1 0 1](https://github.com/user-attachments/assets/29a671a4-7f94-462f-a3cd-757daeb1df6a)

this run was setup to see how having a higher linear velocity cost function would affect the system the system was surprisingly able to remain stable despite having a cost function lower for the maximum angle deviation the system predictably had a higher maximum angle deviation and less position deviation. the system was allowed to run for 2 minutes and the maximum angle deviation = 0.11 and maximum position deviation = 0.14

- Q = [10 1 10 1] R = [0.1] time = 2mins

![10 1 10 1 0 1 ](https://github.com/user-attachments/assets/4ecf1723-b356-4946-bf13-0d8e262efb8d)

the run was setup to see if as time went on the system deviations would increase which was proved right as by the above , to make sure the system didn't eventually fall after sometime a similar setup was left for 15 minutes with no signs of increasing instability noting that the increasing deviations was caused by the earthquake simulator achieving conditions that sometime deviated the results

- Q = [10 1 10 1] R =[0.01]

![10 1 10 1 0 01](https://github.com/user-attachments/assets/07c2274d-5754-4edb-9784-62cb7a1e86da)


this particular run was setup to test that by allowing the actuator effort to be allowed to produce any force to keep it balanced allowed better balancing with the maximum angle deviation = 0.05 and maximum position deviation = 0.08.


### Conclusion:

the LQR controller is highly dependant on the actuator effort cost function as setting a value low enough allows higher control. but this particular value is affected by the actual ability of the physical actuator itself meaning a suitable value for the actuator effort must be found through physical means. the cost functions for the different states are best to be studied through the actual requirement of the system as the control is somewhat intuitive.


