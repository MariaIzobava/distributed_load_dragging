## Meeting with Valerio

1. What are these motion capture things? 

the issue: radio connection is slow. can't run solver on board. MPC normally is not delay compensation. one thing: adding very slow mpc + something that is much faster old solution + 

send control action from mpc, then update 

what is the sending time of the drone. If it's too slow then add receiver on the drone. 

need to find a delay on the crazyflie channel then we can consider improving hardware 

mpc on crazyflie 



## Meeting about introduction report
1. uncrued aerial vehicles
2. impact highlevel, the things I'm doing how it can change industry.
3. chapter not hardcoded
4. formulation: for vector - slamm letter, so X small but bold
5. Jacobians are good, keep it
6. if 2 robots centralised works, experiments should be done, even without orientation. 

7. this week work orientation, next week is demo!



# 01.08.2025

1. applications of my work, snesing something on a particular height


# 08.08.2025

1. Things I've tried so far:
    * substituting tether penalty factor to tether tension factor
    * running with both analytical and numerical derivatives for dynamics
    * having trajectory reference factors for each point and for the last point
    * tried to use Marginals object to print out and examine jacobians
        * on the approaches which was working the Marginals returned "indetermined system" error
    * tried adding priors on each control variable
    * tried penalyzing big differences between subsequent controls with BetweenFactor

2. things I want yet to try:
    * return the next trajectory point not based on time but based on the nearest trajectory point
    * limit the velocity of the drones to [-0.2, 0.2] in the factor graph
    * try exhastive search
    * try position only

3. notes

* move the beginning of problem statement up
* be more specific in problem statement, e.g. robots are coordinated, follow a trajectory for load while roobts don't collide and energy optimised.
* capped letters in the Headers
* tilda before citations 
* remove CADMM if I don't have time to apply or add itto fuutre work
* Important next steps:CADMM and plan real world experiment
* centralised in real world is bigger value than CADMM
* fix position -> report -> real experiment -> CADMM -> rotation
 

 # Phase Space

1. Go to: psclient-3_0_20230317_ubuntu_20_04/bin_linux
2. run: ./master.sh
3. click connect
4. another terminal: got to phasespace workspace
5. run ros2 run phasespace_client psnode
6. positions of the drones will be on the topic: /phasespace_rigids