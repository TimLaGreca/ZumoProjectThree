# ZumoProjectThree
Project Task: You've done a lot this semester with your robot. You've modeled its behavior and controlled its distance traveled using open-loop control and a finite state machine. You've designed a "dead reckoning" waypoint navigation system using a combination of several feedback controllers and a finite state machine. Now, it's time to add one more piece of functionality. It is our hope that you come out of the semester with a nearly autonomous robot you can be proud of, so we'd like you to add the ability to follow a line to your robot's growing repertoire of antics. 
 
For project 3, your robot will race around an autocross-style track with a distinct start and finish point. Your score will be based on the time it takes your robot to complete the course, with time penalties for large deviations (over about 2cm from track center). Some basic properties of the task are listed below:  

1) The track will be a black, 3.5" wide stripe on white paper.
2) The track will consist of straight and constant-radius segments.
3) The minimum turn centerline radius will be 5 inches, and the minimum length of any straight segment will be 8 inches.
4) There will be a small "step" in the track's lateral position on a straight section, with at least 8" of straight track following the step before the next turn.
5) There will be an "obstacle" on one of the longer straights, which your robot will need to navigate around using encoders and the gyro as in project 2. The obstacle will be no larger than a Zumo box.
