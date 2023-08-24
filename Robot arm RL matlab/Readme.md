Find file with name BraccioWithMotorsAndPIDandRL2020 (This file is openned in matlab 2020a)
and find the original file BraccioWithMotorsAndPIDandRL (This file needed matlab2023)

the file can simply be run with no pervious setup as I already removed the visualization part but if having any errors as running
you need to make sure that the solver is fixed-step and solver is ode1be(Backward Euler) with fixed-step size of 0.01 (you can find this in Modeling -> Model Setting)
Use Accelerator run for faster results

you can find the functions I used in the code also as RLNoLoop.m and RLwithLoop I used both to evalute the performance of increasing the parameters in this case. you can check them both are well commented and splitted

if you have any problem running the simulink you can contact me at ielsh091@uottawa.ca