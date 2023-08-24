function u = RLNoLoop (error)
%flag to the first run to put intial conditions
%allocates memory for it only once during the first call to the function
%use isempty in first use as it asiigned only place in memory
%without out any value so it is just empty place
persistent flag
persistent Wc
persistent Wa
persistent Eold
persistent uold

%Intialize now all needed
%In if block to ensure it's set only once
if isempty(flag)
    %First Wa make intial guess (1*1)
    P=0.026;

    Wa= P;
    
    %First Wc make initial guess (2*2)
    Wc=[1,     -P;
        -P,  1];
    
    %the old output response
    uold = 0;

    %the old error will be set for the first time but later we get it
    Eold = 0;

    %Make the flag any value to leave the loop
    %by initializing a Persistent Variable
    flag = 5;
end
%Things to define again instead of persistent as it no updated
    %assign values for alpha to be used later in Wa and Wc
    alpha_A=0.006;
    alpha_C=0.0004;
    %Let the Q to be 0.7
    Q=0.4;

    %random value for R and i may make it something small like 0.01
    % R = rand(1, 1); DOnt use random to have same performance
    R = 0.08;

%Things Change Every time
Z = [error; uold];
Zold = [Eold; uold];

%RL
%Q next for k+1
Qnext = 0.5 * Z' * Wc * Z;

%Cost function
Cost = Eold' * Q * Eold + uold' * R * uold;

%Qdesire
Qdesire = Cost + Qnext;

%Qk approximate = 0.5 * Zk transpose * Wc * Zk
%Zk is [Eold(1*1) ; u(1*1)]
%Mu_K is the old output lets call it uold
%Xk = [Old error (E_old); ]
Qapp = 0.5 * Zold' * Wc * Zold;

%Wc (Tuning law)
Wc = Wc - alpha_C * (Qapp - Qdesire) * (Zold*Zold');

%
Wcu = -Wc(2,2) * Wa;

%u_desire
u_desire = - inv(Wc(2,2)) * Wcu * error;

%estimate the u_est
%(1*1) * (1*1)
u_est = Wa * error;

%Wa
Wa = Wa - alpha_A * (u_est - u_desire) * (Eold');

%Calculate the new action u
u = uold + Wa * error;

%Assign the input error to Eold (error = desired - feedback) to be later
%used in next respond
Eold = error;

end