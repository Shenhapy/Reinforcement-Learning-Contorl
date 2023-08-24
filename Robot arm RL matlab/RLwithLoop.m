function u = RLwithLoop (error)
%flag to the first run to put intial conditions
%allocates memory for it only once during the first call to the function
%use isempty in first use as it asiigned only place in memory
%without out any value so it is just empty place
persistent Flag
persistent Wa
persistent Wc
persistent Eold
persistent Eolder
persistent Uold
persistent wcuu
persistent wcue
persistent uPervious

%Intialize now all needed
%In if block to ensure it's set only once
if isempty(Flag)
    %Make the flag any value to leave the loop
    %by initializing a Persistent Variable
    Flag=5;

    %the old error will be set for the first time but later we get it
    Eold=0;
    Eolder=0;
    
    %First Wa make intial guess (2*2)
    P=0.073;
    wa1=P;
    wa2=-P;
    Wa=[wa1,wa2];

    %the old output response
    Uold=0;
    uPervious=0;

    %First Wc make initial guess (3*3)
    wcuu=1;
    wcue=-wcuu*Wa;
    Wc=[1,0,wcue(1);
        0,1,wcue(2);
        wcue(1),wcue(2),wcuu];  %(3*3)

    wcue=Wc(3,1:2);
    wcuu=Wc(3,3);

end

%Things to define again instead of persistent as it no updated
%assign values for alpha to be used later in Wa and Wc
%random value for R and i may make it something small like 0.01
% R = rand(1, 1); DOnt use random to have same performance
R=0.01;
Q=eye(2);
alphaC=0.000009;
alphaA=0.00000055;

%Things Change Every time
Eo=[Eold;Eolder];   %(2*1)
Er=[error;Eold];%(2*1)


r = 200;
while (r>0)           
    %Cost function
    cost=0.5*(Eo'*Q*Eo+Uold'*R*Uold); 
    Uest=-pinv(wcuu)*wcue*Er;
    Zk1=[error;Eold;Uest];
    Zold=[Eold;Eolder;Uold];

    %Qk approximate = 0.5 * Zk transpose * Wc * Zk
    %Zk is [Eold(1*1) ; u(1*1)]
    %Mu_K is the old output lets call it uold
    %Xk = [Old error (E_old); ]

    Qapp=0.5*Zold'*Wc*Zold;
    %Q next for k+1
    Qnext=0.5*Zk1'*Wc*Zk1;
    %Qdesire
    Qdes=cost+Qnext;

    %Wc (Tuning law)
    Wc=Wc-alphaC*(Qapp-Qdes)*(Zold*Zold');
    wcue=Wc(3,1:2);
    wcuu=Wc(3,3);

    %estimate the u_est
    Uapp=Wa*Eo;
    %u_desire
    Udes=-pinv(wcuu)*wcue*Eo;
    %Wa
    Wa=Wa-alphaA*(Uapp-Udes)*(Eo');
 
    %Calculate the new action u for the loop
    Uold=Wa*Er;

    if (abs(Uapp-Udes)>0.001)
        break;
    end

    r=r-1;
end 

%Assign the input error to Eold (error = desired - feedback) to be later
%used in next respond
Eolder=Eold;
Eold=error; 

%Calculate the new action u
Uold=Wa*Er;
u=uPervious+Uold;
uPervious=u;
