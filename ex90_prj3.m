% Coefficient matrix:
    A=[3.2 4.1 7.7 3.7;
        -0.3 1.2 0.2 0.5;
        -1.8 -1.8 -4.4 -1.8;
        1.7 -0.7 2.9 0.4]
% Initial values:
    y0=[1;-4;2;3]
% If A is diagonalizable then MATLAB finds the P and D  in 
% the definition at page 315 as follows:
    [P,D]=eig(A)
% We now determine the general solution. 
% We start by telling MATLAB that it must consider
% a, b and t as variables. Notice, that a and b are the arbitrary 
% constants in the general solution and that t is 
% the time variable.
     syms a b t;
     arb=[a;b;c;d];
% The general solution can now be expressed.
% Nottice, that D*[t;t] equals [lambda_1*t;lambda_2*t], where
       % lambda_1 and lambda_2 are the two eigenvalues. Hence, exp(D*[t;t;t;t])
% equals [exp(lambda_1 * t); exp(lambda_2*t)].
% The expression (exp(D*[t;t]).*arb equals 
% [a*exp(lambda_1*t);b*exp(lambda_2*t)],
% as .* corresponds to componentwise  multiplication.
    ygen=P*(exp(D*[t;t;t;t]).*arb)
% The above general solution is now rounded off (you can adjust the 
% second argument to get a more precise or less precise answer):
    ygenrounded=vpa(ygen,2)  
% The arbitrary constants corresponding to the given initial conditions 
% are determined. Notice, that exp(lambda_1*0)=1 and exp(lambda_2*0)=1. 
% This implies that we must solve P*[a; b]=y0 for a and b.
    xx=linsolve(P,y0);
% The solution under the given initial conidition y0 becomes:
     y=P*(exp(D*[t;t;t;t]).*xx)
% The above solution is rounded off  (again you can adjust the second argument):
     yrounded=vpa(y,2)