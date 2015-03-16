m =0.2 ;
g= 9.8 ; 
Ib = 0.0104166; %Mass = 0.5 kg ; length = 0.5 m
% Moment of inertia = 1/12 M L^2
r = 0.020;
Iball = 0.0032;


%% Transfer Function

systf = tf([ m *g*r^2] , [ 0.5*(Iball +m*r^2) 0 0 ])

Kp = 10;
C = pid(10,10,10);
systf1=feedback(C*systf,1)
figure();
hold on
axis([-10 10 -10 10])
step(systf1)


%% State Space Model
%Input is Torque
%Output is the ball position

A = [ 0 1  0 0 ; 0 0 m*g/((Ib/r^2) + m) 0 ; 0 0 0 1; -m*g/(Ib + Iball) 0 0 0 ];
B = [1; 0 ; 0; 1/(Ib+Iball)]
C = [1 0 0  0]
D = 0

sys = ss(A,B,C,D);
 
J =  [-13+2*i -13-2*i -10 -11]
K = place(A,B,J)
%Settling time about 0.5 seconds

sys1 = ss(A-B*K , B,C,D)
step(sys);
%figure();
%rlocus(sys);
%figure();
%rlocus(sys1);
%pole(sys1)
%pole(sys)

con = ctrb(A,B)
rank(con)

ob = ctrb(A',C')
rank(ob)
R=1;


