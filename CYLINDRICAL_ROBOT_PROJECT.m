clear all;
thetamin=0;
thetamax=2*pi;
syms Q1 Rho1 Rho2 L1 L2
theta_limit=[thetamin,thetamax];
rho1_limit=[0,40];
rho2_limit=[0;50];
d1=10;  
d2=10;
q1=pi/4;
Ori= [0,0,0];
% Draw of the initial positon of the robot
%  drawnow 
% subplot(1,2,1);
% RPP(q1,d1,d2);
% hold on;
% subplot(1,2,2);
% RPP(q1,d1,d2);
% view(2);
% hold off;
%DH matrix for the initial position
alpha=[0;0;-pi/2];
r=[20;d1+50;d2+62];
theta=[q1,0,0];
d=[0;0;0];
T=DenaHart(alpha, d, theta, r)
% Jacobian symbolic***
syms Q1 Rho1 Rho2 L1 L2% L1=70 fix lenght of the robot in Z, L2=62 lenght of the robot in XY
J= JacobianCylindrical(Q1,Rho1,Rho2,L1,L2);
f=3;
tf=15;
n=15/(1/3);

%graph 0-A (pi/2,0,10)
%      B-A (pi/2,25,10)
O=[0,0,0;pi/2,0,10;pi/2,25,10];% MATRIX WITH THE VALUES FOR EVERY JOIN
for j=1:2
    theta_limit=[O(j,1),O(j+1,1)];
    rho1_limit=[O(j,2),O(j+1,2)];
    rho2_limit=[O(j,3),O(j+1,3)];
    for i=1:(n+1)
        %******* Calcule of r, rd, rdd*****************************
        t(i)=(i-1)*tf/n;
        r(i)=10*(t(i)/tf)^3-15*(t(i)/tf)^4+6*(t(i)/tf)^5;
        rv(i)=30*(t(i)^2/tf^3)-60*(t(i)^3/tf^4)+30*(t(i)^4/tf^5);
        ra(i)=60*(t(i)/tf^3)-180*(t(i)^2/tf^4)+120*(t(i)^3/tf^5);
        %*****simulation position*************************
        theta1(i,j)=theta_limit(1)+(theta_limit(2)-theta_limit(1))*r(i);
        rho1(i,j)=rho1_limit(1)+(rho1_limit(2)-rho1_limit(1))*r(i);
        rho2(i,j)=rho2_limit(1)+(rho2_limit(2)-rho2_limit(1))*r(i);
         %************velocity****************************
        theta1v(i,j)=(theta_limit(2)-theta_limit(1))*rv(i);
        rho1v(i,j)=(rho1_limit(2)-rho1_limit(1))*rv(i);
        rho2v(i,j)=(rho2_limit(2)-rho2_limit(1))*rv(i);
        %***************aceleration***********************
        theta1a(i,j)=(theta_limit(2)-theta_limit(1))*ra(i);
        rho1a(i,j)=(rho1_limit(2)-rho1_limit(1))*ra(i);
        rho2a(i,j)=(rho2_limit(2)-rho2_limit(1))*ra(i); 
        r=[20;rho1(i,j)+50;rho2(i,j)+62];
        theta=[theta1(i,j),0,0];
        Dena(i).T= DenaHart(alpha,d,theta,r);
        drawnow 
        subplot(1,2,1);
        RPP(theta1(i,j),rho1(i,j),rho2(i,j));
        subplot(1,2,2);
        RPP(theta1(i,j),rho1(i,j),rho2(i,j));
        view(180,0);
    end
end
%***************JOIN SPACE SIMULATION MOVEMENT B-C**********************
%Save the last value of x,y,z of the task space simulation previously do it
B= Dena(end).T(1:3,4);
B=B';
C=[-B(1),B(2),B(3)];
%C=[65,65,80];
jt=[B;C];
%jt(:,3)=abs(jt(:,3)-70);
nfinal=size(theta1);

for j=1:1
    rho1n=jt(j+1,3)-70;
    L=sqrt(jt(j+1,1)^2+jt(j+1,2)^2);
    rho2n=L-62;
    q1n=atan2(jt(j+1,2),jt(j+1,1))-pi/2;
    % if(q1n<0)
    %        q1n= q1n+2*pi;
    % end
    theta_limit=[theta1(nfinal(1),nfinal(2)),q1n];
    rho1_limit=[rho1(nfinal(1),nfinal(2)),rho1n];
    rho2_limit=[rho2(nfinal(1),nfinal(2)),rho2n];
end
j=nfinal(2)+1;
for i=1:(n+1)
        %******* Calcule of r, rd, rdd*****************************
        t(i)=(i-1)*tf/n;
        r(i)=10*(t(i)/tf)^3-15*(t(i)/tf)^4+6*(t(i)/tf)^5;
        rv(i)=30*(t(i)^2/tf^3)-60*(t(i)^3/tf^4)+30*(t(i)^4/tf^5);
        ra(i)=60*(t(i)/tf^3)-180*(t(i)^2/tf^4)+120*(t(i)^3/tf^5);
        %*****simulation position*************************
        theta1(i,j)=theta_limit(1)+(theta_limit(2)-theta_limit(1))*r(i);
        rho1(i,j)=rho1_limit(1)+(rho1_limit(2)-rho1_limit(1))*r(i);
        rho2(i,j)=rho2_limit(1)+(rho2_limit(2)-rho2_limit(1))*r(i);
         %************velocity****************************
        theta1v(i,j)=(theta_limit(2)-theta_limit(1))*rv(i);
        rho1v(i,j)=(rho1_limit(2)-rho1_limit(1))*rv(i);
        rho2v(i,j)=(rho2_limit(2)-rho2_limit(1))*rv(i);
        %***************aceleration***********************
        theta1a(i,j)=(theta_limit(2)-theta_limit(1))*ra(i);
        rho1a(i,j)=(rho1_limit(2)-rho1_limit(1))*ra(i);
        rho2a(i,j)=(rho2_limit(2)-rho2_limit(1))*ra(i); 
        r=[20;rho1(i,j)+50;rho2(i,j)+62];
        theta=[theta1(i,j),0,0];
        Dena(i).T= DenaHart(alpha,d,theta,r);
        drawnow 
        subplot(1,2,1);
        RPP(theta1(i,j),rho1(i,j),rho2(i,j));
        subplot(1,2,2);
        RPP(theta1(i,j),rho1(i,j),rho2(i,j));
        view(180,0);
end
Theta1= theta1;
Theta1d= theta1v;
Theta1dd=theta1a;
Rho2= rho2;
Rho2d= rho2v;
Rho2dd=rho2a;
RHO1= rho1;
RHO1d= rho1v;
RHO1dd=rho1a;

theta1= zeros();
theta1v=zeros();
theta1a=zeros();
rho2= zeros();
rho2v=zeros();
rho2a=zeros();
rho1= zeros();
rho1v=zeros();
rho1a=zeros();
%***************TASK SPACE SIMULATION MOVEMENT C-D AND RETURN  O POSITION**********************
nfinal=size(Theta1);
vax=[Theta1(nfinal(1),nfinal(2)),RHO1(nfinal(1),nfinal(2)),Rho2(nfinal(1),nfinal(2))]
O=[vax;vax(1),0,vax(3);vax;0,0,0];% MATRIX WITH THE VALUES FOR EVERY JOIN
x=nfinal(2)+1;
for j=1:3%3
    theta_limit=[O(j,1),O(j+1,1)];
    rho1_limit=[O(j,2),O(j+1,2)];
    rho2_limit=[O(j,3),O(j+1,3)];
    % n=j;
    % j=j+3;
    for i=1:(n+1)
        %******* Calcule of r, rd, rdd*****************************
        t(i)=(i-1)*tf/n;
        r(i)=10*(t(i)/tf)^3-15*(t(i)/tf)^4+6*(t(i)/tf)^5;
        rv(i)=30*(t(i)^2/tf^3)-60*(t(i)^3/tf^4)+30*(t(i)^4/tf^5);
        ra(i)=60*(t(i)/tf^3)-180*(t(i)^2/tf^4)+120*(t(i)^3/tf^5);
        %*****simulation position*************************
        theta1(i,j)=theta_limit(1)+(theta_limit(2)-theta_limit(1))*r(i);
        rho1(i,j)=rho1_limit(1)+(rho1_limit(2)-rho1_limit(1))*r(i);
        rho2(i,j)=rho2_limit(1)+(rho2_limit(2)-rho2_limit(1))*r(i);
         %************velocity****************************
        theta1v(i,j)=(theta_limit(2)-theta_limit(1))*rv(i);
        rho1v(i,j)=(rho1_limit(2)-rho1_limit(1))*rv(i);
        rho2v(i,j)=(rho2_limit(2)-rho2_limit(1))*rv(i);
        %***************aceleration***********************
        theta1a(i,j)=(theta_limit(2)-theta_limit(1))*ra(i);
        rho1a(i,j)=(rho1_limit(2)-rho1_limit(1))*ra(i);
        rho2a(i,j)=(rho2_limit(2)-rho2_limit(1))*ra(i); 
        r=[20;rho1(i,j)+50;rho2(i,j)+62];
        theta=[theta1(i,j),0,0];
        Dena(i).T= DenaHart(alpha,d,theta,r);
        drawnow 
        subplot(1,2,1);
        RPP(theta1(i,j),rho1(i,j),rho2(i,j));
        subplot(1,2,2);
        RPP(theta1(i,j),rho1(i,j),rho2(i,j));
        view(180,0);
        
    end
    % j=n;
end
%************VECTORS FINALS TO PLOT*********************
Time= t';
Theta1= [Theta1,theta1];
Theta1d= [Theta1d,theta1v];
Theta1dd=[Theta1dd,theta1a];
 %----- Vectors for rho2--------------------
      Rho2= [Rho2,rho2];
      Rho2d= [Rho2d,rho2v];
      Rho2dd=[Rho2dd,rho2a];
      %-------Vector for rho1----------------
      RHO1= [RHO1,rho1];
      RHO1d=[RHO1d,rho1v];
      RHO1dd=[RHO1dd,rho1a];
 mov=size(Theta1)
 theta1ax= [Theta1(:,1)];
      Theta1dax= [Theta1d(:,1)];
      Theta1ddax=[Theta1dd(:,1)];
      %----- Vectors for rho2--------------------
      Rho2ax= [Rho2(:,1)];
      Rho2dax= [Rho2d(:,1)];
      Rho2ddax=[Rho2dd(:,1)];
      %-------Vector for rho1----------------
      RHO1ax= [RHO1(:,1)];
      RHO1dax=[RHO1d(:,1)];
      RHO1ddax=[RHO1dd(:,1)];
 for k=2:mov(2)
     theta1ax= [theta1ax;Theta1(:,k)];
      Theta1dax= [Theta1dax;Theta1d(:,k)];
      Theta1ddax=[Theta1ddax;Theta1dd(:,k)];
      %----- Vectors for rho2--------------------
      Rho2ax= [Rho2ax;Rho2(:,k)];
      Rho2dax= [Rho2dax;Rho2d(:,k)];
      Rho2ddax=[Rho2ddax;Rho2dd(:,k)];
      %-------Vector for rho1----------------
      RHO1ax= [RHO1ax;RHO1(:,k)];
      RHO1dax=[RHO1dax;RHO1d(:,k)];
      RHO1ddax=[RHO1ddax;RHO1dd(:,k)];
      Time=[Time;t'+(k)*tf];
 end
 %-------------PLOT------------------
    figure(2);
    subplot(3,3,1);
    plot(Time,theta1ax*180/pi,'m','Linewidth',1);%theta 1
    xlabel('Time (s)', 'FontSize', 10);
    str='$\theta_{1} (deg)$';
    ylabel(str, 'Interpreter','latex');
    subplot(3,3,2);
    plot(Time,Theta1dax*180/pi,'m','Linewidth',1);% w1
    xlabel('Time (s)', 'FontSize', 10);
    str='$\dot{\theta_{1}}$ (deg/s)';
    ylabel(str, 'Interpreter','latex');
    subplot(3,3,3);
    plot(Time,Theta1ddax*180/pi,'m','Linewidth',1);% alpha 1
     xlabel('Time (s)', 'FontSize', 10);
     str='$\ddot{\theta_{1}} (deg/s^{2})$';
    ylabel(str, 'Interpreter','latex');
    %-----Plot theta 2------------------------
    subplot(3,3,4);
    plot(Time,Rho2ax,'g','Linewidth',1);%rho 2
    xlabel('Time (s)', 'FontSize', 10);
    str='$\rho_{2} (mm)$';
    ylabel(str, 'Interpreter','latex');
    subplot(3,3,5);
    plot(Time,Rho2dax,'g','Linewidth',1);% v2
    xlabel('Time (s)', 'FontSize', 10);
    str='$\dot{\rho_{2}}$ (mm/s)';
    ylabel(str, 'Interpreter','latex');
    subplot(3,3,6);
    plot(Time,Rho2ddax,'g','Linewidth',1);% a2
     xlabel('Time (s)', 'FontSize', 10);
     str='$\ddot{\rho_{2}} (mm/s^{2})$';
    ylabel(str, 'Interpreter','latex');
   %--------Subplot Rho---------
   subplot(3,3,7);
    plot(Time,RHO1ax,'r','Linewidth',1);%rho
    xlabel('Time (s)', 'FontSize', 10);
    str='$\rho_{1} (mm)$';
    ylabel(str, 'Interpreter','latex');
    subplot(3,3,8);
    plot(Time,RHO1dax,'r','Linewidth',1);% rho v.lineal
    xlabel('Time (s)', 'FontSize', 10);
    str='$\dot{\rho_{1}}$ (mm/s)';
    ylabel(str, 'Interpreter','latex');
    subplot(3,3,9);
    plot(Time,RHO1ddax,'r','Linewidth',1);% rho a. lineal
     xlabel('Time (s)', 'FontSize', 10);
     str='$\ddot{\rho_{1}} (mm/s^{2})$';
    ylabel(str, 'Interpreter','latex');
    sgtitle('Joint position, velocity and acceleration ');