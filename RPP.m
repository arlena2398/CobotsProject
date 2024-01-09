function pl= RPP(th1,rho1,rho2)   
%% A function named RPP used to generate the DH model of the cylindrical robot in 3D space %%
% INPUT(S):  th1: Angle of first revolute joint
%            rho1, rho2 : Stroke lengths of each prismatic joint
% OUTPUT(S): pl: A vector which saves all plots
% Copyrights: Swaminath Venkateswaran, Associate Professor/Researcher: ESILV/DVRC
%%
    O= [0,0,0];
    A= O+[0,0,10]; %Base of revolute joint q1
   
    %Plotting the link from origin to base of revolute joint-1 %
    h1=scatter3(O(1),O(2),O(3),'MarkerEdgeColor','k','MarkerFaceColor','k','Linewidth',2); %Origin of robot
    hold on;
    h2=plot3([O(1,1),A(1,1)],[O(1,2),A(1,2)],[O(1,3),A(1,3)],'k','LineWidth',2); %Origin to point A
    grid on;
    grid minor;
    set(gcf,'color','white')
    set(gca,'FontSize',20,'FontName','Times New Roman','FontWeight','Bold');
    x0 = 25; y0 = 45; % Origin for the plot screen
    largeur =950; % Length of plot screen from origin
    hauteur =550; % Width of plot screen from origin
    set(gcf,'units','points','position',[ x0, y0, largeur, hauteur])
    
    % Generation of revolute joint using Cylinder%
    r = 3; %Radius of cylinder
    h=10; %Height of cylinder
    [x,y,z] = cylinder(r);
    Ori= A; % Origin of cylinder (Revolute joint)%
    x=x+Ori(1);
    z=z*h+Ori(3); %Extrusion of cylinder%
    y=y+Ori(2);
    H1=surf(x,y,z); % Generating cylinder%
    xlim([-100,150]);
    ylim([-100,150]);
    zlim([-10,50]);
    set(H1,'facecolor','r','edgecolor','r');
    
    % To close the top and bottom faces of the revolute joint % !! Least priority
    theta= 0:0.1:2*pi;
    x1= Ori(1)+r.*cos(theta);y1= Ori(2)+r.*sin(theta);
    z1= Ori(3)*ones(1,length(x1));
    h3=fill3(x1,y1,z1,'r');
    theta= 0:0.1:2*pi;
    x1= Ori(1)+r.*cos(theta);y1= Ori(2)+r.*sin(theta);
    z1= (h+Ori(3))*ones(1,length(x1));
    h4=fill3(x1,y1,z1,'r');
    
    B= A+[0,0,10];% Head of revolute joint
    C= B+ [0,0,rho1+10]; % Offset to point C along z-axis
    
    %Connecting points B-C
    h5=plot3([B(1,1),C(1,1)],[B(1,2),C(1,2)],[B(1,3),C(1,3)],'k','LineWidth',2);
    
    %Sub-function 'rects' called to generate first prismatic joint
    rects(th1,C,20,'m') 
    
    D= C+[0,0,20]; % Point D from head of first prismatic joint
    E= D+[0,0,20]; % Offset E along z-axis from E
    
    %Connecting points D-E
    h6=plot3([D(1,1),E(1,1)],[D(1,2),E(1,2)],[D(1,3),E(1,3)],'k','LineWidth',2);
    
    % Generation of 90 degrees line with respect to DE
    % This line undergoes a z-rotation caused by theta1 and a y-rotation
    th2=pi/2;
    ry= [cos(th2) 0 sin(th2);0 1 0; -sin(th2) 0 cos(th2)];
    rz= [cos(th1) -sin(th1) 0;sin(th1) cos(th1) 0; 0 0 1];
    
    %Generation of point F %
    F= ry*[0;20+rho2;0];
    F= rz*F;
    F= F+E';
    F= F';
    
    %Connecting points E-F
    h7=plot3([F(1,1),E(1,1)],[F(1,2),E(1,2)],[F(1,3),E(1,3)],'k','LineWidth',2);
    
    %Generation of second prismatic joint using function 'rects2'    
    % G is the first face of the joint%
    G= rects2(F,th1,[],[],1,'Yes','g');
    
    % H to generate coordinates of second face caused by blocklength%
    H= ry*[0;40+rho2;0];
    H= rz*H;
    H= H+E';
    H= H';
    
    % I a vector to generate the second face of the prismatic joint%
    I= rects2(H,th1,[],[],1,'Yes','g');
    
    % Function 'rects2' called to generate sides of the prismatic joint%
    rects2([],[],G,I,2,'Yes','g');
    
    % Offset for positioning the end-effector %
    J= ry*[0;60+rho2;0];
    J= rz*J;
    J= J+E';
    J= J';
    
    % Vector to generate end-effector using 'rects2'
    K= rects2(J,th1,[],[],1,'Yes','b');
    
    % Connecting points H-J
    h8=plot3([H(1,1),J(1,1)],[H(1,2),J(1,2)],[H(1,3),J(1,3)],'k','LineWidth',2);
    
    % Generating a pick-place type tool%
    L= ry*[0;62+rho2;0];
    L= rz*L;
    L= L+E';
    L= L';
    
    % Length of end-effector %
    I= rects2(L,th1,[],[],1,'No','b');
    
    % 'rects2' called to generate the end-effector
    rects2([],[],K,I,2,'No','b');
    
    % Scatter point 'L' to represent the end-effector central point%
    h9=scatter3(L(1),L(2),L(3),'+','MarkerEdgeColor','b','MarkerFaceColor','k','Linewidth',2); %Origin of robot
    
    pl= [h1,h2,h3,h4,h5,h6,h7,h8,h9,H1]; %Storing the 'plots' to the output 'pl'

    xlim([-120,150]);
    ylim([-120,150]);
    zlim([-10,150]);
    xlabel('x');
    ylabel('y');
    zlabel('z');

    hold off;

 %%% END OF FUNCTION 'RPP' %%
 
 %% SUB-FUNCTIONS %%
    function rects(theta4,L22,bl,couleur)
    %% A SUB-FUNCTION "rects" TO CONSTRUCT THE PRISMATIC CUBOID %% !! Least priority
    % INPUT(S)  : theta4, L22, rho, bl(depth of block)
    % OUTPUT(S) : NIL
    %%
        % Rot-Z matrix for cuboid as it rotates according to theta-4%
        Rotaz= [cos(theta4),-sin(theta4);sin(theta4),cos(theta4)];
        
        % Generation of rectangle for bottom & top faces %
        Coord1= [L22(1);L22(2)]+Rotaz*[-2;-2];
        Coord2= [L22(1);L22(2)]+Rotaz*[2;-2];
        Coord3= [L22(1);L22(2)]+Rotaz*[2;2];
        Coord4= [L22(1);L22(2)]+Rotaz*[-2;2];
    
        %Extracting phases of cuboid %            
        Rect1x= [Coord1(1,1),Coord2(1,1),Coord3(1,1),Coord4(1,1)];
        Rect1y= [Coord1(2,1),Coord2(2,1),Coord3(2,1),Coord4(2,1)];
        Rect1z= [L22(3),L22(3),L22(3),L22(3)];
        Rect2z= [L22(3)+bl,L22(3)+bl,L22(3)+bl,L22(3)+bl]; %Depth of cuboid of 5%
        
        %Extraction of sides to fill the cuboid%
        Side1x= [Coord1(1,1),Coord1(1,1),Coord4(1,1),Coord4(1,1)];
        Side1y= [Coord1(2,1),Coord1(2,1),Coord4(2,1),Coord4(2,1)];
        Side1z= [L22(3),L22(3)+bl,L22(3)+bl,L22(3)];
        Side2x= [Coord2(1,1),Coord2(1,1),Coord3(1,1),Coord3(1,1)];
        Side2y= [Coord2(2,1),Coord2(2,1),Coord3(2,1),Coord3(2,1)];
        Side2z= [L22(3),L22(3)+bl,L22(3)+bl,L22(3)];
        Side3x= [Coord1(1,1),Coord2(1,1),Coord2(1,1),Coord1(1,1)];
        Side3y= [Coord1(2,1),Coord2(2,1),Coord2(2,1),Coord1(2,1)];
        Side3z= [L22(3),L22(3),L22(3)+bl,L22(3)+bl];
        Side4x= [Coord4(1,1),Coord3(1,1),Coord3(1,1),Coord4(1,1)];
        Side4y= [Coord4(2,1),Coord3(2,1),Coord3(2,1),Coord4(2,1)];
        Side4z= [L22(3),L22(3),L22(3)+bl,L22(3)+bl];
    
        %Filling all faces of cuboid%
        h100=fill3(Rect1x,Rect1y,Rect1z,couleur);
        h101=fill3(Rect1x,Rect1y,Rect2z,couleur);
        h102=fill3(Side1x,Side1y,Side1z,couleur);
        h103=fill3(Side2x,Side2y,Side2z,couleur);
        h104=fill3(Side3x,Side3y,Side3z,couleur);
        h105=fill3(Side4x,Side4y,Side4z,couleur);
    
    end
    
%%
%%
    function a= rects2(map,th1,G,I,i,arg,couleur)
    %% A SUB-FUNCTION "rects2" TO CONSTRUCT THE SECOND PRISMATIC CUBOID %% !! Least priority
    % INPUT(S)  : map, th1, G, I, i, arg, couleur
    % OUTPUT(S) : a- Faces of the rectangle
    %%    
        val=0;
        if(i==1)
            a1 = [-3*cos(val+th1),-3*sin(val+th1),2]+map;
            a2 = [-3*cos(val+th1),-3*sin(val+th1),-2]+map;
            a3 = [3*cos(val+th1),3*sin(val+th1),-2]+map;
            a4 = [3*cos(val+th1),3*sin(val+th1),2]+map;
    
            a=[a1;a2;a3;a4];
            Rect1x= [a(1,1),a(2,1),a(3,1),a(4,1)];
            Rect1y= [a(1,2),a(2,2),a(3,2),a(4,2)];
            Rect1z= [a(1,3),a(2,3),a(3,3),a(4,3)];
    
            if strcmp(arg,'Yes')
    
                h106=fill3(Rect1x,Rect1y,Rect1z,couleur);
    
            end
    
        elseif(i==2)
    
            if strcmp(arg,'Yes')
    
                Side1x= [G(1,1),G(4,1),I(4,1),I(1,1)];
                Side1y= [G(1,2),G(4,2),I(4,2),I(1,2)];
                Side1z= [G(1,3),G(4,3),I(4,3),I(1,3)];
    
                Side2x= [G(2,1),G(3,1),I(3,1),I(2,1)];
                Side2y= [G(2,2),G(3,2),I(3,2),I(2,2)];
                Side2z= [G(2,3),G(3,3),I(3,3),I(2,3)];
    
                h107=fill3(Side1x,Side1y,Side1z,couleur);
                h108=fill3(Side2x,Side2y,Side2z,couleur);
    
            end
            
                Side3x= [G(1,1),G(2,1),I(2,1),I(1,1)];
                Side3y= [G(1,2),G(2,2),I(2,2),I(1,2)];
                Side3z= [G(1,3),G(2,3),I(2,3),I(1,3)];
    
                Side4x= [G(3,1),G(4,1),I(4,1),I(3,1)];
                Side4y= [G(3,2),G(4,2),I(4,2),I(3,2)];
                Side4z= [G(3,3),G(4,3),I(4,3),I(3,3)];
    
                h109=fill3(Side3x,Side3y,Side3z,couleur);
                h110=fill3(Side4x,Side4y,Side4z,couleur);
    
        end
    
    end

end