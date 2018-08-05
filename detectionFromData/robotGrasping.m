%=========================================================================%
 
%                                                                         %       				
 
%       Function who process the information of the neural network        %
 
%			     Final study project INSA 2018                            %
 
%				VICTOR TALBOT MIQ5                                        %
 
% 3D object description, detection and segmentation for autonomous robot grasping  
    
%========================================================================%




%%

function [u,v,X,Y,theta,D] = robotGrasping(number)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

dec=onePassDectionForInstDefaultParamsDisplay(number,'/home/kebsox/catkin_ws/data','/home/kebsox/MachineLearning/Deep_learning_for_detectin_robotic_grasps/deepGraspingCode/backgrounds');

%pin-hole model for the SR300 You have to change with your value 

fx=617.558;
fy=617.558;
cx=312.151;
cy=245.775;
K=[fx 0 cx ; 0 fy cy ; 0 0 1];

%get the position of the second corner
u2=dec(2,2);
v2=dec(2,1);
Ci2=[u2 ; v2 ;1];

Cc2=K^-1*Ci2;

%get the position of the third corner

u3=dec(3,2);
v3=dec(3,1);
Ci3=[u3 ; v3 ;1];

Cc3=K^-1*Ci3;

%use stenopé model to get position in camera frame from image frame
%read my report for more information

Y=(dec(3,1)+dec(1,1))/2;
X=(dec(3,2)+dec(1,2))/2;

coordonnee_plan_image=[X Y 1]';

u=X;
v=Y;
coordonnee_plan_camera=K^-1*coordonnee_plan_image;
X=coordonnee_plan_camera(1);
Y=coordonnee_plan_camera(2);
D=sqrt((Cc3(1)-Cc2(1))^2+(Cc3(2)-Cc2(2))^2);
theta=rad2deg(atan((Cc3(2)-Cc2(2))/(Cc3(1)-Cc2(1))));
end

