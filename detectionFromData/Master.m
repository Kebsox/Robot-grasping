%=========================================================================%
 
%                                                                         %       				
 
%                	Master code                                           %
 
%			     Final study project INSA 2018                            %
 
%				VICTOR TALBOT MIQ5                                        %
 
% 3D object description, detection and segmentation for autonomous robot grasping  
    
%========================================================================%




%%


rosshutdown %close ros in case of error
rosinit %init a ros node

%create publisher for information exchange

chatterpub = rospublisher('/master', 'std_msgs/String');
pause(2) % Wait to ensure publisher is registered
chatterpub_robot = rospublisher('/master_robot', 'std_msgs/String');
pause(2) % Wait to ensure publisher is registered
chatterpub_u = rospublisher('/master_u', 'std_msgs/Int16');
pause(2) % Wait to ensure publisher is registered
chatterpub_v = rospublisher('/master_v', 'std_msgs/Int16');
pause(2) % Wait to ensure publisher is registered
chatterpub_commande = rospublisher('/resultat', 'std_msgs/Float64MultiArray');
pause(2) % Wait to ensure publisher is registered
%sub = rossubscriber('/master_z');
pause(1);
chattermsg = rosmessage(chatterpub);
chattermsg_robot = rosmessage(chatterpub_robot);
chattermsg_u = rosmessage(chatterpub_u);
chattermsg_v = rosmessage(chatterpub_v);
chattermsg_resultat = rosmessage(chatterpub_commande);
continu = true;
number=1;

%create a litlle gui for user
while continu
    
    R = input('1. Catch object \n2. Exit \n ');
    if R==1 %when you want to catch a objet
        
       chattermsg_robot.Data = 'a'; %publish a message for the robot to go to the picture position
       send(chatterpub_robot,chattermsg_robot);
       pause(20)
       chattermsg.Data = 'a'; %publish a message for the camera to take pictures
       send(chatterpub,chattermsg);
       
       chattermsg_robot.Data = 'b'; %publish a message for the robot to go to the waiting position position
       send(chatterpub_robot,chattermsg_robot); 
       pause(2)
       
       [u,v,X,Y,theta,D]=robotGrasping(number) %run the neural network
       u=60;
       v=180;
       chattermsg_u.Data = u;
       send(chatterpub_u,chattermsg_u); 
       pause(2)
       chattermsg_v.Data = v;
       send(chatterpub_v,chattermsg_v); 
       pause(2)
       
       chattermsg_robot.Data = 'a';
       send(chatterpub_robot,chattermsg_robot);
       pause(20)
       
       Z = receive(sub,10)
       number=number+1;
       z=Z.Data
       
       chattermsg_robot.Data = 'b';
       
       send(chatterpub_robot,chattermsg_robot); %send the result of machine learning to a ros topic
       pause(20)
       
       chattermsg_resultat.Data = [X Y z theta D];
       send(chatpub,chattermsg_resultat);
    end
    
    if R==2 %exit the node 
        
        chattermsg.Data = 'b';
        send(chatterpub,chattermsg);
        pause(2)
        chattermsg_robot.Data = 'c';
        send(chatterpub_robot,chattermsg_robot);
        disp('exit');
        
        continu = false;
        rosshutdown
    end
    
    
end