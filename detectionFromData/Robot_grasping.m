


dec=onePassDectionForInstDefaultParamsDisplay(10,'/home/kebsox/MachineLearning/Deep_learning_for_detectin_robotic_grasps/deepGraspingCode/data','/home/kebsox/MachineLearning/Deep_learning_for_detectin_robotic_grasps/deepGraspingCode/backgrounds');

D=sqrt((dec(3,1)-dec(2,1))^2+(dec(3,2)-dec(2,2))^2);

a=sqrt((dec(2,1)-dec(1,1))^2+(dec(2,2)-dec(1,2))^2);

theta=atan((dec(4,1)-dec(2,1))/(dec(4,2)-dec(2,2)) );

Y=(dec(3,1)+dec(1,1))/2;
X=(dec(3,2)+dec(1,2))/2;

coordonnee_plan_image=[X Y 1]';

fx=617.558;
fy=617.558;
cx=312.151;
cy=245.775;
Modele_stenope=[fx 0 cx ; 0 fy cy ; 0 0 1];

coordonnee_plan_camera=coordonnee_plan_image.*Modele_stenope^-1;