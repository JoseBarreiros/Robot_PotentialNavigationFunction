%% Version 1.0
%% Implementation of Robot Path Planning using Potential Field Method 
%% Created by Jose Barreiros, PhD student, Systems Engineering, Cornell University. 

delete(findall(0,'Type','figure')) %clean figures 

% Import Image

filepath = 'map_hw1.png';

image = imread(filepath);

% Convert to grayscale and then black and white image based on arbitrary threshold

grayimage = rgb2gray(image);

bw= grayimage < 0.5;

%Plot the workspace

[y,x] = find(flipud(bw));
plot(x,y,'.m');
xlim([0 size(bw,1)]);
ylim([0 size(bw,1)]);
hold;

%Define start and end location of the robot. 
q0 = [50,900]; 

qf = [1100,800];

%Plot start and end location with a square marker.
plot([q0(:,1) qf(:,1)],[q0(:,2) qf(:,2)],'bs');  %include start and end point. 

%Start counting the computation time
tic;

%Initialize variables 

niu=300;
epsilon=0.3;
ro_0=30;

Q=[]; %vector used for plotting the results. 
q=q0; %initialize loop with inital position
F_at=[0,0];%attraction force init
F_rep=[0,0];%repulsive force init
T=0.3; %period of simulation
margin=2; %margin of error of robot position.  can be described as the precision of the robot or minimum step
dg=sqrt((qf(1)-q(1))^2+(qf(2)-q(2))^2); %euclidian distance from q to goal qf
delta_q=dg; %delta_q:  Diference from the last position to actual position of the robot.  It's been init with the distance from goal to robot. 
gama=0.005; %index to determine convergence.   It is used to stop infinite loop once the robot has reached a local minimum.  


while ((dg>margin)& (delta_q>gama))   
% Run the algorith.   The loop will stop when it reaches the goal
%within margin distance or find a local minimum. 
    
     %Atraction force
    F_at=-epsilon*(q-qf);  
     %Repulsion force
    F_rep=repulsion(q,bw,niu,ro_0);
    %Total Force
    Fn=F_at+F_rep;
    
    %Update q_n1 and include point in Q to present results at the end.
    q_n1=q;  % position q in n-1 (last position q) 
    Q=cat(1,q,Q);  %array that includes the trajectory. Used for plotting
    
    %Kinematics 
    an=1*Fn; %acceleration, assuming m=1.
    q=q_n1+0.5*an*T^2; %q: actual robot position
    
    %Calculate distance towards goal and change distance in last time step
    delta_q=sqrt(sum((q - q_n1) .^ 2)); %change of distance from qn to qn-1
    dg=sqrt(sum((q - qf) .^ 2)); %euclidian distance from q to goal qf
    
    %plot the new point q
    plot(q(1),q(2),'r.');
    
end

%Computation time
elap=toc

%%The following part is used just to present the results of simulation. 

%Plot start and end location with a square marker.
plot([q0(:,1) qf(:,1)],[q0(:,2) qf(:,2)],'bs');  
plot(Q(:,1),Q(:,2),'r.');
tex=text(q0(1),q0(2),'q0');
tex.FontSize=11;
tex.FontWeight = 'bold';
tex=text(qf(1),qf(2),'qf');
tex.FontSize=11;
tex.FontWeight = 'bold';

%Include a legend of the simulation paramentes.
txt=sprintf('epsilon: %1.2f\tniu: %i\tro_0:%i\nT: %.1f\tElapsedTime: %2.3f sec', epsilon,niu,ro_0,T,elap);
tex=text(370,70,txt);
tex.FontSize=11;
tex.FontWeight = 'bold';


function Fr=repulsion(x,ws,niu_1,ro_zero)   
%This function will retrive a vector of the total repulsion force for a
%given point x in a workspace ws with set paramentes, niu (multiplier of
%the force) and ro_0: distance of influence of repulsion of the obstacle. 

%Sense the enviroment usign a circle of radio=ro_0
targets=sensor(x,ws,ro_zero); %retrieve an array of points containing the discrete location of the obstacles (targets). 

%clear Rep Force vector
Fr=[0,0];
%plot(x(1),x(2),'om')
%plot(target(1),target(2),'oc')

%Calculate the repuslion force Frep(i)for every target located by the
%sensor and add up to total Frep.

if isempty(targets) 
    Fr=[0,0]; 
else 
    for k=1:size(targets,1)
      target=targets(k,:);  
      ro_q=sqrt((x(1)-target(1))^2+(x(2)-target(2))^2);
      if ro_q<=ro_zero
        Fr=Fr+niu_1*((1/ro_q)-(1/ro_zero))*(1/ro_q^2)*(x-target);
      else
        Fr=[0,0];    
      end
    end
end
end

function target_points=sensor(point,work_s,radio)
%This function will retrive a set of obstacle point sensed in a circle range or
%radio "radio" centered in "point", located in workspace "work_s"

%Retrive a mask matrix including a circle. 
cir=circle(point,radio,size(work_s,1),size(work_s,2));

%To plot the range of sensing
%[y,x] = find((cir));
%plot(x,y,'.c');

%The target area is calculated by intersecting the circle with the
%workspace. 
target_area=flipud(work_s.*flipud(cir));
[y,x] = find((target_area)); %transformation from a sparse matrix to a set of point [i,j]
plot(x,y,'.g');

 %plot(point(1),point(2),'*k')
 
target_points=[x(:),y(:)];
 %    plot(target_point(1),target_point(2),'*b')
 end
 
  
function circlePixels=circle(p,r,size_x,size_y)
% Create a logical image of a circle with specified
% diameter, center, and image size.

% First create the image.
imageSizeX = size_x;
imageSizeY = size_y;
[columnsInImage rowsInImage] = meshgrid(1:imageSizeX, 1:imageSizeY);

% Next create the circle in the image.
centerX = p(1);
centerY = p(2);
radius = r;
circlePixels = (rowsInImage - centerY).^2 + (columnsInImage - centerX).^2 <= radius.^2;

%For plotting
[y,x] = find((circlePixels));
plt=plot(x,y,'.c');

end

