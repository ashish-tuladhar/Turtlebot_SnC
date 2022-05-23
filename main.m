
close all;
clear all;

rosshutdown;
ip = '192.168.17.149';
% intialise ros if it isn't initialised
try
    rosnode list
catch exp
    rosinit(ip,11311,'NodeHost','192.168.17.149')
end

ipaddress = '192.168.17.149'; % IP address of your robot
tbot = turtlebot(ipaddress,11311);
tbot.Velocity.TopicName = '/cmd_vel';

 
depthsub = rossubscriber('/camera/depth/image_raw');
depthmsg = receive(depthsub,100);
depthimg = readImage(depthmsg);


rgbsub=rossubscriber('/camera/rgb/image_raw');
rgbmsg=receive(rgbsub,100);
rgbimg=readImage(rgbmsg);

odom = rossubscriber('/odom');


original= rgb2gray(imread('cereal.jpg'));
distorted = rgb2gray(rgbimg);



%% Detect Image 

[tform, inlierDistorted, inlierOriginal]= ImageProcessing(tbot, original,distorted);


%% Finding Global Coordinates


%
t1 = [0.064 -0.065 0.104];%base footprint to camera link
t1mat = trvec2tform(t1);
t2 = [0.005 0.018 0.013];%camera link to camera rgb
t2mat = trvec2tform(t2);
Quat = quaternion([0.500398163355 -0.499999841466 0.499601836645 -0.499999841466]);%Constant
t3 = rotmat(Quat,'point');%camera rgb to rgb optical frame
t3mat = rotm2tform(t3);
TFCamera2Base = t1mat*t2mat*t3mat;%basefootprint to RGB optical frame
K = [1206.8897719532354 0.0 960.5; 0.0 1206.8897719532354 540.5; 0.0 0.0 1.0 ];


msgOdo = receive(odom,10);
orientW = msgOdo.Pose.Pose.Orientation.W;
orientX = msgOdo.Pose.Pose.Orientation.X;
orientY = msgOdo.Pose.Pose.Orientation.Y;
orientZ = msgOdo.Pose.Pose.Orientation.Z;
quatBot = quaternion([orientW orientX orientY orientZ]);
rotBot = rotmat(quatBot,'point');
tmat1 = rotm2tform(rotBot);
transBot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
tmat2 = trvec2tform(transBot);
TFBase2Global = tmat1*tmat2; 

depthimg=getDepthImage(tbot);

%[baseCoordinate, globalCoordinate]=Pixel2Global(inlierDistorted,depthimg, K, TFCamera2Base, TFBase2Global);
%% LAHDKS

depthValues = nan(inlierDistorted.Count,1);
%CP = nan(3,inlierDistorted.Count);
cameraCoordinate = nan(3,inlierDistorted.Count);
baseCoordinate = nan(4,inlierDistorted.Count);
globalCoordinate = nan(4,inlierDistorted.Count);

for i=1 : inlierDistorted.Count
    depthValues(i) = depthimg(round(inlierDistorted.Location(i,2)),round(inlierDistorted.Location(i,1)));
    %CP(:,i) = [(depthValues(i)* inlierDistorted.Location(i,1)) ; (depthValues(i)* inlierDistorted.Location(i,2)) ; depthValues(i)];
    cameraCoordinate(:,i) = K\[(depthValues(i)* inlierDistorted.Location(i,1)) ; (depthValues(i)* inlierDistorted.Location(i,2)) ; depthValues(i)];

    %cameraCoordinate(:,i) = K\CP(:,i);
    baseCoordinate(:,i) =  TFCamera2Base*[cameraCoordinate(:,i);1];
    %globalCoordinate(:,i)=TFBase2Global*baseCoordinate(:,i);

end

baseCoordinate=baseCoordinate(:,all(~isnan(baseCoordinate)));

for i=1:size(baseCoordinate,2)
    globalCoordinate(:,i)=TFBase2Global*baseCoordinate(:,i);
end

globalCoordinate=globalCoordinate(:,all(~isnan(globalCoordinate)));

%% Finding Object Location and Plane
objectPointLocal=mean(baseCoordinate,2);
objectPointGlobal=TFBase2Global*objectPointLocal

ptCloud = pointCloud(globalCoordinate(1:3,:)');
plane = pcfitplane(ptCloud,1);
normal=plane.Normal(1:2);
parallel=[0 0];
parallel(1)=-normal(2);
parallel(2)=normal(1);


%% Finding the lines and intersection point

norPt1=[objectPointGlobal(1), objectPointGlobal(2)];
norPt2=[objectPointGlobal(1)-normal(1),objectPointGlobal(2)-normal(2)];

parPt1=[msgOdo.Pose.Pose.Position.X, msgOdo.Pose.Pose.Position.Y];
parPt2=[msgOdo.Pose.Pose.Position.X+parallel(1),msgOdo.Pose.Pose.Position.Y+parallel(2)];

% line1x=[norPt1(1),norPt2(1)];
% line1y=[norPt1(2),norPt2(2)];
% line2x=[parPt1(1),parPt2(1)];
% line2y=[parPt1(2),parPt2(2)];

line1=[norPt1(1),norPt1(2),norPt2(1),norPt2(2)];
line2=[parPt1(1),parPt1(2),parPt2(1),parPt2(2)];
%[xint,yint]=linexline(line1x,line1y,line2x,line2y,0)
[xint,yint]=line_intersection(line1,line2);

intersectionPoint=[xint, yint]




%% Going to the Intersection Point

%MoveTurtlebotToSetLocation(tbot,xint,yint);
moveBot(tbot,xint,yint);
if (dot(parPt1-norPt1,normal)>0)
    finalPoint=[objectPointGlobal(1)+0.5*normal(1),objectPointGlobal(2)+0.5*normal(2)];
else
    finalPoint=[objectPointGlobal(1)-0.5*normal(1),objectPointGlobal(2)-0.5*normal(2)];
end
finalPoint
%MoveTurtlebotToSetLocation(tbot,finalPoint(1),finalPoint(2));
moveBot(tbot,finalPoint(1),finalPoint(2));
