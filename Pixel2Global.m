
function [baseCoordinate, globalCoordinate]=Pixel2Global(inlierDistorted,depthimg, K, TFCamera2Base, TFBase2Global)

depthValues = nan(inlierDistorted.Count,1);
CP = nan(3,inlierDistorted.Count);
cameraCoordinate = nan(3,inlierDistorted.Count);
baseCoordinate = nan(4,inlierDistorted.Count);
globalCoordinate = nan(4,inlierDistorted.Count);

for i=1 : inlierDistorted.Count
    depthValues(i) = depthimg(round(inlierDistorted.Location(i,2)),round(inlierDistorted.Location(i,1)));
    CP(:,i) = [(depthValues(i)* inlierDistorted.Location(i,1)) ; (depthValues(i)* inlierDistorted.Location(i,2)) ; depthValues(i)];
    %cameraCoordinate(:,i) = K\[(depthValues(i)* inlierDistorted.Location(i,1)) ; (depthValues(i)* inlierDistorted.Location(i,2)) ; depthValues(i)];

    cameraCoordinate(:,i) = K\CP(:,i);
    baseCoordinate(:,i) =  TFCamera2Base*[cameraCoordinate(:,i);1];
    %globalCoordinate(:,i)=TFBase2Global*baseCoordinate(:,i);


end

baseCoordinate=baseCoordinate(:,all(~isnan(baseCoordinate)));

for i=1:size(baseCoordinate,2)
    globalCoordinate(:,i)=TFBase2Global*baseCoordinate(:,i);
end

globalCoordinate=globalCoordinate(:,all(~isnan(globalCoordinate)));
end


