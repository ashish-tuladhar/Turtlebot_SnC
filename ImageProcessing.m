function [tform, inlierDistorted, inlierOriginal]= ImageProcessing(tbot, original,distorted)

%% Detect Image 

ptsOriginal = detectSURFFeatures(original);
ptsDistorted = detectSURFFeatures(distorted);
[featuresOriginal, validPtsOriginal]=extractFeatures(original,ptsOriginal);
[featuresDistorted,validPtsDistorted]=extractFeatures(distorted,ptsDistorted);

indexPairs=matchFeatures(featuresOriginal,featuresDistorted);
points = numel(indexPairs,1);

if points < 50
    while true
        findSigns(tbot,points);
%         depthmsg = receive(depthsub,100);
%         depthimg = readImage(depthmsg);
%         rgbmsg=receive(rgbsub,100);
%         rgbimg=readImage(rgbmsg);
        
        rgbimg = getColorImage(tbot);

        distorted = rgb2gray(rgbimg);
        
        ptsOriginal = detectSURFFeatures(original);
        ptsDistorted = detectSURFFeatures(distorted);
        [featuresOriginal, validPtsOriginal]=extractFeatures(original,ptsOriginal);
        [featuresDistorted,validPtsDistorted]=extractFeatures(distorted,ptsDistorted);
        
        indexPairs=matchFeatures(featuresOriginal,featuresDistorted);
        
        points = numel(indexPairs);
        disp('Searching for image...')
        if points > 50
            disp('Image Found!');
            break;
        end
    end
end
matchedOriginal=validPtsOriginal(indexPairs(:,1));
matchedDistorted=validPtsDistorted(indexPairs(:,2));

figure();
showMatchedFeatures(original,distorted,matchedOriginal,matchedDistorted);
[tform, inlierDistorted, inlierOriginal]= estimateGeometricTransform( matchedDistorted,matchedOriginal,'projective');

%% Make a box

originalBox = [1, 1; 
              size(original, 2), 1; 
              size(original, 2), size(original, 1); 
              1, size(original, 1); 
              1, 1]; 
        
Tinv =tform.invert;
distortedBox = transformPointsForward(Tinv, originalBox);

figure()
imshow(distorted);
axis on 
hold on;
line(distortedBox(:, 1), distortedBox(:, 2), 'Color', 'r', 'LineWidth', 2);




end