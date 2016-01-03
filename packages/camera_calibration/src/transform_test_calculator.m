function transform_test_calculator()
    %global kinectPoints;
    %global actualPoints;
    
    
 %   kinectPoint=[.34249;-.863501;-0.776416];
 %   robotPoint = calculateTransform(kinectPoint)
%     fprintf('Beginning point generation\n');
%     for i = 1:10
%         kinectPoint = rand(1,3); % Assumes no points will be father than 1 meter away in any direction
%         kinectPoints(i,:) = kinectPoint;
%         actualPoint = calculateTransform(kinectPoint);
%         actualPoints(i,:) = actualPoint;
%         fprintf('Point pair calculated. Kinect point is (%f, %f, %f), robot point is (%f, %f, %f)\n', kinectPoint(1), kinectPoint(2), kinectPoint(3), actualPoint(1), actualPoint(2), actualPoint(3)); 
%     end
%     fprintf('Point generation complete\n');
%end

%function actualPoint = calculateTransform(kinectPoint)
    %rotation = [3.06586;3.0245;2.6097];%sym('rotation',[3 1]);
    rotation = [-0.520712;0.119524;-0.778738];
    %translation = [0.0610103;0.947181;0.0760576];%sym('translation',[3 1]);
    translation = [-0.0302156;0.529536;0.381398];
    kinectPoint = [-0.999558;-0.304923;0.464212];%sym('kinect', [3 1]);
    %robotPoint = sym('robot', [3 1]);
    Rx = [1,0,0; 0, cos(rotation(1)), -sin(rotation(1)); 0, sin(rotation(1)), cos(rotation(1))];
    Ry = [cos(rotation(2)), 0, sin(rotation(2)); 0, 1, 0; -sin(rotation(2)), 0, cos(rotation(2))];
    Rz = [cos(rotation(3)), -sin(rotation(3)), 0; sin(rotation(3)), cos(rotation(3)), 0;0,0,1];
    R = Rx*Ry*Rz;
    p = [translation(1);translation(2);translation(3)];
    g = [R p; 0 0 0 1];
    k = [kinectPoint(1); kinectPoint(2); kinectPoint(3); 1];
    t = g*k;
    transformedPoint = [t(1); t(2); t(3)];
    %output =  robotPoint - transformedPoint;
    disp(transformedPoint);
end