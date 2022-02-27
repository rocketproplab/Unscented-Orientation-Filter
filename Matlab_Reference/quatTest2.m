function [] = quatTest2()
    %
    % quatTest2()
    % This function tests the quaternion operations, and demonstrates 
    %   rotations by quaternion.
    
    % Define orientation, the quaternion representing the rotation (in our 
    %   case, the rotation of our rocket gyroscope). [0;0;0;1] represents 
    %   no rotation.
    orientation = [0;0;0;1];
    
    % Define dir, the direction we want to graph. The graph will show which
    %   direction dir would point if you would performed a series of 
    %   rotations on it. 
    % The format is [x;y;z]
    dir = [0.577;0.577;0.577];
    
    % Define vecs, the vectors we are graphing.
    vecs = zeros(3,21);
    vecs(1:3,1) = dir;
    clf
    
    % Define i = 1:number of rotations in this direction
    for i = 1:20
        
        % rate of change of rotation, in radians, received from the gyro.
        % roc = [roll; pitch; yaw]
        roc = [0; 0; pi/20];
        
        % update the rotation quaternion with the roc
        % the third parameter is the time interval for the roc, received
        %   from the gyro.
        orientation = gyroIntegrate(orientation,roc,1);
        
        % rotate dir by the new quaternion, add it to the vecs to graph
        vec = quatRotate(orientation,dir);
        vecs(1:3,i+1) = vec;
    end
    
    % graphs vecs with color 1
    plotQuats(vecs,1)
    hold on
    
    % restart vecs, graph again with a new roc and colors
    vecs = zeros(3,11);
    vecs(1:3,1) = vec;
    for i = 1:10
        roc = [0; -pi/20; 0];
        orientation = gyroIntegrate(orientation,roc,1);
        vec = quatRotate(orientation,dir);
        vecs(1:3,i+1) = vec;
    end
    
    % use color 2
    plotQuats(vecs,2)
    
    % restart vecs, graph again with a new roc and colors
    vecs = zeros(3,11);
    vecs(1:3,1) = vec;
    for i = 1:10
        roc = [-pi/20; 0; 0];
        orientation = gyroIntegrate(orientation,roc,1);
        vec = quatRotate(orientation,dir);
        vecs(1:3,i+1) = vec;
    end
    
    % use color 3
    plotQuats(vecs,3)
    
    % make a legend, and define the graph size 
    legend("1","2","3")
    xlim([-norm(dir) norm(dir)])
    ylim([-norm(dir) norm(dir)])
    zlim([-norm(dir) norm(dir)])
end

function [] = plotQuats(threeVecs,idx)
    %
    % plotQuats(fourVecs,idx)
    % this function plots the vectors rotated by the orientation at each
    %   time.
    % fourVecs are the quaternions to graph. Since these are rotated
    %   vectors, only the first three components matter.
    % idx is
    
    % record starting points for vectors
    starts = zeros(3,size(threeVecs,2));
    
    % record the color of the axes
    colorOrder = get(gca, 'ColorOrder');
    
    % pick a color to graph the vectors
    color = colorOrder(mod(idx, size(colorOrder, 1)+1),:);
    
    % graph vectors
    quiver3(starts(1,:),starts(2,:),starts(3,:),threeVecs(1,:),...
        threeVecs(2,:),threeVecs(3,:),"color", color);
    hold on
    grid on
    
    % graph line
    plot3(threeVecs(1,:),threeVecs(2,:),threeVecs(3,:),"color", color);
end