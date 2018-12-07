h =COM_OpenNXT(); %Connect to the NXT Brick via USB
COM_SetDefaultNXT(h);
OpenUltrasonic(SENSOR_1);
num_scan = 36;
reverse = 0;
iteration_correct = 1;

while iteration_correct <= 2
    half = -1;
    [ang_correct,readings] = sensor_rotate(num_scan);

    if ang_correct > 180
        half = 1;
        ang_correct = 360 - ang_correct;
    end
    correct = ang_correct / 180 * pi;
    turn(correct * half);

    if iteration_correct == 1 && correct == 0 && readings(19,1) > 10
        reverse = -(10);
    end
    iteration_correct = iteration_correct + 1;
end

% [~,readings] = sensor_rotate(8);
if readings(5,1) > readings(1,1) * 2 || readings(28,1) > readings(1,1) * 2
    reverse = -(10);
    move(reverse);
    
    half = -1;
    [~,readings] = sensor_rotate(num_scan);
    readings(1:8) = ones(8,1) .* 255;
    readings(29:36) = ones(8,1) .* 255;
    [~,smallest] = min(readings);
    ang_correct = (smallest-1)/num_scan* 360;
    
    if ang_correct > 180
        half = 1;
        ang_correct = 360 - ang_correct;
    end
    correct = ang_correct / 180 * pi;
    turn(correct * half);
end

map=[0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map
botSim = BotSim(map,[0,0,0]);
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%variables
max_step = 5; %maximum distance forward
iteration_move = 1; %number of iteration passed then move robot
num_scan = 4; %generate number of scans
num =500; % number of particles
maxNumOfIterations = 300; %maximum number of iteration before localisation stops
sigma = 3; %standard deviation for weightage probability
k = 0; %damping factor for weightage
add_in = num * 10/100; %number of extra particles which is randomly distributed in resampling
radii = sigma; %max radius of circle from a big weightage point
num_big_particle = 20; %number of biggest particle checked for convergence
percent_diff = 70/100; %percentage difference between particles
conv_pos_diff = radii * 2; %using maximum distance between big particles to identify clusters
max_est_loop = 3; %maximum number of loop the particle follow robot before fully converges
min_incluster = (num - add_in) * 20/100; %minimum number of particles together for a cluster
conv_parti_num = num * 65/100; %minimum number of particles in a cluster for convergence

%generate some random particles inside the map
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(particles(i).generateScanConfig(num_scan)); %number of scan per particle
    particles(i).setTurningNoise(0);
    particles(i).setMotionNoise(0);
    particles(i).setSensorNoise(0);
end

%% Localisation code
n = 0;
est_loop = 0; %count number of loop estimated particle follow robot
converged =0; %The localisation has not converged yet
correct_right = 0;
correct_left = 0;
correct_ang = 0;
correct_once = 1;
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    est_converge = 0; %the solution maybe converged if 1
    oldpos_particle = zeros(num,2); %position of all particles
    weight = zeros(num,1); %weightage for each particles
    norm_weight = zeros(num,1);
    n = n+1; %increment the current number of iterations
    moves = max_step; %movement forward
    bot_turn = 0; %robot turn to avoid obstacle
    %% Move robot and prevent robot going off map
%     if rem(n,iteration_move) == 0
%         botScan = sensor(num_scan); %get a scan from the real robot
        if n == 1
        [~,botScan] = sensor_rotate(num_scan);
        
        else
            if correct_right == 1
                turn(correct_ang);
                correct_right = 0;
                correct_once = 0;
            end
            
            if correct_left == 1
                turn(-correct_ang);
                correct_left = 0;
                correct_once = 0;
            end
        end

        %robot size right of , sensor of +/- 3cm, back of , front of 3cm
%         while botScan(1,1) <= moves + 15 || botScan(1,1) > 120%|| botScan(2,1) <= 6 || botScan(8,1) <= 6
%             [~,ang_far] = max(botScan);
%         wall_close = 0;
%         reverse = 0;
        far = 0;
        dist_side = 15;
        turn_ang = 0;
        if botScan(1,1) <= moves + 20 || botScan(1,1) > 120
            [~,far] = max(botScan(2:4,1));
            far = far + 1;
            if far == 4
%                 if botScan(2,1) < 23
%                     turn(-pi/2);
%                     reverse = -(25 - botScan(2,1));
%                     move(reverse);
%                     turn(pi);
%                     wall_close = 1;
%                 else
                    turn_ang = pi/2;
                    turn(turn_ang);
%                 end
            elseif far == 2
%                 if botScan(4,1) < 23
%                     turn(pi/2);
%                     reverse = -(25 - botScan(4,1));
%                     move(reverse);
%                     turn(-pi);
%                     wall_close = 1;
%                 else
                    turn_ang = -pi/2;
                    turn(turn_ang);
%                 end
            elseif far == 3
                turn_ang = pi;
                turn(turn_ang);
            end
            
            bot_turn = -turn_ang;
            correct_once = 1;
%             botScan = sensor(num_scan);
        end
        move(moves);
%         botScan = sensor(num_scan);
        [~,botScan] = sensor_rotate(num_scan);
        
        if n > 1 && correct_once == 1
            if botScan(2,1) < dist_side && abs(move_scan(2,1) - botScan(2,1)) < 10 && abs(move_scan(2,1) - botScan(2,1)) > 2
                correct_right = 1;
                correct_ang = atan((move_scan(2,1) - botScan(2,1))/moves);
            end
            
            if botScan(4,1) < dist_side && abs(move_scan(4,1) - botScan(4,1)) < 10 && abs(move_scan(4,1) - botScan(4,1)) > 2
                correct_left = 1;
                correct_ang = atan((move_scan(4,1) - botScan(4,1))/moves);
            end
        end
        move_scan = botScan;
    
    %% Write code for updating your particles scans
    % particles move -8cm if 180 deg turn
    % turn 90deg right, go right 4cm, back 3cm
    particleScan = zeros(num,num_scan);
    for i = 1:num
%         if rem(n,iteration_move) == 0
            if bot_turn ~= 0
                if far == 3
                    particles(i).move(-8);
                end
                
                %back 3cm
                if far == 2 || far == 4
                    particles(i).move(-3);
                end
                
                particles(i).turn(bot_turn);
                if far == 2 || far == 4
                    particles(i).move(4);
                end
            end
            
%             if wall_close == 1
%                 particles(i).move(-reverse);
%             end
            
            particles(i).move(moves);
            if particles(i).insideMap() == 0 %make sure particles are inside map after movement
                particles(i).randomPose(0); %randomly scatter particles from outside map into inside map
            end
%         end
        oldpos_particle(i,:) = particles(i).getBotPos(); %store position of all particles after movement
        particleScan(i,:) = particles(i).ultraScan();
    end
    
    %% Write code for scoring your particles
    particle_angle = zeros(num,1);
    for i = 1:num
        diff = zeros(num_scan,1);
        p_scan = particleScan(i,:);
        
        for j = 1:num_scan %finding the set of scans which is closest to robot set of scans
            diff(j,1) = min(abs(p_scan - botScan'));
            p_scan = circshift(particleScan(i,:),j); %cycle scan positions over number of scans to find best combination to robot scan
        end
        
        [~,pos] = min(diff); %find the combination of particle sensor which is closest to robot
        p_scan = circshift(particleScan(i,:),pos-1); %shift partile sensor readings to match closer to robot
        sensor_diff = (p_scan - botScan');
        sensor_diff = norm(sensor_diff);
        weight(i) = (1/(sqrt(2*pi*sigma^2))) * exp(-1 * sensor_diff^2 /(2*sigma^2)) + k;
        
        angle = (2 * pi / num_scan) * (pos-1);
        particles(i).turn(-angle); %turn the particle based on orientation of robot
        
        particle_angle(i,1) = particles(i).getBotAng(); %get orientation of all particles
        while particle_angle(i,1) > 2*pi %make sure the angle is within 360 degrees
            particle_angle(i,1) = particle_angle(i,1) - (2*pi);
        end
        
        while particle_angle(i,1) < 0 %make sure the angle is within 360 degrees
            particle_angle(i,1) = particle_angle(i,1) + (2*pi);
        end
        particles(i).setBotAng(particle_angle(i,1));
    end
    
    weight_sum = sum(weight);
    last_weight = 0;
    for i = 1:num
        weight(i) = (weight(i) / weight_sum); %normalise weightings
        norm_weight(i) = weight(i);
        weight(i) = weight(i) + last_weight; %cummulative weightings for roulette selection
        last_weight = weight(i); 
    end

    %% Write code for resampling your particles
    old_particle = zeros(num,1);
    for i = 1:(num-add_in) %iterate to assign identifier to each new particle
        key = rand(1);
        for j = 1:num %iterate to find which old particle match the new particle by matching weightings
            if j == 1
                if key <= weight(j) && key >= 0
                   old_particle(j) = old_particle(j) + 1;
                end
            else
              if key <= weight(j) && key > weight(j-1)
                  old_particle(j) = old_particle(j) + 1;
              end
            end
        end
    end
    
    %assign position of new particle at random position in the circle size
    %of old big particle, circle size grow with weightings up to a limit
    count_new = 0;
    for i = 1:num
        if old_particle(i) > 0
            for j = 1:old_particle(i)
                count_new = count_new + 1;
                new_particle = oldpos_particle(i,:);
                rand_ang = rand(1) * sigma * 2;
                rand_ang = rand_ang / 180 * pi;
                new_ang = particle_angle(i,1) - (sigma/180*pi) + rand_ang;
                while new_ang > 2*pi %make sure the angle is within 360 degrees
                    new_ang = new_ang - (2*pi);
                end
                
                while new_ang < 0 %make sure the angle is within 360 degrees
                    new_ang = new_ang + (2*pi);
                end
                
                particles(count_new).setBotAng(new_ang);
                
                inside = 0;
                while inside == 0 %ensure new particles are inside the map, if not, rerandom the position of new particles
                    rand_x_pos = rand(1) * radii * 2; %random number within the circle with radius rad, this is for x-coordinate
                    rand_y_pos = rand(1) * radii * 2;
                    unsure_particle(1) = (new_particle(1)-radii) + rand_x_pos;
                    unsure_particle(2) = (new_particle(2)-radii) + rand_y_pos;
                    particles(count_new).setBotPos(unsure_particle);
                    if particles(count_new).insideMap() == 1
                        inside = 1;
                    end
                end
            end
        end
    end
    
    %% Write code to check for convergence
    real_big = zeros(num_big_particle,1);
    real_big(1) = 1;
    est_posbot = NaN(num_big_particle,2);
    [weight_val,which_big] = maxk(norm_weight,num_big_particle); %find a few big weighted particles
    for i = 2:num_big_particle
        if abs(weight_val(1) - weight_val(i)) <= (weight_val(1) * percent_diff)
            real_big(i) = 1; %remove smaller particles in comparison with biggest particle
        else
            break
        end
    end
    
    num_real_big = sum(real_big);
    which_outside_cluster = zeros(num_big_particle,1);
    if num_real_big == 1 
        est_posbot(1,:) = oldpos_particle(which_big(1),:);
    else
        for i = 1:num_real_big %else check the position for big particles
            est_posbot(i,:) = oldpos_particle(which_big(i),:);
        end
        
        %check if big particles are in a single cluster
        est_posbot(any(isnan(est_posbot), 2),:)=[];
        for i = 2:size(est_posbot,1)
            if pdist([est_posbot(1,:);est_posbot(i,:)],'euclidean') > conv_pos_diff
                which_outside_cluster(i,1) = 1;
            end
        end
    end
    
    %look for positions of clusters if there are more than 1
    est_posbot(any(isnan(est_posbot), 2),:)=[];
    num_outcluster = sum(which_outside_cluster);
    cluster_point = zeros(1+num_outcluster,2);
    cluster_point(1,:) = est_posbot(1,:);
    if num_outcluster > 0
        for i = 2:size(est_posbot,1)
            if which_outside_cluster(i,1) == 1
                cluster_point(i,:) = est_posbot(i,:);
            end
        end
    end

    conv_parti_pos = zeros(num-add_in,2);
    conv_part_angle = zeros(num-add_in,1);
    for i = 1:num-add_in
        conv_parti_pos(i,:) = particles(i).getBotPos(); %get position of all particles except random ones
        conv_part_angle(i,1) = particles(i).getBotAng(); %get angle of all particles except random ones
    end
    
    num_incluster = zeros(size(cluster_point,1),1);
    for i = 1:size(cluster_point,1)
        for j = 1:num-add_in
            if pdist([cluster_point(i,:);conv_parti_pos(j,:)],'euclidean') <= conv_pos_diff
                num_incluster(i,1) = num_incluster(i,1) + 1; %count number of particles at a cluster point
            end
        end
    end
    
    cluster_num = 0;
    for i = 1:size(cluster_point,1)
        if num_incluster(i,1) > min_incluster %verify clusters exceed minimum number of particles
            cluster_num = cluster_num + 1;
        end
    end
    
    %if there is only 1 cluster of particles and exceed minimum number of particles in a 
    %cluster for convergence, estimated converge
%     if cluster_num == 1 && num_incluster(1,1) > conv_parti_num 
    if cluster_num == 1
        est_converge = 1; 
    end
    
    if est_converge == 1 %count the number of loop since estimated convergence start
        est_loop = est_loop + 1;
    else
        est_loop = 0;
    end
    
    if est_loop == max_est_loop
        converged = 1; %converged after big particles follow robot for a few loops
    end
    
    if converged == 1
        %look for position and orientation of particles in converged cluster
        count_conv_pos = 0;
        for i = 1:num-add_in
            if pdist([cluster_point(1,:);conv_parti_pos(i,:)],'euclidean') <= conv_pos_diff
                count_conv_pos = count_conv_pos + 1;
                cluster_particles(count_conv_pos,1:2) = conv_parti_pos(i,:);
                cluster_ang(count_conv_pos,1) = conv_part_angle(i,1);
            end
        end
        
        %remove discontinunity around 0 degrees when comparing quadrant 1
        %and 4 angles
        num_ang = size(cluster_ang,1);
        for i = 1:num_ang
            quad_1 = 0;
            if cluster_ang(i,1) >= 0 && cluster_ang(i,1) <= pi/2
                quad_1 = 1;
            end
            
            if cluster_ang(i,1) >= 3*pi/2 && cluster_ang(i,1) <= 2*pi && quad_1 == 1
                cluster_ang(i,1) = cluster_ang(i,1) - (2*pi);
            end
        end
        
        %estimated robot postion and orientation is the average position 
        %and orientation of all particles is the converged cluster
        est_botposition(1,1) = mean(cluster_particles(:,1),1);
        est_botposition(1,2) = mean(cluster_particles(:,2),1);
        est_botangle = median(cluster_ang(:,1),1);

        while est_botangle > 2*pi %make sure the angle is within 360 degrees
            est_botangle = est_botangle - (2*pi);
        end
        
        while est_botangle < 0 %make sure the angle is within 360 degrees
            est_botangle = est_botangle + (2*pi);
        end
    end
    
    for i = (num-add_in+1):num %selected few particles that are randomly scattered inside map
        particles(i).randomPose(0); 
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
%     if botSim.debug()
%         hold off; %the drawMap() function will clear the drawing when hold is off
%         botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
%         for i =1:num
%             particle_pos = particles(i).getBotPos();
%             plot(particle_pos(1),particle_pos(2),'Marker','.','Color',[0.4 0.4 0.8]);
%         end
% %         botSim.drawBot(4,'k'); %draw robot with line length 30 and green
%         drawnow;
%     end
end

% botSim.drawMap();
% botSim.setBotPos([est_botposition(1,1);est_botposition(1,2)]);
% botSim.setBotAng(est_botangle)
% botSim.drawBot(4,'k');
% est_botposition
% est_botangle = est_botangle/pi*180
%COM_CloseNXT(h); %Close the serial connection

map=[0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map
ang = 110;
%[bestPosX, bestPosY, currentAng] = relocalise([22,66], ang/180*pi, 4, map);
currentPos = [bestPosX, bestPosY];
nextPos = [22,88];
%changeAng2(currentAng,currentPos, nextPos)
start = [22, 22];
goal = [95,95];
mainBot = BotSim(map);
%est_botposition = start;
%est_botangle = 0;
astar(map, est_botposition, goal, mainBot, est_botangle)
%[bestPosX, bestPosY, bestAng] = relocalise(start, 0.2, 4, map);
COM_CloseNXT(h);

function [weight, optimumAngle] = getWeight(particleScanDistances,SCAN_NUMBER, botSimScanDistances)
    var = 80;
        
    weightForShift = zeros(SCAN_NUMBER, 1);
    
    eucledianDistance = sqrt(sum((botSimScanDistances-particleScanDistances).^2));
    max_weight = (1/sqrt(2*pi*var))*exp(-((eucledianDistance)^2/(2*var)));
    weight = max_weight;
    optimumAngle = 0;
end

function [bestPosX, bestPosY, bestAng] = relocalise(expectedPos, expectedAng, SCAN_NUMBER, map)
    particles = 1500;
    posVar = 4;
    angleVar = 0.25;
    botSim = BotSim(map);
    botSim.setBotPos(expectedPos)
    botSim.setBotAng(expectedAng)
    botSim.setScanConfig(botSim.generateScanConfig(SCAN_NUMBER));
    [botSimScanDistances1, cp] = botSim.ultraScan();
    %scan here
    [ang_correct,botSimScanDistances] = sensor_rotate(SCAN_NUMBER);
    for i=1:particles
        mx = normrnd(0, posVar);
        my = normrnd(0, posVar);
        ma = normrnd(0, angleVar);
        newParticlePoint = [expectedPos(1) + mx, expectedPos(2) + my];
        if(botSim.pointInsideMap(newParticlePoint))
            simulatedParticle(i) = BotSim(map);
            simulatedParticle(i).setBotPos(newParticlePoint)
            simulatedParticle(i).setBotAng(expectedAng + ma);
            simulatedParticle(i).setScanConfig(simulatedParticle(i).generateScanConfig(SCAN_NUMBER));
            simulatedParticle(i).drawBot(0);
            [particleScanDistances, cp] = simulatedParticle(i).ultraScan();  
            for j=1:SCAN_NUMBER
                if(botSimScanDistances(j) > 90)
                    particleScanDistances(j) =  botSimScanDistances(j);
                end
                if(botSimScanDistances(j) == -1)
                    particleScanDistances(j) = -1;
                end
            end
            [max_weight, optimumAngle] = getWeight(particleScanDistances,SCAN_NUMBER, botSimScanDistances);
            unnormalisedWeights(i) = max_weight;
            simulatedParticle(i).setBotAng(simulatedParticle(i).getBotAng())
        end
    end
        
    weight = unnormalisedWeights./sum(unnormalisedWeights);
    x_estimate = 0;
    y_estimate = 0;
    ang_estimate = 0;
    for i = 1:length(weight)
       coords = simulatedParticle(i).getBotPos();
       x_estimate = x_estimate + weight(i) * coords(1);
       y_estimate = y_estimate + weight(i) * coords(2);
       ang_estimate = ang_estimate + weight(i) * simulatedParticle(i).getBotAng();
    end
    [bestParticleWeight, bestParticleIndex] = max(weight);
    bestPos = simulatedParticle(bestParticleIndex).getBotPos();
    bestAng = mod(ang_estimate, 2*pi);%mod(simulatedParticle(bestParticleIndex).getBotAng(), 2*pi);
    bestPosX = x_estimate;%bestPos(1);
    bestPosY = y_estimate;%bestPos(2);
    disp("Best Ang")
    disp(bestAng/(pi) * 180)
    disp("Best Pos")
    disp(bestPos)
    disp("STEP")
end


function astar(map, start, goal, mainBot, angleStart)
    limsMin = min(map);
    limsMax = max(map);
    dims = limsMax-limsMin;
    pointGridResolution = 5;
    numberOfPointsXY = dims/pointGridResolution;
    [largerGirdPoints, pos] = max(numberOfPointsXY);
    numberOfPoints = ceil(numberOfPointsXY(pos));

    pointsMap = createPointsMap2(mainBot, goal, start);
    
    [startIndex, ang] = convergeToStart(pointsMap, start, angleStart);
    %startIndex = findNearestIndex(start, pointsMap);
    goalIndex = findNearestIndex(goal, pointsMap);
    
    path = calcPath(startIndex, goalIndex, pointsMap, numberOfPoints, pointGridResolution);
    for i = 1:length(path)
        disp(pointsMap(path(i)).coord)
    end
    [currentAng, currentPos] = moveToGoal(path, pointsMap,map, ang);
    %add localise again
    
    %move to goal
    nextPos = goal;
    currentAng = changeAng(currentAng, currentPos, nextPos);
    d = calcDist(currentPos, nextPos);
    
    move(d)
    %mainBot.move(dist);
    %mainBot.drawBot(0);
        
end

function d = calcDist(start, goal)
    d = sqrt((goal(2)-start(2))^2 + (goal(1)-start(1))^2);
end

function [startIndex, angle] = convergeToStart(pointsMap, bestPos, bestAng)
    
    pointsMapIndex = findNearestIndex(bestPos, pointsMap);
    disp(pointsMap(pointsMapIndex).coord)
    
    angle = changeAng(bestAng, bestPos, pointsMap(pointsMapIndex).coord);
    dist = calcDist(bestPos, pointsMap(pointsMapIndex).coord);
    disp(angle)
    startIndex = pointsMapIndex;
    move(dist)
    
    %robot.move(dist);
    %robot.turn(-angle);
end

function nearestIndex = findNearestIndex(coord, pointsMap)
    distances = zeros(length(pointsMap), 1);
    for i = 1:length(pointsMap)
        if(pointsMap(i).canTraverse == 1)
            distances(i,:) = calcDist(coord, pointsMap(i).coord); 
        else
             distances(i,:) = inf;
        end
    end
    [d, nearestIndex] = min(distances);
end

function h = calcH(pos, goal, res)
   xDif = abs(goal(1) - pos(1));
   yDif = abs(goal(2) - pos(2)); 
   differnce = xDif - yDif;
   if(differnce > 0)
       diagMoves = xDif;
       gridMoves = differnce;
   else
       diagMoves = yDif;
       gridMoves = -1*differnce;
   end    
   h = diagMoves*diagDistance(res) + gridMoves*res;
end

function indexes = findAdjIndexes(index, numberOfPoints)
    if index == 1
        indexes = [2];
    elseif index == 8
        indexes = [7];
    else 
        indexes = [index - 1 , index + 1];
    end
end 


function path = calcPath(startIndex, goalIndex, pointsMap, numberOfPoints, res)
    
    currentIndex = startIndex;
    pointsMap(startIndex).gscore = 0;
    pointsMap(startIndex).explored = 1;
    pointsMap(startIndex).shortestPath(1) = currentIndex; 
      
    while(currentIndex ~= goalIndex)      
        pointsMap = calcG(pointsMap, currentIndex, numberOfPoints, res);
        currentIndex = findNextPointToExplore(pointsMap);      
        pointsMap(currentIndex).explored = 1; 
    end
    path = pointsMap(goalIndex).shortestPath;
    
end

function minFScoreIndex = findNextPointToExplore(pointsMap)
    minFScore = inf;
    minFScoreIndex = 0;
    for i = 1: length(pointsMap)
        if pointsMap(i).explored == 0 && pointsMap(i).hscore + pointsMap(i).gscore < minFScore
            minFScoreIndex = i;
            minFScore = pointsMap(i).hscore + pointsMap(i).gscore;
        end
    end
end

function pointsMap = calcG(pointsMap, currentIndex, numberOfPoints, res)
    adjIndexes = findAdjIndexes(currentIndex ,numberOfPoints);
    for i = 1: length(adjIndexes)
        adjIndex = adjIndexes(i);
        if pointsMap(adjIndex).explored == 0 && pointsMap(adjIndex).canTraverse == 1                
            newGScore = gScoreForPoint(pointsMap, adjIndex, res, currentIndex, numberOfPoints);           
            if(newGScore < pointsMap(adjIndex).gscore) 
                pointsMap(adjIndex).gscore = newGScore;
                pointsMap(adjIndex).shortestPath = pointsMap(currentIndex).shortestPath;
                pointsMap(adjIndex).shortestPath(length(pointsMap(currentIndex).shortestPath) + 1) = adjIndex;
            end
        end
    end 
end

function g = gScoreForPoint(pointsMap, adjIndex, res, currentIndex, numberOfPoints)
    point1x = pointsMap(currentIndex).coord(1);
    point1y = pointsMap(currentIndex).coord(2);
    point2x = pointsMap(adjIndex).coord(1);
    point2y = pointsMap(adjIndex).coord(2);
    moveCost = abs(point1x - point2x) + (point1y - point2y);
    g = pointsMap(currentIndex).gscore + moveCost;
end


function pointsMap = createPointsMap2(robot, goal,  bestPos)
    index = 1;
    i = 1;
    x1 = 22;
    y1 = 22;
    positions = [[x1 + 22,y1], [x1,y1], [x1,y1 + 22], [x1,y1 + 44], [x1,y1 + 66],[x1 + 22, y1 + 66], [x1 + 44,y1 + 66], [x1 + 66,y1 + 66]];
    while i < length(positions)
        p1 = positions(i);
        p2 = positions(i + 1);
        i = i + 2;
        pos = [p1, p2];
        pointsMap(index) = createPoint(robot, goal,bestPos, 1 ,pos);
        index = index + 1;
        plot(pos(1),pos(2),'O')
    end
end

function point = createPoint(robot, goal,bestPos, res, position)
    point.coord = position;
    point.explored = 0;
    point.gscore = inf;
    point.hscore = calcH(bestPos, goal, res);
    point.canTraverse = robot.pointInsideMap(position);
end

function angle = changeAng(currentAng, currentPos, nextPos)
    
    if currentPos(1) ~= nextPos(1) || currentPos(2) ~= nextPos(2)
        angle = mod(atan2((nextPos(2)-currentPos(2)),(nextPos(1)-currentPos(1))), 2*pi);
        disp("turn")
        disp((-angle + currentAng)/ pi *180)
        if angle ~= currentAng
            turn(-angle + currentAng);
        end
    else
        angle = currentAng;
    end
end
function angle = changeAng2(currentAng, currentPos, nextPos)
    
    if currentPos(1) ~= nextPos(1) || currentPos(2) ~= nextPos(2)
        angle = mod(atan2((nextPos(2)-currentPos(2)),(nextPos(1)-currentPos(1))), 2*pi);
        disp("current")
        disp(currentAng/ pi *180)
        disp("to face")
        disp(angle/ pi *180)
        if angle ~= currentAng
            disp("turn")
            disp(currentAng - angle)
            turn(currentAng - angle);
        end
    else
        angle = currentAng;
    end
end

function [currentAng,currentPos] = moveToGoal(path, pointsMap, map, ang)
    currentAng = ang;
    currentPos = pointsMap(path(1)).coord;
    for i = 2:length(path)
        nextPos = pointsMap(path(i)).coord;
        currentAng = changeAng2(currentAng, currentPos, nextPos);
        d = calcDist(currentPos, nextPos);
        move(d)
        [bestPosX, bestPosY, currentAng] = relocalise(nextPos, currentAng, 4, map);
        currentPos = [bestPosX, bestPosY];  
        disp("Angle")
        disp(currentAng/180 * pi)
        disp("Current")
        disp(currentPos)
    end
end

function d = diagDistance(res)
    d = sqrt(2*res^2);
end
function move(d)
    if d ~= 0
        rotation = round(abs((d/10.1) * 360));
        motor = NXTMotor('AC') ;
        if d > 0
            motor.Power = 70;
        else
            motor.Power = -70;
        end
        motor.TachoLimit = rotation;
        motor.SendToNXT()
        motor.WaitFor()
    end
end

function turn(ang)
    if ang ~= 0
    if ang > pi
        ang = ang - (2*pi);
    end
    
    if ang < -pi
        ang = abs(ang) - (2*pi);
    end
    
%     rotation = round(ang/pi*931.6);
    
    rotation = round(ang/pi*931.6);
    motor = NXTMotor('A') ;
    motor2 = NXTMotor('C') ;
    %disp(ang)
    %disp(rotation)
    power = 40;
    if (rotation > 0)
        motor.Power = -power; 
        motor2.Power = power; 
    else
        motor.Power = power; 
        motor2.Power = -power;
        rotation = abs(rotation);
    end
    %disp(rotation)
    motor.TachoLimit = rotation;
    motor2.TachoLimit = rotation;
    motor.SendToNXT()
    motor2.SendToNXT()
    motor.WaitFor()
    motor2.WaitFor()
    end
end

function h = init()
    COM_CloseNXT all %Close any left over connections to the NXT
    h =COM_OpenNXT(); %Connect to the NXT Brick via USB
    COM_SetDefaultNXT(h); %Set our connection as the default
    OpenUltrasonic(SENSOR_1)
end

function [ang_correct,readings] = sensor_rotate(num_scan)
d = GetUltrasonic(SENSOR_1);
d = GetUltrasonic(SENSOR_1);

readings = zeros(num_scan,1);
readings(1,1) = d;

drivingPower = 30;
returnPower = -50;
dist  = (round(360/num_scan) * (num_scan - 1)) + 3; % in degrees
sensor_motor = MOTOR_B;

%define motor to tuen sensor
m_sensor = NXTMotor(sensor_motor, 'Power', drivingPower);
m_sensor.TachoLimit = dist;

% prepare motor
m_sensor.Stop('off');
m_sensor.ResetPosition();

data = m_sensor.ReadFromNXT();
pos  = data.Position;
m_sensor.SendToNXT()
num = 1;
Degree = round(360/num_scan);
diff = round(360/num_scan);
while pos <= dist - 1
    data = m_sensor.ReadFromNXT();
    pos  = data.Position;
    if pos == Degree 
        num = num + 1;
        readings(num,1) = GetUltrasonic(SENSOR_1);
        Degree = Degree + diff;
    end
    
    if num == num_scan
        break
        
%     elseif pos == dist - 1 && num == num_scan - 1
%         num = num + 1;
%         readings(num,1) = GetUltrasonic(SENSOR_1);
%         break
    end
end
readings=readings(1:num_scan);
m_sensor.WaitFor()
% rotate_lim = dist * (num_scan -1) + pos;
% m_sensor.TachoLimit = rotate_lim;

%define motor to return to original position
sensor_return = NXTMotor(sensor_motor, 'Power', returnPower);
data = sensor_return.ReadFromNXT();
pos  = data.Position;
sensor_return.TachoLimit = pos;

sensor_return.SendToNXT()
sensor_return.WaitFor()
[~,smallest] = min(readings);
ang_correct = (smallest-1)/num_scan* 360;
end