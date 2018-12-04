map = [0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110]; %default map
start = [22,88];
goal = [44,22];
mainBot = BotSim(map);
mainBot.drawMap()
h = init();
%relocalise([88,88], 0, 36, map)
astar(map, start, goal, mainBot)
%finish(h)

function [weight, optimumAngle] = getWeight(particleScanDistances,SCAN_NUMBER, botSimScanDistances)
    var = 80;
        
    weightForShift = zeros(SCAN_NUMBER, 1);
    
    eucledianDistance = sqrt(sum((botSimScanDistances-particleScanDistances).^2));
    max_weight = (1/sqrt(2*pi*var))*exp(-((eucledianDistance)^2/(2*var)));
    weight = max_weight;
    optimumAngle = 0;
end

function [bestPosX, bestPosY, bestAng] = relocalise(expectedPos, expectedAng, SCAN_NUMBER, map)
    particles = 1000;
    posVar = 4;
    angleVar = 0.2;
    botSim = BotSim(map);
    botSim.setBotPos(expectedPos)
    botSim.setScanConfig(botSim.generateScanConfig(SCAN_NUMBER));
    [botSimScanDistances1, cp] = botSim.ultraScan();
    %disp("Correct")
    %disp(botSimScanDistances1)
    %scan here
    [ang_correct,botSimScanDistances] = sensor_rotate(SCAN_NUMBER);
    %disp("Got")
    %disp(botSimScanDistances)
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
                if(botSimScanDistances(j) == 255)
                    particleScanDistances(j) = 255;
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
    [bestParticleWeight, bestParticleIndex] = max(weight);
    bestPos = simulatedParticle(bestParticleIndex).getBotPos();
    bestAng = mod(simulatedParticle(bestParticleIndex).getBotAng(), 2*pi);
    bestPosX = bestPos(1);
    bestPosY = bestPos(2);
    disp("Best Ang")
    disp(bestAng/(pi) * 180)
end


function astar(map, start, goal, mainBot)
    limsMin = min(map);
    limsMax = max(map);
    dims = limsMax-limsMin;
    pointGridResolution = 5;
    numberOfPointsXY = dims/pointGridResolution;
    [largerGirdPoints, pos] = max(numberOfPointsXY);
    numberOfPoints = ceil(numberOfPointsXY(pos));

    pointsMap = createPointsMap2(mainBot, goal, start);
    
    startIndex = convergeToStart(pointsMap, start, 0);
    %startIndex = findNearestIndex(start, pointsMap);
    goalIndex = findNearestIndex(goal, pointsMap);
    
    path = calcPath(startIndex, goalIndex, pointsMap, numberOfPoints, pointGridResolution);
    for i = 1:length(path)
        disp(pointsMap(path(i)).coord)
    end
    currentAng = moveToGoal(path, pointsMap,map);
    %add localise again
    
    %move to goal
    currentPos = pointsMap(path(length(path) - 1)).coord;
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

function startIndex = convergeToStart(pointsMap, bestPos, bestAng)
    
    pointsMapIndex = findNearestIndex(bestPos, pointsMap);
    disp(pointsMap(pointsMapIndex).coord)
    
    angle = changeAng(bestAng, bestPos, pointsMap(pointsMapIndex).coord);
    dist = calcDist(bestPos, pointsMap(pointsMapIndex).coord);
    %disp(angle)
    startIndex = pointsMapIndex;
    move(dist)
    turn(-angle)
    
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
    angle = mod(atan2((nextPos(2)-currentPos(2)),(nextPos(1)-currentPos(1))), 2*pi);
    disp("turn")
    disp((angle - currentAng)/ pi *180)
    if angle ~= currentAng
        turn(-angle + currentAng);
    end
end

function currentAng = moveToGoal(path, pointsMap, map)
    currentAng = 0;
    disp(currentAng)
    currentPos = pointsMap(path(1)).coord;
    for i = 2:length(path)
        nextPos = pointsMap(path(i)).coord;
        currentAng = changeAng(currentAng, currentPos, nextPos);
        d = calcDist(currentPos, nextPos);
        move(d)
        [bestPosX, bestPosY, currentAng] = relocalise(nextPos, currentAng, 36, map);
        currentPos = [bestPosX, bestPosY];   
        disp("Current")
        disp(currentPos)
    end
end

function d = diagDistance(res)
    d = sqrt(2*res^2);
end
function move(d)
    rotation = round((d/13.21) * 360);
    motor = NXTMotor('AC') ;
    motor.Power = -100; 
    motor.TachoLimit = rotation;
    motor.SendToNXT()
    motor.WaitFor()
end

function turn(ang)
    disp(ang)
    if ang ~= 0
        if(ang > pi)
            ang = pi - ang;
        end
        if(ang < -pi)
            ang = -pi - ang;
        end
    rotation = round(ang/(2*pi)*800);
    
    motor = NXTMotor('A') ;
    motor2 = NXTMotor('C') ;
    %disp(ang)
    %disp(rotation)
    if (rotation > 0)
        motor.Power = 50; 
        motor2.Power = -50; 
        
    else
        motor.Power = -50; 
        motor2.Power = 50;
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
dist  = round(360/num_scan) * (num_scan - 1); % in degrees
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
while pos <= dist + 2
    data = m_sensor.ReadFromNXT();
    pos  = data.Position;
    if pos == Degree 
        num = num + 1;
        readings(num,1) = GetUltrasonic(SENSOR_1);
        Degree = Degree + diff;
    end
    
    if num == num_scan
        break
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