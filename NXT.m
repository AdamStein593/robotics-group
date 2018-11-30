map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
goal = [20,20];
start = [80,80];
mainBot = BotSim(map);
mainBot.drawMap()
createPointsMap2(mainBot, goal,  start)
%h = init();
%rotateSensor();
%astar(map, start, goal, mainBot)
%finish(h)

function astar(map, start, goal, mainBot)
    limsMin = min(map);
    limsMax = max(map);
    dims = limsMax-limsMin;
    pointGridResolution = 5;
    numberOfPointsXY = dims/pointGridResolution;
    [largerGirdPoints, pos] = max(numberOfPointsXY);
    numberOfPoints = ceil(numberOfPointsXY(pos));

    pointsMap = createPointsMap(mainBot, goal, numberOfPoints, pointGridResolution, 3, start);
    
    startIndex = findNearestIndex(start, pointsMap);
    goalIndex = findNearestIndex(goal, pointsMap);
    
    path = calcPath(startIndex, goalIndex, pointsMap, numberOfPoints, pointGridResolution);
    currentAng = moveToGoal(mainBot, path, pointsMap, pointGridResolution);
              
    %mainBot.move(dist);
    %mainBot.drawBot(0);
        
end

function d = calcDist(start, goal)
    d = sqrt((goal(2)-start(2))^2 + (goal(1)-start(1))^2);
end

function startIndex = convergeToStart(robot, pointsMap, bestPos, bestAng)
    
    pointsMapIndex = findNearestIndex(bestPos, pointsMap);
    
    angle = changeAng(robot, bestAng, bestPos, pointsMap(pointsMapIndex).coord);
    dist = calcDist(bestPos, pointsMap(pointsMapIndex).coord);

    startIndex = pointsMapIndex;
    robot.move(dist);
    robot.turn(-angle);
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
    bottom = index <= numberOfPoints;
    top = index >= (numberOfPoints - 1)*numberOfPoints;
    left = mod(index, numberOfPoints) == 1;
    right = mod(index, numberOfPoints) == 0;
    
    indexUpLeft = index + numberOfPoints - 1;
    indexUp = index + numberOfPoints;
    indexUpRight = index + numberOfPoints + 1;
    indexLeft = index - 1;
    indexRight = index + 1;
    indexDownLeft = index - numberOfPoints - 1;
    indexDown = index - numberOfPoints;
    indexDownRight = index - numberOfPoints + 1;

    if bottom && left     
        indexes = [indexRight, indexUp, indexUpRight];
    elseif bottom && right
        indexes = [indexLeft, indexUp, indexUpLeft];
    elseif top && left
        indexes = [indexRight, indexDown, indexDownRight];
    elseif top && right
        indexes = [indexLeft, indexDown, indexDownLeft];
    elseif bottom
        indexes = [indexRight, indexLeft, indexUp, indexUpLeft, indexUpRight];
    elseif top
        indexes = [indexRight, indexLeft, indexDown, indexDownLeft, indexDownRight];
    elseif right
        indexes = [indexLeft, indexUpLeft, indexDownLeft, indexUp, indexDown];            
    elseif left
    	indexes = [indexRight, indexUpRight, indexDownRight, indexUp, indexDown];    
    else
        indexes = [indexLeft, indexRight, indexUpLeft, indexDownLeft, indexUpRight, indexDownRight, indexUp, indexDown];
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
    moveCost = res;
    diagMove = abs(adjIndex - currentIndex) ~=1 && abs(adjIndex - currentIndex) ~= numberOfPoints;
    if diagMove
        moveCost = diagDistance(res);
    end
    g = pointsMap(currentIndex).gscore + moveCost;
end

function paddedPointsMap = applyPad(pointsMap, numberOfPoints)
    for i = 1:length(pointsMap)
        if(pointsMap(i).canTraverse == 1)
           adjIndexes = findAdjIndexes(i, numberOfPoints);
           pointsMap(i).canTraverseTemp = 1;
           for j = 1:length(adjIndexes)
               if(pointsMap(adjIndexes(j)).canTraverse == 0)
                   pointsMap(i).canTraverseTemp = 0;
               end
           end
        else
            pointsMap(i).canTraverseTemp = 0;
        end
    end
    for i = 1:length(pointsMap)
        pointsMap(i).canTraverse = pointsMap(i).canTraverseTemp;
    end
    paddedPointsMap = pointsMap;
end

function pointsMap = createPointsMap(robot, goal, numberOfPoints, res, pads, bestPos)
    index = 1;
    for i = 1:numberOfPoints
        for j = 1:numberOfPoints
            pointsMap(index) = createPoint(robot, goal,bestPos, res,[i * res, j * res]);
            index = index + 1;
        end
    end
    for i = 1:pads
        pointsMap = applyPad(pointsMap, numberOfPoints);
    end    
end

function pointsMap = createPointsMap2(robot, goal,  bestPos)
    index = 1;
    i = 1;
    x1 = 25;
    positions = [[x1,20], [x1,40], [x1,60], [x1,80],[x1 + 30, 80], [x1 + 60,80]];
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

function angle = changeAng(robot, currentAng, currentPos, nextPos)
    angle = atan2((nextPos(2)-currentPos(2)),(nextPos(1)-currentPos(1)));
    if angle ~= currentAng
        turn(angle - currentAng);
    end
end

function currentAng = moveToGoal(robot, path, pointsMap, res)
    currentAng = 0;
    currentPos = pointsMap(path(1)).coord;
    for i = 1:length(path)
        nextPos = pointsMap(path(i)).coord;
        currentAng = changeAng(robot, currentAng, currentPos, nextPos);
        straightMove = mod(currentAng, pi/2) == 0;
        if straightMove
            move(res);
        else
            move(diagDistance(res));           
        end
        
        robot.drawBot(0);
        currentPos = nextPos;      
    end
end

function d = diagDistance(res)
    d = sqrt(2*res^2);
end
function move(d)
    rotation = round((d/13.21) * 360);
    motor = NXTMotor('AC') ;
    motor.Power = 100; 
    motor.TachoLimit = rotation;
    motor.SendToNXT()
    motor.WaitFor()
end
function turn(ang)
    rotation = round(ang/(2*pi)*1000);
    
    motor = NXTMotor('A') ;
    motor2 = NXTMotor('C') ;
    disp(ang)
    disp(rotation)
    if (rotation > 0)
        motor.Power = -50; 
        motor2.Power = 50; 
    else
        motor.Power = 50; 
        motor2.Power = -50;
        rotation = abs(rotation);
    end
    disp(rotation)
    motor.TachoLimit = rotation;
    motor2.TachoLimit = rotation;
    motor.SendToNXT()
    motor2.SendToNXT()
    motor.WaitFor()
    motor2.WaitFor()
    
end

function h = init()
    COM_CloseNXT all %Close any left over connections to the NXT
    h =COM_OpenNXT(); %Connect to the NXT Brick via USB
    COM_SetDefaultNXT(h); %Set our connection as the default
    OpenUltrasonic(SENSOR_4)
end

function rotateSensor()
    d = GetUltrasonic(SENSOR_4);
    disp(d)
    scanNumber = 4;
    motor = NXTMotor('B') ;
    motor.Power = 100;
    rotation = round(360/ scanNumber);
    totalRotation = rotation * (scanNumber -1);
    for i=1:scanNumber -1
        motor.TachoLimit = rotation;
        motor.SendToNXT()
        motor.WaitFor()
        d = GetUltrasonic(SENSOR_4);
        disp(d)

    end
    motor.Power = -100;
    motor.TachoLimit = totalRotation;
    motor.SendToNXT()
    motor.WaitFor()
    
end

function finish(h)
    COM_CloseNXT(h)
end