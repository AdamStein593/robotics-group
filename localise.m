function [botSim] = localise(botSim,map,target)
    %This function returns botSim, and accepts, botSim, a map and a target.
    %LOCALISE Template localisation function

    %% setup code
    %you can modify the map to take account of your robots configuration space
    modifiedMap = map; %you need to do this modification yourself
    botSim.setMap(modifiedMap);

    [bestPosX, bestPosY, bestAng] = particleFilter(botSim, modifiedMap, [-1,-1,-1, -1]);
    bestPos = [bestPosX, bestPosY];
    astar(modifiedMap, target, botSim, bestPos, bestAng)
end

function [bestPosX, bestPosY, bestAng] = particleFilter(botSim, modifiedMap,knownInfo)
    SCAN_NUMBER = 64;
    botSim.setScanConfig(botSim.generateScanConfig(SCAN_NUMBER));
    
    limsMin = min(modifiedMap);
    limsMax = max(modifiedMap);
    dims = limsMax-limsMin;
    
    %generate some random particles inside the map
    numberOfPoints = ceil((dims(1)/5) * (dims(2)/5)); % number of particles
    simulatedParticle(numberOfPoints,1) = BotSim;
    
    if(knownInfo(1) == -1)
        convergence_threshold = 0.6;
        moveDivideFactor = 5;
        maxNumOfIterations = 30;
        for i = 1:numberOfPoints
            simulatedParticle(i) = BotSim(modifiedMap); 
            simulatedParticle(i).randomPose(0); 
            simulatedParticle(i).setScanConfig(simulatedParticle(i).generateScanConfig(SCAN_NUMBER))
            simulatedParticle(i).setBotAng(rand(1)*2*pi);
            
        end
    else
        convergence_threshold = 0.6;
        moveDivideFactor = 40;
        maxNumOfIterations = 10;
        pathLength = knownInfo(4); 
        sampleVariancePos = pathLength*0.1;
        sampleVarianceAng = pathLength*0.01;
        for i=1:numberOfPoints
            baseCoords = [knownInfo(1), knownInfo(2)];
            baseAng = knownInfo(3);
            mx = normrnd(0, sampleVariancePos);
            my = normrnd(0, sampleVariancePos);
            ma = normrnd(0, sampleVarianceAng);
            newParticlePoint = [baseCoords(1) + mx, baseCoords(2) + my];
            if(botSim.pointInsideMap(newParticlePoint))
                simulatedParticle(i) = BotSim(modifiedMap);
                simulatedParticle(i).setBotPos(newParticlePoint)
                simulatedParticle(i).setBotAng(baseAng + ma);
                simulatedParticle(i).setScanConfig(simulatedParticle(i).generateScanConfig(SCAN_NUMBER));
            end

        end
    end
    
    n = 0;
    
    converged =0; 
    while(converged == 0 && n < maxNumOfIterations) 
        n = n+1; 
        [botSimScanDistances, crossingPoint] = botSim.ultraScan();
        if(numberOfPoints == 0)
            numberOfPoints = ceil((dims(1)/5) * (dims(2)/5)); 
            simulatedParticle(numberOfPoints,1) = BotSim;
            for i = 1:numberOfPoints
                simulatedParticle(i) = BotSim(modifiedMap);  
                simulatedParticle(i).randomPose(0); 
                simulatedParticle(i).setScanConfig(simulatedParticle(i).generateScanConfig(SCAN_NUMBER))
                simulatedParticle(i).setBotAng(rand(1)*2*pi);
            end
        end
        %% Write code for updating your particles scans
        unnormalisedWeights = zeros(numberOfPoints, 1);
        for i=1:numberOfPoints
            [particleScanDistances, cp] = simulatedParticle(i).ultraScan();  
        %% Write code for scoring your particles    
            [max_weight, optimumAngle] = getWeight(particleScanDistances,SCAN_NUMBER, botSimScanDistances);
            unnormalisedWeights(i) = max_weight;
            simulatedParticle(i).setBotAng(simulatedParticle(i).getBotAng() +  optimumAngle)
        end
        weight = unnormalisedWeights./sum(unnormalisedWeights);
        %% Write code for resampling your particles
        sampleVariancePos = 0.3;
        sampleVarianceAng = 0.02;
        newParticlesIndex = 1;
        for i=1:numberOfPoints
            baseCoords = simulatedParticle(i).getBotPos();
            baseAng = simulatedParticle(i).getBotAng();
            sampleNum =  round(weight(i) * numberOfPoints);
            for j=1:sampleNum
                mx = normrnd(0, sampleVariancePos);
                my = normrnd(0, sampleVariancePos);
                ma = normrnd(0, sampleVarianceAng);
                newParticlePoint = [baseCoords(1) + mx, baseCoords(2) + my];
                if(botSim.pointInsideMap(newParticlePoint))
                    newSimedParticle(newParticlesIndex) = BotSim(modifiedMap);
                    newSimedParticle(newParticlesIndex).setBotPos(newParticlePoint)
                    newSimedParticle(newParticlesIndex).setBotAng(baseAng + ma);
                    newSimedParticle(newParticlesIndex).setScanConfig(newSimedParticle(newParticlesIndex).generateScanConfig(SCAN_NUMBER));
                    newParticlesIndex = newParticlesIndex + 1; 
                end

            end
        end
        numberOfPoints = newParticlesIndex - 1;
        simulatedParticle = newSimedParticle; 
        %% Write code to check for convergence   
        positions = zeros(numberOfPoints, 2);   
        for i=1:numberOfPoints
            positions(i,:) = simulatedParticle(i).getBotPos();
        end
        stdev = std(positions);
        if(stdev(1) < convergence_threshold && stdev(2) < convergence_threshold)
            converged = 1;
        end
        %% Write code to decide how to move next
        % here they just turn in cicles as an example
        [furthestWallDistance, scanPosition] = max(botSimScanDistances);
        angle = scanPosition*2*pi/SCAN_NUMBER;
        distanceToMove = furthestWallDistance/moveDivideFactor;
        botSim.turn(angle);
        botSim.move(distanceToMove);
        botSim.drawBot(0);
        for i=1:numberOfPoints
            simulatedParticle(i).turn(angle);
            simulatedParticle(i).move(distanceToMove);
        end
       
        %% Drawing
        %only draw if you are in debug mode or it will be slow during marking
        if botSim.debug()
            hold off; %the drawMap() function will clear the drawing when hold is off
            botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
            botSim.drawBot(30,'g'); %draw robot with line length 30 and green
            for i =1:numberOfPoints
                simulatedParticle(i).drawBot(3); %draw particle with line length 3 and default color
            end
            drawnow;
        end
        
    end

    [bestParticleWeight, bestParticleIndex] = max(weight);
    bestPos = simulatedParticle(bestParticleIndex).getBotPos();
    bestAng = mod(simulatedParticle(bestParticleIndex).getBotAng(), 2*pi);
    bestPosX = bestPos(1);
    bestPosY = bestPos(2);
end

function [weight, optimumAngle] = getWeight(particleScanDistances,SCAN_NUMBER, botSimScanDistances)
    var = 80;
    weightForShift = zeros(SCAN_NUMBER, 1);
    for j=1:SCAN_NUMBER
        shiftDistances = circshift(botSimScanDistances,j);
        eucledianDistance = sqrt(sum((shiftDistances-particleScanDistances).^2));
        weightForShift(j) = (1/sqrt(2*pi*var))*exp(-((eucledianDistance)^2/(2*var)));
    end
    [max_weight, max_pos] = max(weightForShift);
    weight = max_weight;
    optimumAngle = max_pos*2*pi/SCAN_NUMBER;
end

function astar(map, goal, mainBot, bestPos, bestAng)
    limsMin = min(map);
    limsMax = max(map);
    dims = limsMax-limsMin;
    pointGridResolution = 3;
    numberOfPointsXY = dims/pointGridResolution;
    [largerGirdPoints, pos] = max(numberOfPointsXY);
    numberOfPoints = ceil(numberOfPointsXY(pos));

    pointsMap = createPointsMap(mainBot, goal, numberOfPoints, pointGridResolution, 3, bestPos);

    startIndex = convergeToStart(mainBot, pointsMap, bestPos, bestAng);
    goalIndex = findNearestIndex(goal, pointsMap);

    path = calcPath(startIndex, goalIndex, pointsMap, numberOfPoints, pointGridResolution);

    currentAng = moveToGoal(mainBot, path, pointsMap, pointGridResolution);
    
    [bestPosX, bestPosY, bestAng] = particleFilter(mainBot, map, [goal(1),goal(2),currentAng, length(path)]);
      
    bestPos = [bestPosX, bestPosY];
    
    changeAng(mainBot, bestAng, bestPos, goal);

    dist = calcDist(bestPos, goal);
    mainBot.move(dist);
    mainBot.drawBot(0);
        
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
        robot.turn(angle - currentAng);
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
            robot.move(res);
        else
            robot.move(diagDistance(res));           
        end
        
        robot.drawBot(0);
        currentPos = nextPos;      
    end
end

function d = diagDistance(res)
    d = sqrt(2*res^2);
end