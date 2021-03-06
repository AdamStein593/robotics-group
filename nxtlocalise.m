map = [0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110]; %default map
start = [85,85];
mainBot = BotSim(map);
mainBot.drawMap()
h = init();
[bestPosX, bestPosY, bestAng] = particleFilter(mainBot, map);
disp(bestPosX)
disp(bestPosY)
disp(bestAng)
finish(h)

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

function [bestPosX, bestPosY, bestAng] = particleFilter(botSim, modifiedMap)
    SCAN_NUMBER = 4;

    limsMin = min(modifiedMap);
    limsMax = max(modifiedMap);
    dims = limsMax-limsMin;
    
    %generate some random particles inside the map
    numberOfPoints = 300;
    simulatedParticle(numberOfPoints,1) = BotSim;
    
    convergence_threshold = 0.6;
    moveDivideFactor = 4;
    maxNumOfIterations = 2;
    for i = 1:numberOfPoints
        simulatedParticle(i) = BotSim(modifiedMap); 
        simulatedParticle(i).randomPose(0); 
        simulatedParticle(i).setScanConfig(simulatedParticle(i).generateScanConfig(SCAN_NUMBER))
        simulatedParticle(i).setBotAng(rand(1)*2*pi);

    end
    
    
    n = 0;
    %botSimScanDistances = rotateSensor(SCAN_NUMBER);
    %disp(botSimScanDistances)
    converged =0; 
    while(converged == 0 && n < maxNumOfIterations) 
        n = n+1; 
        botSimScanDistances = rotateSensor(SCAN_NUMBER);
        if(numberOfPoints == 0)
            numberOfPoints = 100; 
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
        %disp(stdev)
        if(stdev(1) < convergence_threshold && stdev(2) < convergence_threshold)
            converged = 1;
        end
        %% Write code to decide how to move next
        % here they just turn in cicles as an example
        [furthestWallDistance, scanPosition] = max(botSimScanDistances);
        scanPosition = scanPosition - 1;
        angle = scanPosition*2*pi/SCAN_NUMBER - pi;
        
        distanceToMove = furthestWallDistance/moveDivideFactor;
        turn(-angle);
        move(distanceToMove);
        simulatedParticle(1).drawBot(0)
        for i=1:numberOfPoints
            simulatedParticle(i).turn(angle + pi);
            simulatedParticle(i).move(distanceToMove);
        end
       simulatedParticle(1).drawBot(0)
        
    end

    [bestParticleWeight, bestParticleIndex] = max(weight);
    bestPos = simulatedParticle(bestParticleIndex).getBotPos();
    bestAng = mod(simulatedParticle(bestParticleIndex).getBotAng(), 2*pi);
    bestPosX = bestPos(1);
    bestPosY = bestPos(2);
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
    rotation = round(ang/(2*pi)*900);
    
    motor = NXTMotor('A') ;
    motor2 = NXTMotor('C') ;
    %disp(ang)
    %disp(rotation)
    if (rotation > 0)
        motor.Power = -50; 
        motor2.Power = 50; 
    else
        motor.Power = 50; 
        motor2.Power = -50;
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

function h = init()
    COM_CloseNXT all %Close any left over connections to the NXT
    h =COM_OpenNXT(); %Connect to the NXT Brick via USB
    COM_SetDefaultNXT(h); %Set our connection as the default
    OpenUltrasonic(SENSOR_4)
end

function readings = rotateSensor(scanNumber)
    d = GetUltrasonic(SENSOR_4);
    d = GetUltrasonic(SENSOR_4);
    readings(1) = d;
    motor = NXTMotor('B') ;
    motor.Power = 100;
    rotation = round(360/ scanNumber);
    totalRotation = rotation * (scanNumber -1);
    for i=1:scanNumber -1
        motor.TachoLimit = rotation;
        motor.SendToNXT()
        motor.WaitFor()
        d = GetUltrasonic(SENSOR_4);
        readings(i + 1) = d;

    end
    motor.Power = -100;
    motor.TachoLimit = totalRotation;
    motor.SendToNXT()
    motor.WaitFor()
    readings = readings';
    
end

function finish(h)
    COM_CloseNXT(h)
end