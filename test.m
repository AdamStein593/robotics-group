%test robotics
COM_CloseNXT all %Close any left over connections to the NXT
h =COM_OpenNXT(); %Connect to the NXT Brick via USB
COM_SetDefaultNXT(h); %Set our connection as the default

NXT_PlayTone(440, 500) % Play a tone of (freq, duration)

%motorA = NXTMotor('A') ;
%motorA.Power = -100; 
motorC = NXTMotor('C') ;
motorC.Power = -100; 

%motorA.TachoLimit = 720;
motorC.TachoLimit = 287;
%motorA.SendToNXT()
motorC.SendToNXT()
COM_CloseNXT(h) %Close the serial connection
