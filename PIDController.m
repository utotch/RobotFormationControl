classdef PIDController < handle
    
properties(Constant)
BufferSec=10; % sec    
DefaultNumIntegrateFrame=8; % frames
DefaultFinishFrames=10; % frames
EpsilonAngrad=2*pi*10/360;
EpsilonDistance=0.1;
end

properties
angleMode;  
K_P;
K_I;
K_D;
fps;
deltaTsec;
minTargetValue;
maxTargetValue;
minControlValue=-1.0;
maxControlValue=1.0;
errorValue;
integratorValue;
numIntegrateFrame;
targetValue=0.0; % normal §Œä‚Å‚Í‚±‚Ì‚Ü‚Ü‚Å‚æ‚¢
sensorValue;
offsetControlValue; % Ã–€ŽC‚ð’´‚¦‚é‚½‚ß‚É—š‚©‚¹‚éƒQƒ^•ª
finishCounter=0;
end
    
methods

function obj=PIDController(Gain,fps,targetRange,controllRange,angleMode)
assert(numel(Gain)==4);
assert(numel(targetRange)==2);
assert(numel(controllRange)==2);
if ~exist('angleMode','var'); angleMode=false; end;
obj.K_P=Gain(1); obj.K_I=Gain(2); obj.K_D=Gain(3);
obj.offsetControlValue=Gain(4);
obj.fps=fps;
obj.minTargetValue=targetRange(1); obj.maxTargetValue=targetRange(2);
obj.minControlValue=controllRange(1); obj.maxControlValue=controllRange(2);
obj.angleMode=angleMode;
obj.deltaTsec=1/obj.fps;
bufSize=floor(PIDController.BufferSec/obj.deltaTsec);
obj.sensorValue=NaN(1,bufSize);
obj.errorValue=zeros(1,bufSize);
obj.integratorValue=zeros(1,bufSize);
obj.numIntegrateFrame=PIDController.DefaultNumIntegrateFrame;
end

function Y=update_PID(obj,sensor_val,target_val)
obj.errorValue=circshift(obj.errorValue,[0 -1]);
obj.integratorValue=circshift(obj.integratorValue,[0 -1]);
obj.errorValue(end) = target_val-sensor_val;
if obj.angleMode
    % nomalize to -pi < e < pi
    if obj.errorValue(end) > pi
        obj.errorValue(end)=obj.errorValue(end)-2*pi;
    elseif obj.errorValue(end) < -pi
        obj.errorValue(end)=obj.errorValue(end)+2*pi;
    end
end
integrates=[obj.errorValue((end-obj.numIntegrateFrame+1):end-1) mean(obj.errorValue((end-1):end),2) * obj.deltaTsec];
obj.integratorValue(end) = sum(integrates,2);
Pterm=obj.K_P * obj.errorValue(end);
Iterm=obj.K_I * obj.integratorValue(end);
Dterm=obj.K_D * (obj.errorValue(end)-obj.errorValue(end-1)) / obj.deltaTsec;
Y=Pterm+Iterm+Dterm;
Y=Y+sign(Y)*obj.offsetControlValue;
if Y < obj.minControlValue
    Y=obj.minControlValue;
elseif obj.maxControlValue < Y 
    Y=obj.maxControlValue;
end

if obj.angleMode
    epsilon=PIDController.EpsilonAngrad;
else
    epsilon=PIDController.EpsilonDistance;
end
if abs(obj.errorValue(end) < epsilon)
    obj.finishCounter=obj.finishCounter+1;
end
end 

function setTargetValue(obj,val)
assert(isnan(val) || (obj.minTargetValue <= val &&  val <= obj.maxTargetValue));
obj.targetValue=val;        
end

function b=isFinished(obj)
b = obj.finishCounter > PIDController.DefaultFinishFrames;
end

function b=isEnable(obj)
   b=~isnan(obj.targetValue);
end

function doFinish(obj)
    obj.finishCounter=0;
end

function setSensorValue(obj,val)
obj.sensorValue=circshift(obj.sensorValue,[0 -1]);
obj.sensorValue(end)=val;
end

function val=getSensorValue(obj)
val=obj.sensorValue(end);    
end

function V=doSampleProcess(obj,sensor_val,optX) % optX=movoToMotorDirection
obj.setSensorValue(sensor_val);
Y=obj.update_PID(obj.getSensorValue,obj.targetValue);    
if ~exist('optX','var')
    V = Y;
    assert(numel(V)==1);
else
    V = Y * optX./norm(optX);            
    assert(numel(V)==2);
end
end    

end
end

