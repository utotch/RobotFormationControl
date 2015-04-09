classdef RobotPIDControl < handle
properties(Constant)
DefaultRobotID=1;
DefaultFPS=1;
MaxSensorLogSec=10; % sec
DefaultNumIntegrateFrame=8; % frames
DefaultDataFile='tmp.mat';
end

properties
pidMode;    
hOmnikit;
hFig;
hTargetPlot;
hSensorPlot;
hErrorPlot;
hPeakPlot;
hIntegratorPlot;
K_C=0.0;
T_C=NaN;
K_P=0.0;
K_I=0.0;
K_D=0.0;
hText_T_C;
hText_target;
hText_sensor;
hText_controlValue;
hText_K_P;
hText_K_I;
hText_K_D;
hSlider_target;
hSlider_sensor;
hSlider_controlValue;
hSlider_K_P;
hSlider_K_I;
hSlider_K_D;

errorValue;
integratorValue;
numIntegrateFrame;
targetValue=0.0;
sensorValue=0.0;
sensorValueLog;
sensorPeak=NaN(2,1);
offsetControlValue=0.055; % Ã–€ŽC‚ð’´‚¦‚é‚½‚ß‚É—š‚©‚¹‚éƒQƒ^•ª

robotID;
% criticalPeriodTimer=NaN;
fps;

deltaTsec;
MinTargetValue;
MaxTargetValue;
minControlValue=-1.0;
maxControlValue=1.0;
end
    
methods
    
function obj=RobotPIDControl(fps,hOmnikit,mode)
if ~exist('fps','var'); fps=RobotPIDControl.DefaultFPS; end;
if ~exist('hOmnikit','var')
    obj.hOmnikit=NaN;
else 
    obj.hOmnikit=hOmnikit;
end
if ~exist('mode','var'); mode='angle'; end;
obj.pidMode=mode;
if strcmpi(mode,'angle') 
    obj.MinTargetValue=-pi;
    obj.MaxTargetValue=pi;    
else % mode='normal'
    % ³‹K‰»À•W (-1,-1) - (1,1) ‚Ì’[‚©‚ç’[‚Ü‚Å
    obj.MinTargetValue=-2*sqrt(2); 
    obj.MaxTargetValue=+2*sqrt(2);    
end
obj.robotID=RobotPIDControl.DefaultRobotID;
obj.fps=fps;
obj.deltaTsec=1/obj.fps;
bufSize=floor(RobotPIDControl.MaxSensorLogSec/obj.deltaTsec);
obj.sensorValueLog=NaN(1,bufSize);
obj.errorValue=zeros(1,bufSize);
obj.integratorValue=zeros(1,bufSize);
obj.numIntegrateFrame=RobotPIDControl.DefaultNumIntegrateFrame;
end

function Y=update_PID(obj,sensor_val,target_val)
obj.errorValue=circshift(obj.errorValue,[0 -1]);
obj.integratorValue=circshift(obj.integratorValue,[0 -1]);
obj.errorValue(end) = target_val-sensor_val;
if strcmp(obj.pidMode,'angle')
    % nomalize to -pi < e < pi
    if obj.errorValue(end) > pi
        obj.errorValue(end)=obj.errorValue(end)-2*pi;
    elseif obj.errorValue(end) < -pi
        obj.errorValue(end)=obj.errorValue(end)+2*pi;
    end
end

% obj.integratorValue(end) = obj.integratorValue(end-1) + mean(obj.errorValue((end-1):end),2) * obj.deltaTsec;
integrates=[obj.errorValue((end-obj.numIntegrateFrame+1):end-1) mean(obj.errorValue((end-1):end),2) * obj.deltaTsec];
obj.integratorValue(end) = sum(integrates,2);
fprintf(2,'update_PID(%f,%f), integrator=%f\n',sensor_val,target_val,obj.integratorValue(end));    
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
end    

function updateCriticalPeriod(obj) % not complete yet
[Y,I]=Util.peak_detect(obj.sensorValueLog,3);  
if numel(I) > 0
    obj.sensorPeak=[I;Y];
else
    obj.sensorPeak=NaN(2,1);
end
if numel(I) >= 2
obj.T_C=(I(end)-I(end-1))*obj.deltaTsec;
set(obj.hText_T_C,'String',sprintf('T_C:%f',obj.T_C));
fprintf(2,'CriticalPeriod Detection Success:%f [sec]\n',obj.T_C);
else 
obj.T_C=NaN;    
end
end

function autoTunePID(obj,K_C,T_C) % not complete yet
% Ziegler Nichols Tuning
if isnan(obj.T_C)
    fprintf(2,'T_C is unknown. cannot auto detect\n');
    return;
end
obj.K_C=K_C;
obj.T_C=T_C;
obj.K_P=0.6*K_C;
T_I=0.5*T_C;
T_D=0.125*T_C;
obj.K_I=obj.K_P / T_I;
obj.K_D=obj.K_P * T_D;
if obj.K_P < obj.minControlValue
    obj.K_P=obj.minControlValue;
elseif obj.maxControlValue < obj.K_P
    obj.K_P=obj.maxControlValue;
end
if obj.K_I < obj.minControlValue
    obj.K_I=obj.minControlValue;
elseif obj.maxControlValue < obj.K_I
    obj.K_I=obj.maxControlValue;
end
if obj.K_D < obj.minControlValue
    obj.K_D=obj.minControlValue;
elseif obj.maxControlValue < obj.K_D
    obj.K_D=obj.maxControlValue;
end
set(obj.hSlider_K_P,'Value',obj.K_P);
set(obj.hSlider_K_I,'Value',obj.K_I);
set(obj.hSlider_K_D,'Value',obj.K_D);
obj.savePID;
end

function onPIDSliderChanged(obj,src,event)
    val = get(src,'Value');
    tag=get(src,'Tag');
    if (strcmp(tag,'K_P'))
        obj.K_P=val;
        set(obj.hText_K_P,'String',sprintf('K_P:%f',obj.K_P));
    elseif (strcmp(tag,'K_I'))
        obj.K_I=val;
        set(obj.hText_K_I,'String',sprintf('K_I:%f',obj.K_I));
    elseif (strcmp(tag,'K_D'))        
        obj.K_D=val;
        set(obj.hText_K_D,'String',sprintf('K_D:%f',obj.K_D));
    elseif (strcmp(tag,'target'))
        obj.setTargetValue(val);
    elseif (strcmp(tag,'sensor'))        
        obj.setSensorValue(val);
    elseif (strcmp(tag,'controlValue'))
        obj.maxControlValue=val;
        obj.minControlValue=-val;
        set(obj.hSlider_K_P,'Max',obj.maxControlValue, 'Min', obj.minControlValue);
        set(obj.hSlider_K_I,'Max',obj.maxControlValue, 'Min', obj.minControlValue);
        set(obj.hSlider_K_D,'Max',obj.maxControlValue, 'Min', obj.minControlValue);
        set(obj.hText_controlValue,'String',sprintf('controlValue:%f',obj.maxControlValue));
    end
    fprintf(2,'Slider Event:%s=%f\n',tag,val);
end

function setTargetValue(obj,val)
assert(obj.MinTargetValue < val &&  val < obj.MaxTargetValue);
obj.targetValue=val;        
set(obj.hText_target,'String',sprintf('target:%f',obj.targetValue));
end

function setSensorValue(obj,val)
obj.sensorValue=val;        
set(obj.hText_sensor,'String',sprintf('sensor:%f',obj.sensorValue));
obj.sensorValueLog=circshift(obj.sensorValueLog,[0 -1]);
obj.sensorValueLog(end)=obj.sensorValue;
end

function savePID(obj)
K_critical=obj.K_C;
T_critical=obj.T_C;
Pgain=obj.K_P;
Igain=obj.K_I;
Dgain=obj.K_D;
save(RobotPIDControl.DefaultDataFile,'Pgain','Igain','Dgain','K_critical','T_critical');
fprintf(2,'PID Gain:K_P=%f, K_I=%f, K_D=%f, K_critical=%f, T_critical=%f\n',...
    Pgain,Igain,Dgain,K_critical,T_critical);
end

function loadGains(obj)
load(RobotPIDControl.DefaultDataFile);
obj.K_C=K_critical;
obj.T_C=T_critical;
obj.K_P=Pgain;
obj.K_I=Igain;
obj.K_D=Dgain;
set(obj.hSlider_K_P,'Value',obj.K_P);
set(obj.hSlider_K_I,'Value',obj.K_I);
set(obj.hSlider_K_D,'Value',obj.K_D);
end

function resetGains(obj)
obj.K_P=0;
obj.K_I=0;
obj.K_D=0;
set(obj.hSlider_K_P,'Value',obj.K_P);
set(obj.hSlider_K_I,'Value',obj.K_I);
set(obj.hSlider_K_D,'Value',obj.K_D);
end

function initialize(obj)
obj.hFig = figure(3); clf(3);

siz=20;
LabelSize=[150 siz];
SliderStep=[0.001 0.001];
SliderSize=[400 siz];
ButtonSize=[100 siz];

obj.hText_T_C = uicontrol('Parent',obj.hFig,'Style','text',...
        'Position',[0 siz*11 LabelSize], 'String', 'T_C');
obj.hText_target = uicontrol('Parent',obj.hFig,'Style','text',...
        'Position',[0 siz*10 LabelSize], 'String', 'target');
obj.hSlider_target = uicontrol('Parent',obj.hFig,'Style','slider',...
    'Position',[LabelSize(1) siz*10 SliderSize]) ;
set(obj.hSlider_target,...
    'Max', obj.MaxTargetValue,...
    'Min', obj.MinTargetValue,...
    'SliderStep', SliderStep,...
    'Value', obj.targetValue,...
    'Tag', 'target',...
    'Callback', {@obj.onPIDSliderChanged});

obj.hText_sensor = uicontrol('Parent',obj.hFig,'Style','text',...
        'Position',[0 siz*8 LabelSize], 'String', 'sensor');
obj.hSlider_sensor = uicontrol('Parent',obj.hFig,'Style','slider',...
    'Position',[LabelSize(1) siz*8 SliderSize]) ;
set(obj.hSlider_sensor,...
    'Max', obj.MaxTargetValue,...
    'Min', obj.MinTargetValue,...
    'SliderStep', SliderStep,...
    'Value', obj.sensorValue,...
    'Tag', 'sensor',...
    'Callback', {@obj.onPIDSliderChanged});

obj.hText_K_P = uicontrol('Parent',obj.hFig,'Style','text',...
        'Position',[0 siz*5 LabelSize], 'String', 'K_P');
obj.hSlider_K_P = uicontrol('Parent',obj.hFig,'Style','slider',...
    'Position',[LabelSize(1) siz*5 SliderSize]) ;
set(obj.hSlider_K_P,...
    'Max', obj.maxControlValue,...
    'Min', obj.minControlValue,...
    'SliderStep', SliderStep,...
    'Value', obj.K_P,...
    'Tag', 'K_P',...
    'Callback', {@obj.onPIDSliderChanged});

obj.hText_K_I = uicontrol('Parent',obj.hFig,'Style','text',...
        'Position',[0 siz*4 LabelSize], 'String', 'K_I');
obj.hSlider_K_I = uicontrol('Parent',obj.hFig,'Style','slider',...
    'Position',[LabelSize(1) siz*4 SliderSize]) ;
set(obj.hSlider_K_I,...
    'Max', obj.maxControlValue,...
    'Min', obj.minControlValue,...
    'SliderStep', SliderStep,...
    'Value', obj.K_I,...
    'Tag', 'K_I',...
    'Callback', {@obj.onPIDSliderChanged});

obj.hText_K_D = uicontrol('Parent',obj.hFig,'Style','text',...
        'Position',[0 siz*3 LabelSize], 'String', 'K_D');
obj.hSlider_K_D = uicontrol('Parent',obj.hFig,'Style','slider',...
    'Position',[LabelSize(1) siz*3 SliderSize]) ;
set(obj.hSlider_K_D,...
    'Max', obj.maxControlValue,...
    'Min', obj.minControlValue,...
    'SliderStep', SliderStep,...
    'Value', obj.K_D,...
    'Tag', 'K_D',...
    'Callback', {@obj.onPIDSliderChanged});

obj.hText_controlValue = uicontrol('Parent',obj.hFig,'Style','text',...
        'Position',[0 siz*2 LabelSize], 'String', 'controlValue');
obj.hSlider_controlValue = uicontrol('Parent',obj.hFig,'Style','slider',...
    'Position',[LabelSize(1) siz*2 SliderSize]) ;
set(obj.hSlider_controlValue,...
    'Max', 1.0,...
    'Min', 0.0001,...
    'SliderStep', SliderStep,...
    'Value', obj.maxControlValue,...
    'Tag', 'controlValue',...
    'Callback', {@obj.onPIDSliderChanged});

hTuneButton=uicontrol('Parent',obj.hFig,'Style','pushbutton',...
    'Position',[0 siz*1 ButtonSize],'String','AutoTune');    
set(hTuneButton,'Callback',{@(src,event) obj.autoTunePID(obj.K_P,obj.T_C)});
hSaveButton=uicontrol('Parent',obj.hFig,'Style','pushbutton',...
    'Position',[ButtonSize(1) siz*1 ButtonSize],'String','Save Gains');    
set(hSaveButton,'Callback',{@(src,event) obj.savePID});
hLoadButton=uicontrol('Parent',obj.hFig,'Style','pushbutton',...
    'Position',[ButtonSize(1)*2 siz*1 ButtonSize],'String','Load Gains');    
set(hLoadButton,'Callback',{@(src,event) obj.loadGains});
hResetButton=uicontrol('Parent',obj.hFig,'Style','pushbutton',...
    'Position',[ButtonSize(1)*3 siz*1 ButtonSize],'String','Reset Gains');    
set(hResetButton,'Callback',{@(src,event) obj.resetGains});


subplot(2,1,1);
N=length(obj.sensorValueLog);
obj.hTargetPlot=plot(1:N,obj.targetValue*ones(1,N),'r-'); hold on;
obj.hSensorPlot=plot(1:N,obj.sensorValueLog,'-o');
obj.hPeakPlot=plot(obj.sensorPeak(1,:),obj.sensorPeak(2,:),'or');
obj.hErrorPlot=plot(1:N,obj.errorValue,'-.c');
obj.hIntegratorPlot=plot(1:N,obj.errorValue,'-.m');
axis([1 N obj.MinTargetValue obj.MaxTargetValue]);
legend('target','sensor','error','integrator',0);
drawnow;
end

function doSampleProcess(obj,sensor_val,optX) % optX=movoToMotorDirection
if ~exist('sensor_val','var'); sensor_val=get(obj.hSlider_sensor,'Value'); end;
obj.setSensorValue(sensor_val);
Y=obj.update_PID(obj.sensorValue,obj.targetValue);    
obj.updateCriticalPeriod();
set(obj.hTargetPlot,'YData',obj.targetValue*ones(1,length(obj.sensorValueLog)));
set(obj.hSensorPlot,'YData',obj.sensorValueLog);
set(obj.hPeakPlot,'XData',obj.sensorPeak(1,:));
set(obj.hPeakPlot,'YData',obj.sensorPeak(2,:));
set(obj.hErrorPlot,'YData',obj.errorValue);
set(obj.hIntegratorPlot,'YData',obj.integratorValue);

if isa(obj.hOmnikit,'utotch.arduino.OmnikitRemote') && obj.hOmnikit.isConnected
    if strcmp(obj.pidMode,'angle')
        fprintf(2,'[Real]omnikit.rotate(%d,%f)\n',obj.robotID,Y);    
        obj.hOmnikit.rotate(obj.robotID,Y);
    else
        V = Y * optX./norm(optX);        
        obj.hOmnikit.moveToDirection(obj.robotID,V(1),V(2));
        fprintf(2,'[Real]omnikit.moveToDirection(%d,%f,%f) [norm=%f]\n',obj.robotID, V(1), V(2), norm(V));           
    end
else
    if strcmp(obj.pidMode,'angle')
        fprintf(2,'[Dummy]omnikit.rotate(%d,%f)\n',obj.robotID,Y);                
    else
        V = Y * optX./norm(optX);        
        fprintf(2,'[Dummy]omnikit.moveToDirection(%d,%f,%f) [norm=%f]\n',obj.robotID, V(1), V(2), norm(V));
    end    
end
end

function run(obj,optX)
obj.initialize;    
while(true)
t1=tic;    
delta=toc(t1);
% obj.doSampleProcess();
obj.doSampleProcess(get(obj.hSlider_sensor,'Value'),optX);
pause_sec=max(obj.deltaTsec-delta,0);
pause(pause_sec);
drawnow;
end
end
    
end
end


