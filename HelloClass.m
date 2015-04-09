classdef HelloClass < handle
properties
message=[]; 
lastKey;
isContinue=true;
motorValue=1.0;
hMotorValueSlider;
end

methods(Static)
    
function deltaT=cameraCaputreTest(width,height,fps)
if ~exist('width','var'); width=640; end;
if ~exist('height','var'); height=480; end;
if ~exist('fps','var'); fps=30; end;

deviceId=0;
videoInput=vi_create();
    function closeCamera(vi,devid)
        vi_stop_device(vi,devid);
        vi_delete(vi);
        disp('camera delete done');
    end
c=onCleanup(@() closeCamera(videoInput,deviceId));
numDevices=vi_list_devices(videoInput);
vi_setup_device(videoInput,deviceId,width,height,fps);
img=zeros(height,width,3,'uint8');
figure(1);
hImg=imshow(img);
drawnow;

maxFrame=100;
deltaT=zeros(4,maxFrame);
for I=1:maxFrame
    t1=tic;
    vi_is_frame_new(videoInput,deviceId);
    deltaT(1,I)=toc(t1)*1000;
    t2=tic;
    img=vi_get_pixels(videoInput,deviceId);
    fprintf(2,'[Captured Size]%d x %d\n',size(img,2),size(img,1));    
    deltaT(2,I)=toc(t2)*1000;
    t3=tic;
    set(hImg,'CData',img);
    deltaT(3,I)=toc(t3)*1000;
    t4=tic;
    drawnow;
    deltaT(4,I)=toc(t4)*1000;
end

end
    
function onKeyPressedStatic(src,event,data)
    disp('KeyPressed:');
    disp(event.Key);
    disp(data);
end
end

methods
function obj=HelloClass()
obj.message='HelloWorld!';    
end

function onKeyPressed(obj,src,event)
    disp('KeyPressed:');
    disp(event.Key);
    if (strcmp(event.Key,'x')) 
        fud=get(src,'UserData');
        fud.isContinue=false;
        disp('Continue stopping...');
    end
end

function onKeyReleased(obj,src,event,data)
    disp('KeyReleased:');
    disp(event.Key);
    disp(data);
end

function onMouseDown(obj,src,event)
    disp('MouseDown:');    
    disp(get(gca,'CurrentPoint'));
end

function eventHandlerTest(obj)
hFig=figure(1);
set(hFig,'KeyPressFcn',{@(src,event) HelloClass.onKeyPressedStatic(src,event,10)});
set(hFig,'KeyReleaseFcn',{@(src,event) obj.onKeyReleased(src,event,20)});
set(hFig,'WindowButtonDownFcn',{@obj.onMouseDown});
end

function loopExitTest(obj)
hFig=figure(1);
figUserData.isContinue=true;
set(hFig,'UserData',figUserData);
set(hFig,'KeyPressFcn',{@obj.onKeyPressed});
while(figUserData.isContinue)
   pause(1); 
   disp('loop');
end
disp('loop exit');
end

function print(obj)
    disp(obj.message);
end

function testLoopCheck(obj)
    function b=loopCheck()
%         v=mod(floor(cputime*1000),7);
        v=cputime;
        disp('checking...');
        disp(v);
        b=v~=1;
    end
while(loopCheck())
    pause(1);
    disp('loop...');
end
end

function testLoopCheck2(obj)
    function b=loopCheck()
        v=mod(floor(cputime*1000),3);        
        disp('checking...');
        disp(v);
        b=v~=1;
    end
isContinue=true;    
while(isContinue)
    pause(1);
    disp('loop...');
    isContinue=loopCheck();
end
end

% function b=isContinue(obj)
% b=~strcmp(obj.lastKey,'x');    
% end

function keySave(obj,src,event)
    fprintf(1,'keySave:%s\n',event.Key);
    obj.lastKey=event.Key;
end

function testLoopCheck3(obj)
b=true;  
hFig=figure(1);
set(hFig,'KeyPressFcn',{@obj.keySave});
while(b)
pause(1);
disp('loop');
fprintf(1,'lastKey:%s\n',obj.lastKey);
ud=get(hFig,'UserData');
b=obj.isContinue();
end
end

function keyCheck(obj,src,event)
    disp('keyCheck');
    obj.isContinue=false;
end

function testLoopCheck4(obj)
hFig=figure(1);
set(hFig,'KeyPressFcn',{@obj.keyCheck});
while(obj.isContinue)
    pause(1);
    disp('loop...');
end
disp('loop exit!');
end

function testButton(obj)
hFig=figure(1);
    function onOkButtonPressed(src,event)
        disp('onOkButtonPressed');
        ud=get(src,'UserData');
        ud.finished=true;
        set(src,'UserData',ud);
    end
okButton=uicontrol('Parent',hFig,'Style','pushbutton',...
    'Position',[100 100 200 150],'String','OK');    
set(okButton,'Callback',{@onOkButtonPressed});
userData.finished=false;
set(okButton,'UserData',userData);
while(true)
    disp('loop...');    
    userData=get(okButton,'UserData');
    if userData.finished
        break;
    end
    pause(1);
end
disp('loop done');
end

function onMotorValueSliderChanged(obj,src,event)
    obj.motorValue = min(get(obj.hMotorValueSlider,'Value'), 1.0);
    fprintf(2,'Slider Event:MotorValue=%d\n',obj.motorValue);
end

function testSlider1(obj)
hFig=figure(1); clf(1);    
obj.hMotorValueSlider = uicontrol('Parent',hFig,'Style','slider',...
    'Position',[10 10 100 15]) ;
set(obj.hMotorValueSlider, 'Max', 1.0);
set(obj.hMotorValueSlider, 'Min', 0.0);
set(obj.hMotorValueSlider, 'SliderStep', [0.1, 0.2]);
set(obj.hMotorValueSlider, 'Value', obj.motorValue);
set(obj.hMotorValueSlider, 'Callback', {@obj.onMotorValueSliderChanged})
end


function delete(obj)
    disp('destructor called');
end

end
    
end

