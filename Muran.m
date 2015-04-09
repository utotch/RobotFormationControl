classdef Muran < handle
% 起動方法
% Muran.main(960,720,30,'RGB',0,[0.03 0.15])
% Muran.main(960,720,30,'manual',0,[0.03 0.15])
% Muran.main(960,720,30,'skip',0,[0.03 0.15])
properties(Constant)
DefaultDataDir = 'C:/home/workspace/OrumihMatlabProject/data/images/';
SampleImg = 'marker_sample4.jpg';
DefaultDataFile='Muran.mat';
DefaultBinarizeThreshold = 50; % 100-128くらい？
% DefaultCleaningElement = strel('square',3);
DefaultCleaningElementFactor=400;
% MinContourLength=10000; MaxContourLength=20000; % desktop Large Marker
MinContourLength=1000; MaxContourLength=1400; % 960x720 no pyramid RealGround (tight bound)

MarkerRangeInScreen=[0.1 0.5];
DefaultNumRobots=9;

DefaultWidth=640; DefaultHeight=480; DefaultFPS=3;
% DefaultWidth=640; DefaultHeight=480; DefaultFPS=30;
% DefaultWidth=960; DefaultHeight=720; DefaultFPS=5;
% DefaultWidth=1280; DefaultHeight=960; DefaultFPS=5;
Debug=false;
DetectCustomContourDetector=false;
LostRobotStopFrames=50;
ArrowKeyDef=[1 0 -1 0; 0 1 0 -1]; % rightarrow=[1;0], uparrow=[0;1]; leftarrow=[-1;0]; downarrow=[0;-1]
end

properties
cameraManager;
camera;
hFigCalibration;
hCalibrationImg;
hAxisCalibration;
hCalibrationRect;
calibPoints=NaN(2,4);
H;
invH;
calibrationMode;
calibrationDone=false;
minContourLength;
maxContourLength;
contourDetectPyramidLevel;
numRobots;
rectReal;
rectNormal;
robotStateNormal; % [X;Y;theta_rad]
robotPosReal;
robotMissed;
robotDetected;
robotSelect;
robotTargetPointNormal;
robotTargetPointReal;
% robotTargetAngradNormal; % robotFeedbackController で実装
hImage;
hContours;
hCorners;
hNormal;
hRobots;
hRobotsText;
hMessage;
hTemplate;
hQuiver;
hAxisNormal;
hAxisReal;
hMousePointerReal;
hMousePointerNormal;
hRobotTargetPointReal;
hRobotTargetPointNormal;
hRobotTargetTextReal;
hRobotTargetTextNormal;
hRobotTargetLineReal;
hRobotTargetLineNormal;
hFig1;
hFig2;
mouseButtonState=zeros(1,4); % 1=normal=left, 2=extend=middle, 3=alt=right, 4=open=doubleclick
keyState=zeros(1,256);
arrowKeyState=zeros(4,1); % right, up, left, down
hOmnikit;
hConnectButton;
hRobotProxyButton;
hMotorValueSlider;
motorValue=1.0;

hRobotPIDControl; % for Debug & Tuning
hRobotFeedbackController;
hFormationManager;
hStrokeManager;
end

methods
function delete(obj)
    obj.camera.delete;
    obj.cameraManager.delete;
    disp('Camera Safely Deleted.');
    clear obj.hOmniKit;
%     disp('Java Object Safely Deleted.');    
end
function obj=Muran(width,height,fps,calibMode,level,range) 
iptsetpref('ImshowBorder','tight');    
obj.calibrationMode=calibMode;
obj.contourDetectPyramidLevel=level;
obj.cameraManager=CameraManager;
obj.camera=obj.cameraManager.setupCameraDevice(0,width,height,fps);
obj.numRobots=Muran.DefaultNumRobots;
obj.rectReal=zeros(2,4,obj.numRobots);
obj.rectNormal=zeros(2,4,obj.numRobots);
obj.robotStateNormal=NaN(3,obj.numRobots);
obj.robotPosReal=NaN(2,obj.numRobots);
obj.robotTargetPointNormal=NaN(2,obj.numRobots);
obj.robotTargetPointReal=NaN(2,obj.numRobots);
% obj.robotTargetAngradNormal=NaN(1,obj.numRobots);
obj.robotMissed=zeros(obj.numRobots,1);
obj.robotDetected=zeros(obj.numRobots,1);
obj.robotSelect=zeros(obj.numRobots,1);
obj.hRobotTargetTextReal=zeros(obj.numRobots,1);
obj.hRobotTargetTextNormal=zeros(obj.numRobots,1);
obj.minContourLength=floor(2*(obj.camera.width+obj.camera.height)*range(1));
obj.maxContourLength=floor(2*(obj.camera.width+obj.camera.height)*range(2));
% obj.minContourLength=floor(2*(obj.camera.width+obj.camera.height)*Muran.MarkerRangeInScreen(1));
% obj.maxContourLength=floor(2*(obj.camera.width+obj.camera.height)*Muran.MarkerRangeInScreen(2));
% obj.minContourLength=range(1);
% obj.maxContourLength=range(2);
obj.hOmnikit=utotch.arduino.OmnikitRemote();
obj.hRobotFeedbackController=RobotController(obj,obj.hOmnikit,obj.numRobots,fps);
obj.hFormationManager=FormationManager(obj);
obj.hStrokeManager=StrokeManager(obj);
end

function onCalibrationDoneButtonPressed(obj,~,~)
    fprintf(1,'Calibration Finished\n');
    obj.calibrationDone=true;
end

function onCalibrationMouseDown(obj,~,~)
    cp=get(obj.hAxisCalibration,'CurrentPoint');
    seltype=get(obj.hFigCalibration,'SelectionType');
    fprintf(2,'[MouseDown](%f,%f)\n',cp(1,1),cp(1,2));
    if strcmp(seltype,'normal') 
        X=[cp(1,1); cp(1,2)];
        for I=1:4
            if isnan(obj.calibPoints(1,I))
                obj.calibPoints(:,I)=X;
                break;
            end
        end
    else
        obj.calibPoints=NaN(2,4);
    end
end

function [HomographyMatrix,invHomographyMatrix]=runCalibration(obj)
obj.hFigCalibration=figure(1); clf(1);
set(obj.hFigCalibration, 'WindowButtonDownFcn', {@obj.onCalibrationMouseDown});
okButton=uicontrol('Parent',obj.hFigCalibration,'Style','pushbutton',...
    'Position',[5 5 30 20],'String','OK');    
set(okButton,'Callback',{@obj.onCalibrationDoneButtonPressed});
img=obj.camera.createBuffer;
binImg=zeros(obj.camera.height,obj.camera.width);
subplot('Position',[0.0 0.5  1.0 0.5]);
obj.hCalibrationImg=imshow(img); obj.hAxisCalibration = gca;
hold on; obj.hCalibrationRect=plot(0,0,'or');
subplot('Position',[0.0 0.25 0.5 0.25]); hRed=imshow(binImg); 
subplot('Position',[0.0 0.0  0.5 0.25]); hGreen=imshow(binImg);
subplot('Position',[0.5 0.0  0.5 0.25]); hBlue=imshow(binImg);
subplot('Position',[0.5 0.25 0.5 0.25]); hCyan=imshow(binImg);
drawnow;
% obj.camera.setBrightness(0.8);
calibRGB=strcmpi(obj.calibrationMode,'RGB');
manual=strcmpi(obj.calibrationMode,'manual');
while(~obj.calibrationDone)
img=obj.camera.captureFrame;
set(obj.hCalibrationImg,'CData',img);
if calibRGB
    I=Img.get_marker_red_bin(img); set(hRed,'CData',I);
    I=Img.get_marker_green_bin(img); set(hGreen,'CData',I);
    I=Img.get_marker_blue_bin(img); set(hBlue,'CData',I);
    I=Img.get_marker_cyan_bin(img); set(hCyan,'CData',I);
else
    HSV=rgb2hsv(img); Hue=HSV(:,:,1); S=HSV(:,:,2); V=HSV(:,:,3);
    I=Img.get_marker_red_bin_hsv(Hue,S,V); set(hRed,'CData',I);
    I=Img.get_marker_green_bin_hsv(Hue,S,V); set(hGreen,'CData',I);
    I=Img.get_marker_blue_bin_hsv(Hue,S,V); set(hBlue,'CData',I);
    I=Img.get_marker_cyan_bin_hsv(Hue,S,V); set(hCyan,'CData',I);
end

try
    if calibRGB
        obj.calibPoints=Marker.calc_calibration_points(img);
    elseif ~manual
        obj.calibPoints=Marker.calc_calibration_points_hsv(Hue,S,V);
    end
    set(obj.hCalibrationRect,'XData',obj.calibPoints(1,:));
    set(obj.hCalibrationRect,'YData',obj.calibPoints(2,:));
catch ex
    disp(ex.message);
end
drawnow;
end
[HomographyMatrix,invHomographyMatrix]=Marker.calc_homography_matrix(obj.calibPoints);
save(Muran.DefaultDataFile,'HomographyMatrix');
fprintf(2,'HomographyMatrix saved to %s\n',Muran.DefaultDataFile);
end

function moveRobots(obj)
    for I=1:obj.numRobots
        if obj.robotMissed(I) > Muran.LostRobotStopFrames
            fprintf(2,'[LOST ROBOT]%d . STOP ROBOT\n',I);
            obj.hOmnikit.motorStop(I);
            obj.hOmnikit.flush(I);
            obj.robotMissed(I)=0;
            obj.rectReal(:,:,I)=NaN(size(obj.rectReal(:,:,I)));
            obj.rectNormal(:,:,I)=NaN(size(obj.rectNormal(:,:,I)));
            obj.robotStateNormal(:,I)=NaN(3,1);
            obj.robotPosReal(:,I)=NaN(2,1);
            set(obj.hRobots(I),'Visible','off');
            set(obj.hRobotsText(I),'Visible','off');
        end
        if ~isnan(obj.robotTargetPointNormal(1,I)) % destination exist
        end

    end
end

function updateRobotPosition(obj)
colorImg = obj.camera.captureFrame;
binImg=obj.getBinaryImage(colorImg);
numDetected=obj.detectMarkers(binImg);
set(obj.hImage,'CData',binImg);
set(obj.hMessage,'String',sprintf('[%dx%d]%d marker detected',obj.camera.width,obj.camera.height,numDetected));
qX=zeros(4,obj.numRobots);
for I=1:obj.numRobots
    if obj.robotDetected(I)
        set(obj.hRobots(I),'XData',obj.rectNormal(1,:,I),'YData',obj.rectNormal(2,:,I),'Color','r','Visible','on');
        set(obj.hRobotsText(I),'Position',obj.robotStateNormal([1 2],I)','Visible','on');
        qX([1 2],I)=obj.robotStateNormal([1 2],I);
        qX([3 4],I)=(obj.rectNormal(:,1,I)-obj.robotStateNormal([1 2],I)+obj.rectNormal(:,4,I)-obj.robotStateNormal([1 2],I))./2;    
        set(obj.hRobotTargetLineReal(I),'XData',[obj.robotPosReal(1,I) obj.robotTargetPointReal(1,I)],...
                     'YData',[obj.robotPosReal(2,I) obj.robotTargetPointReal(2,I)]);
        set(obj.hRobotTargetLineNormal(I),'XData',[obj.robotStateNormal(1,I) obj.robotTargetPointNormal(1,I)],...
                     'YData',[obj.robotStateNormal(2,I) obj.robotTargetPointNormal(2,I)]);
        obj.robotMissed(I)=0;         
    else 
        set(obj.hRobots(I),'Color','b');   
        if obj.hOmnikit.isConnected
            obj.robotMissed(I)=obj.robotMissed(I)+1;
        end
    end
end
set(obj.hQuiver,'XData',qX(1,:));
set(obj.hQuiver,'YData',qX(2,:));
set(obj.hQuiver,'UData',qX(3,:));
set(obj.hQuiver,'VData',qX(4,:));

end

function numDetected=detectMarkers(obj,binImg)
numDetected=0;
binImgRough=binImg;
for I=1:obj.contourDetectPyramidLevel
    binImgRough=impyramid(binImgRough,'reduce');
end
calcScale=size(binImg,1)/size(binImgRough,1);
if Muran.DetectCustomContourDetector
    corners=obj.detectMarkerCandidateSlow(binImgRough,calcScale);    
else
    corners=obj.detectMarkerCandidateFast(binImgRough,calcScale);
end
if numel(corners)==0 
    return;
end
cs_points=flipud(cell2mat(corners)');
set(obj.hCorners,'XData',cs_points(1,:).*calcScale);
set(obj.hCorners,'YData',cs_points(2,:).*calcScale);

numCandidates=length(corners);
obj.robotDetected=zeros(obj.numRobots,1);
for I=1:numCandidates
    realCorners=circshift(rot90(corners{I}',2),[0 1]).* calcScale;
    RN3=obj.H*[realCorners;ones(1,4)];
    RN2=[RN3(1,:)./RN3(3,:); RN3(2,:)./RN3(3,:)];
    templatePoints=Marker.template_points(realCorners);
    if Img.bounds_contain(size(binImg),templatePoints,2) % 近接領域分を引く
        template=Marker.extract_template_near(binImg,templatePoints);
        if Muran.Debug
            set(obj.hTemplate(I),'CData',reshape(~template,4,4));
        end
        [id,rot]=Marker.decode_template1_with_rotation(template);
%     RectReal(:,:,I)=circshift(R,[0 rot]);
        if id > 0 
            obj.rectNormal(:,:,id)=circshift(RN2,[0 rot]);
            obj.robotStateNormal([1 2],id)=mean(obj.rectNormal(:,:,id),2);
            % robot up is up then angrad =0; robot up is right then angrad
            % = -pi/2
            obj.robotStateNormal(3,id)=mean([atan2(obj.rectNormal(1,2,id)-obj.rectNormal(1,1,id),obj.rectNormal(2,1,id)-obj.rectNormal(2,2,id)) ...
                atan2(obj.rectNormal(1,3,id)-obj.rectNormal(1,4,id),obj.rectNormal(2,4,id)-obj.rectNormal(2,3,id))]); % needs debug
            X=obj.invH*[obj.robotStateNormal([1 2],id); 1];
            obj.robotPosReal(:,id)=[X(1)/X(3); X(2)/X(3)];
            obj.robotDetected(id)=1;
        end
    end
end
numDetected=sum(obj.robotDetected);
end

function corners=detectMarkerCandidateSlow(obj,binImg,calcScale)
[labelImg,numLabels] = bwlabel(Img.set_edge(binImg,0),8);
contours=Img.contours_in_length(labelImg,numLabels,floor(obj.minContourLength/calcScale),floor(obj.maxContourLength/calcScale));
corners=cell(size(contours));
if numel(corners) == 0; return; end;
cs_points=flipud(cell2mat(contours)'); % 
set(obj.hContours,'XData',cs_points(1,:).*calcScale); %
set(obj.hContours,'YData',cs_points(2,:).*calcScale); %

for I=1:length(corners)
    RowCol=contours{I};
    % k を N/8 にすることで、4点に絞り込むことが容易になる
    corners{I}=Img.corner_detect(RowCol,size(RowCol,1)/4,size(RowCol,1)/8,0.2); 
end
corners=Util.filter_cell(corners,@(x) length(x)==4);
corners=Img.filter_outers(corners);
end

function corners=detectMarkerCandidateFast(obj,binImg,calcScale)
[contours,~]=bwboundaries(Img.set_edge(binImg,1),8,'noholes');
if numel(contours) == 0; return; end;
% cs_points=flipud(cell2mat(contours)'); % 
cs_points=zeros(2,1);
corners=cell(size(contours));
for I=1:length(corners)
    RowCol=contours{I};
    len=size(RowCol,1);
    if floor(obj.minContourLength/calcScale) < len && len < floor(obj.maxContourLength/calcScale)    
        % k を N/8 にすることで、4点に絞り込むことが容易になる
        corners{I}=Img.corner_detect(RowCol,size(RowCol,1)/4,size(RowCol,1)/8,0.2); 
        cs_points=[cs_points flipud(RowCol')];
    end
end
set(obj.hContours,'XData',cs_points(1,:).*calcScale); %
set(obj.hContours,'YData',cs_points(2,:).*calcScale); %
corners=Util.filter_cell(corners,@(x) length(x)==4);
corners=Img.filter_outers(corners);
end

function binImg=getBinaryImage(obj,colorImg)
grayImg = rgb2gray(colorImg);
binRawImg = grayImg < Muran.DefaultBinarizeThreshold;
binImg = imopen(binRawImg,strel('square',floor(length(binRawImg)/Muran.DefaultCleaningElementFactor)));
end

% function onRobotSelectButtonPressed(obj,src,~)
%     
% end
function setRobotSelect(obj,id,b)
    obj.robotSelect(id)=b;
    set(obj.hRobotProxyButton(id),'Value',b');
    fprintf(2,'[RobotSelect]');
    disp(obj.robotSelect');
end

function doRobotRotation(obj,dir,onoff)
    if ~obj.hOmnikit.isConnected; return; end;
    fprintf(2,'[robot rotation]%d %d\n',dir,onoff);
    for I=1:obj.numRobots
        if obj.robotSelect(I)
            if onoff
                obj.hOmnikit.rotate(I,dir*obj.motorValue);
                obj.hOmnikit.flush(I);
            else
%                 obj.hOmnikit.motorStop(I);
                obj.hOmnikit.motorBreak(I,obj.motorValue);
                obj.hOmnikit.flush(I);
            end
        end
    end
end

function doRobotStop(obj,robots)
    if ~obj.hOmnikit.isConnected; return; end;    
    for id=1:obj.numRobots
        if robots(id)    
            obj.hRobotFeedbackController.angleFeedbackController(id).setTargetValue(NaN);
            fprintf(2,'robot stop:%d\n',id);
            obj.hOmnikit.motorStop(id);
            obj.hOmnikit.flush(id);
         end
    end
end

function doControlRobotDirection(obj,robots,angrad)
    if obj.hOmnikit.isConnected
        for id=1:obj.numRobots
            if robots(id)
%                 obj.robotTargetAngradNormal=angrad;
                obj.hRobotFeedbackController.angleFeedbackController(id).setTargetValue(angrad);
            end
        end
    end
end

function onRealImageActionPerformed(obj,x,y,onoff,buttonid)
    if (buttonid==4) % double click
        return;
    end    
    fprintf(2,'RealImageAction:(%f,%f),%d)\n',x,y,onoff);
end

function resetRobotTarget(obj,id)
    fprintf(2,'resetRobotTarget(%d)\n',id);
    obj.robotTargetPointNormal(:,id)=[NaN;NaN];
    obj.robotTargetPointReal(:,id)=[NaN;NaN];
    set(obj.hRobotTargetPointNormal,'XData',obj.robotTargetPointNormal(1,:),...
        'YData',obj.robotTargetPointNormal(2,:));
    set(obj.hRobotTargetPointReal,'XData',obj.robotTargetPointReal(1,:),...
        'YData',obj.robotTargetPointReal(1,:));
    set(obj.hRobotTargetTextReal(id),'Position',[NaN NaN]);
    set(obj.hRobotTargetTextNormal(id),'Position',[NaN NaN]);
    set(obj.hRobotTargetLineReal(id),'XData',[NaN NaN],'YData',[NaN NaN]);
    set(obj.hRobotTargetLineNormal(id),'XData',[NaN NaN],'YData',[NaN NaN]);
end

function setTargetPointNormal(obj,id,NX)
assert(size(NX,1)==2 && size(NX,2)==1);
obj.robotTargetPointNormal(:,id)=NX;
X=obj.invH*[NX;1];
obj.robotTargetPointReal(:,id)=[X(1)/X(3); X(2)/X(3)];
set(obj.hRobotTargetPointNormal,'XData',obj.robotTargetPointNormal(1,:),...
     'YData',obj.robotTargetPointNormal(2,:));
set(obj.hRobotTargetPointReal,'XData',obj.robotTargetPointReal(1,:),...
     'YData',obj.robotTargetPointReal(2,:));
set(obj.hRobotTargetTextNormal(id),'Position',obj.robotTargetPointNormal(:,id));                
set(obj.hRobotTargetTextReal(id),'Position',obj.robotTargetPointReal(:,id));
end

function onNormalCoordinateActionPerformed(obj,x,y,onoff,buttonid)
    colors={'r','g','b','c'};
    fprintf(2,'RealImageAction:(%f,%f),%d)\n',x,y,onoff);
    
    if (buttonid==4) % double click
        for id=1:obj.numRobots
            obj.resetRobotTarget(id);
        end
        obj.doRobotStop(ones(obj.numRobots,1));
        return;
    elseif (buttonid==2) % middle button
        for k=int32('1'):int32('9')
            if obj.keyState(k)
                fprintf(2,'set destination point:robot %d',k);
                id=k-int32('0');
                obj.setTargetPointNormal(id,[x;y]);
                break;
            end
        end
        return
    end
    
    if onoff 
        X=obj.invH*[x;y;1];
        RX=[X(1)/X(3);X(2)/X(3)];
        set(obj.hMousePointerReal,'Visible','on','XData',RX(1),'YData',RX(2),'Color',colors{buttonid});
        set(obj.hMousePointerNormal,'Visible','on','XData',x,'YData',y,'Color',colors{buttonid});
        if buttonid==1
%             obj.manualControlRobots([x;y],RX); % mousePointNormal, mousePointReal
             obj.manualControlRobots([x;y]); % mousePointNormal
        elseif buttonid==3
            % do something
        end
    else
        set(obj.hMousePointerReal,'Visible','off');
        set(obj.hMousePointerNormal,'Visible','off');
        if buttonid==1
            obj.doRobotStop(obj.robotSelect);
        elseif buttonid==3
            % do something
        end
    end
end

function [Y,theta]=normalVectorToMotorControlVector(obj,id,X)
    theta=obj.robotStateNormal(3,id);
    R=[cos(-theta) -sin(-theta); sin(-theta) cos(-theta)];
    Y=R*X;
end

function manualControlRobots(obj,NX,absoluteMode) % mousePointNormal
if ~exist('absoluteMode','var'); absoluteMode=false; end;
if ~obj.hOmnikit.isConnected; return; end;
    for I=1:obj.numRobots
        if obj.robotDetected(I) && obj.robotSelect(I)
            if (absoluteMode)
                DX=NX;
            else
                DX=NX-obj.robotStateNormal([1 2],I);
                if Muran.Debug
                    fprintf(2,'[ManualControl(relative)](%f,%f)-(%f,%f)=(%f,%f)\n', ...
                        NX(1),NX(2),obj.robotStateNormal(1,I),obj.robotStateNormal(2,I),DX(1),DX(2));
                end
            end
             [Y,theta]=obj.normalVectorToMotorControlVector(I,DX);
            alpha=atan2(DX(2),DX(1));
            fprintf(2,'[ManualControl]%d:[INPUT](%f,%f),angle=%f [ControlVector](%f,%f),angle=%f\n', ...
                I,DX(1),DX(2),360*alpha/(2*pi), Y(1),Y(2),360*theta/(2*pi));
            if norm(Y) > 0.05
                obj.hOmnikit.moveToDirection(I,Y(1),Y(2));
            else 
                obj.hOmnikit.motorStop(I);
            end
            obj.hOmnikit.flush(I);    
        end
    end
end

function onMouseDown(obj,src,event)
    fprintf(2,'[MouseDown]\n');    
    seltype=get(obj.hFig1,'SelectionType');
    if strcmp(seltype,'normal') 
        buttonid=1;
    elseif strcmp(seltype,'extend')
        buttonid=2;
    elseif strcmp(seltype,'alt')
        buttonid=3;
    elseif strcmp(seltype,'open')
        buttonid=4;
    end
    obj.mouseButtonState(buttonid)=true;
    cp=get(obj.hAxisNormal,'CurrentPoint');
    x=cp(1,1); y=cp(1,2);
    if (-1 <= x && -1 <= y && x <= 1 && y < 1)
        obj.onNormalCoordinateActionPerformed(x,y,true,buttonid);
        return;
    end
    cp=get(obj.hAxisReal,'CurrentPoint');
    x=cp(1,1); y=cp(1,2);
    if (x > 0 && y > 0 && x < obj.camera.width && y < obj.camera.height)
        obj.onRealImageActionPerformed(x,y,true,buttonid)
        return;
    end
end

function onMouseMove(obj,src,event)
    cp=get(obj.hAxisNormal,'CurrentPoint');
    x=cp(1,1); y=cp(1,2);
    if (-1 <= x && -1 <= y && x <= 1 && y < 1)
        for I=1:length(obj.mouseButtonState)
           if obj.mouseButtonState(I)
               obj.onNormalCoordinateActionPerformed(x,y,true,I);
           end
        end
        return;
    end
    cp=get(obj.hAxisReal,'CurrentPoint');
    x=cp(1,1); y=cp(1,2);
    if (x > 0 && y > 0 && x < obj.camera.width && y < obj.camera.height)
        for I=1:length(obj.mouseButtonState)
           if obj.mouseButtonState(I)
               obj.onRealImageActionPerformed(x,y,true,I);
           end
        end        
        return;
    end
end

function onMouseUp(obj,src,event)
    fprintf(2,'[MouseUp]\n');
    seltype=get(obj.hFig1,'SelectionType');
    if strcmp(seltype,'normal') 
        buttonid=1;
    elseif strcmp(seltype,'extend')
        buttonid=2;
    elseif strcmp(seltype,'alt')
        buttonid=3;
    elseif strcmp(seltype,'open')
        buttonid=4;
    end
    obj.mouseButtonState(buttonid)=false;    
    cp=get(obj.hAxisNormal,'CurrentPoint');
    x=cp(1,1); y=cp(1,2);
    if (-1 <= x && -1 <= y && x <= 1 && y < 1)
        obj.onNormalCoordinateActionPerformed(x,y,false,buttonid);
        return;
    end
    cp=get(obj.hAxisReal,'CurrentPoint');
    x=cp(1,1); y=cp(1,2);
    if (x > 0 && y > 0 && x < obj.camera.width && y < obj.camera.height)
        obj.onRealImageActionPerformed(x,y,false,buttonid)
        return;
    end
end

function setAllRobotSelect(obj,b)
    for id=1:obj.numRobots
        obj.setRobotSelect(id,b);
    end
end

function onKeyPressed(obj,src,event)
     if Muran.Debug 
         fprintf(2,'[KeyPressed]Key=%s, Character=%s\n',event.Key,event.Character);
     end
    if length(event.Key)==1
        keyid=int32(event.Key);
        obj.keyState(keyid)=true;
        numid=keyid-int32('0');
        if 0 < numid && numid <= 9
            set(obj.hRobotProxyButton(numid),'BackgroundColor','r');
            obj.setRobotSelect(numid,true);
        end
        switch event.Key
            case 'a'
                obj.hFormationManager.doFormationAll(FormationManager.Default9);
                fprintf(2,'doFormationAll:Default9\n');                
            case 'b'
                obj.hFormationManager.doFormationAll(FormationManager.Side45);                
                fprintf(2,'doFormationAll:Side45\n');
            case 'c'
                obj.hFormationManager.doFormationAll(FormationManager.Side54);                                
                fprintf(2,'doFormationAll:Side54\n');
            case 'd'
                obj.hFormationManager.doFormationAll(FormationManager.Dance);                                
                fprintf(2,'doFormationAll:Dance\n');
            case 's'
                obj.hFormationManager.doFormationAll(FormationManager.CharS);                                
                fprintf(2,'doFormationAll:CharS\n');
            case 't'
                obj.hFormationManager.doFormationAll(FormationManager.Tetris);                                
                fprintf(2,'doFormationAll:Tetris\n');
            case 'x'
                obj.hFormationManager.doFormationAll(FormationManager.Cross);                                
                fprintf(2,'doFormationAll:Cross\n');
            case 'v'
                obj.hFormationManager.doFormationAll(FormationManager.Triangle);                                
                fprintf(2,'doFormationAll:Trianglen');                
            case 'i'
                obj.hStrokeManager.updateTarget;
                fprintf(2,'iPad Stroke Update\n');
            case 'q'
                obj.doControlRobotDirection([1 1 1 0 0 0 0 0 0],0);
                fprintf(2,'RotateUP 1,2,3\n');
            case 'w'
                obj.doControlRobotDirection([0 0 0 1 1 1 0 0 0],0);
                fprintf(2,'RotateUP 4,5,6\n');                
            case 'e'
                obj.doControlRobotDirection([0 0 0 0 0 0 1 1 1],0);
                fprintf(2,'RotateUP 7,8,9\n');        
            case 'r'
                obj.doRobotRotation(1,true); 
                fprintf(2,'Rotate Right\n');        
            case 'f'
                obj.doRobotRotation(-1,true); 
                fprintf(2,'Rotate Left\n');        
            case 'z'
                obj.doRobotStop(ones(obj.numRobots,1));
                fprintf(2,'robotAllStop\n');   
            otherwise
        end
    else
        if strcmp(event.Key,'rightarrow')
            if ~obj.arrowKeyState(1)
                obj.arrowKeyState(1)=1;
                obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);
            end
        elseif strcmp(event.Key,'uparrow')
            if ~obj.arrowKeyState(2)
                obj.arrowKeyState(2)=1;
                obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);            
            end
        elseif strcmp(event.Key,'leftarrow')
            if ~obj.arrowKeyState(3)
                obj.arrowKeyState(3)=1;
                obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);
            end
        elseif strcmp(event.Key,'downarrow')
            if ~obj.arrowKeyState(4)
                obj.arrowKeyState(4)=1;
                obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);            
            end
        end
    end
end

function onKeyReleased(obj,src,event)
%     fprintf(2,'[KeyReleased]%s\n',event.Key);
    if length(event.Key)==1
        keyid=int32(event.Key);
        obj.keyState(keyid)=false;
        numid=keyid-int32('0');
        if 0 < numid && numid <= 9
            set(obj.hRobotProxyButton(numid),'BackgroundColor','w');
%             obj.setRobotSelect(numid,get(obj.hRobotProxyButton(numid),'Value'));
             obj.setRobotSelect(numid,false);
        end
    else
        if strcmp(event.Key,'rightarrow')
            obj.arrowKeyState(1)=0;
            obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);            
        elseif strcmp(event.Key,'uparrow')
            obj.arrowKeyState(2)=0;
            obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);            
        elseif strcmp(event.Key,'leftarrow')
            obj.arrowKeyState(3)=0;
            obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);            
        elseif strcmp(event.Key,'downarrow')
            obj.arrowKeyState(4)=0;
            obj.manualControlRobots(obj.motorValue*Muran.ArrowKeyDef*obj.arrowKeyState,true);            
        end
    end    
end

function onConnectButtonPressed(obj,src,event)
    obj.hOmnikit.connect();
    fprintf(2,'OmnikitRemote:connect=%d\n',obj.hOmnikit.isConnected);
    obj.hStrokeManager.connect();
    fprintf(2,'StrokeStateRemote:connect=%d\n',obj.hStrokeManager.isConnected);
end

function onMotorValueSliderChanged(obj,src,event)
    obj.motorValue = min(get(obj.hMotorValueSlider,'Value'), 1.0);
    fprintf(2,'Slider Event:MotorValue=%d\n',obj.motorValue);
end

function run(obj)
if Muran.Debug    
obj.hFig2=figure(2); clf(2);
for I=1:25
    subplot(5,5,I);
    obj.hTemplate(I)=imshow(zeros(4,4));
end
obj.hRobotPIDControl=RobotPIDControl(obj.camera.fps,obj.hOmnikit,'normal');
obj.hRobotPIDControl.initialize;
end
obj.hFig1=figure(1); clf(1);
set(obj.hFig1, 'KeyPressFcn',           {@obj.onKeyPressed})
set(obj.hFig1, 'KeyReleaseFcn',         {@obj.onKeyReleased})
binImg=zeros(obj.camera.height,obj.camera.width);
% subplot(1,2,1);
subplot('Position',[0 0 0.5 1.0]);
obj.hImage=imshow(binImg); hold on;
obj.hContours=plot(0,0,'r.');
obj.hCorners=plot(0,0,'co');
obj.hAxisReal=gca;
obj.hMousePointerReal=plot(0,0,'xr','Visible','off','MarkerSize',20,'LineWidth',3);
obj.hRobotTargetPointReal=plot(NaN,NaN,'vg','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','g');
for I=1:obj.numRobots
    obj.hRobotTargetLineReal(I)=line([NaN NaN],[NaN NaN],'color','k','LineStyle',':');
    obj.hRobotTargetTextReal(I)=text(NaN,NaN,int2str(I),'color','k');
end
subplot(1,2,2);
obj.hQuiver=quiver([],[],[],[],0,'LineWidth',3,'Color','r'); hold on;
siz=30;
obj.hRobotTargetPointNormal=plot(NaN,NaN,'vg','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','g');

obj.hMotorValueSlider = uicontrol('Parent',obj.hFig1,'Style','slider',...
    'Position',[siz*4 5 siz*9 siz/2]) ;
set(obj.hMotorValueSlider, 'Max', 1.0);
set(obj.hMotorValueSlider, 'Min', 0.0);
set(obj.hMotorValueSlider, 'SliderStep', [0.1, 0.2]);
set(obj.hMotorValueSlider, 'Value', obj.motorValue);
set(obj.hMotorValueSlider, 'Callback', {@obj.onMotorValueSliderChanged})

obj.hConnectButton=uicontrol('Parent',obj.hFig1,'Style','pushbutton',...
        'Position',[0 5+siz siz*3 siz],'String','Connect');    
set(obj.hConnectButton,'Callback',{@obj.onConnectButtonPressed});
for I=1:obj.numRobots
    obj.hRobots(I)=line([-1 -1],[-1 -1],'LineWidth',3);
    obj.hRobotsText(I)=text(NaN,NaN,int2str(I),'BackgroundColor','c','FontSize',12);
    obj.hRobotTargetLineNormal(I)=line([NaN NaN],[NaN NaN],'color','k','LineStyle',':');
    obj.hRobotTargetTextNormal(I)=text(NaN,NaN,int2str(I),'color','k');
    
    obj.hRobotProxyButton(I)=uicontrol('Parent',obj.hFig1,'Style','togglebutton',...
        'Position',[siz*3+siz*I 5+siz siz siz],'String',int2str(I));    
    set(obj.hRobotProxyButton(I),'Callback',{@(src,event) obj.setRobotSelect(I,get(src,'Value'))});
end
button=uicontrol('Parent',obj.hFig1,'Style','togglebutton',...
        'Position',[siz*3+siz*(obj.numRobots+1) 5+siz siz*2 siz],'String','全選択');    
set(button,'Callback',{@(src,event) obj.setAllRobotSelect(get(src,'Value'))});
button=uicontrol('Parent',obj.hFig1,'Style','togglebutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*2 5+siz siz*2 siz],'String','左回転');    
set(button,'Callback',{@(src,event) obj.doRobotRotation(1,get(src,'Value'))});
button=uicontrol('Parent',obj.hFig1,'Style','togglebutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*4 5+siz siz*2 siz],'String','右回転');    
set(button,'Callback',{@(src,event) obj.doRobotRotation(-1,get(src,'Value'))});
button=uicontrol('Parent',obj.hFig1,'Style','pushbutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*6 5+siz siz*2 siz],'String','左向');    
set(button,'Callback',{@(src,event) obj.doControlRobotDirection(obj.robotSelect,pi/2)});
button=uicontrol('Parent',obj.hFig1,'Style','pushbutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*8 5+siz siz*2 siz],'String','下向');    
set(button,'Callback',{@(src,event) obj.doControlRobotDirection(obj.robotSelect,pi)});
button=uicontrol('Parent',obj.hFig1,'Style','pushbutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*10 5+siz siz*2 siz],'String','上向');    
set(button,'Callback',{@(src,event) obj.doControlRobotDirection(obj.robotSelect,0)});
button=uicontrol('Parent',obj.hFig1,'Style','pushbutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*12 5+siz siz*2 siz],'String','右向');    
set(button,'Callback',{@(src,event) obj.doControlRobotDirection(obj.robotSelect,-pi/2)});
button=uicontrol('Parent',obj.hFig1,'Style','pushbutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*14 5+siz siz*2 siz],'String','AllStop');    
set(button,'Callback',{@(src,event) obj.doRobotStop(ones(obj.numRobots,1))});
button=uicontrol('Parent',obj.hFig1,'Style','pushbutton',...
        'Position',[siz*3+siz*(obj.numRobots+1)+siz*16 5+siz siz*2 siz],'String','iPad');    
set(button,'Callback',{@(src,event) obj.hStrokeManager.updateTarget});

set(obj.hFig1, 'WindowButtonDownFcn',   {@obj.onMouseDown});
set(obj.hFig1, 'WindowButtonMotionFcn', {@obj.onMouseMove});
set(obj.hFig1, 'WindowButtonUpFcn',     {@obj.onMouseUp});

obj.hMessage=text(-1,-1.5,'message','FontSize',12);
axis equal; axis([-1 1 -1 1]);
obj.hAxisNormal=gca;
obj.hMousePointerNormal=plot(0,0,'xr','Visible','off','MarkerSize',20,'LineWidth',3);

while(true)
timer=tic;    
obj.updateRobotPosition();
if (obj.hOmnikit.isConnected)
    set(obj.hConnectButton,'BackgroundColor','y');
    obj.moveRobots(); 
    obj.hRobotFeedbackController.doFeedbackControl;
else 
    set(obj.hConnectButton,'BackgroundColor','b');
end
if Muran.Debug
    id=obj.hRobotPIDControl.robotID;
    if strcmp(obj.hRobotPIDControl.pidMode,'angle')
      obj.hRobotPIDControl.doSampleProcess(obj.robotStateNormal(3,obj.hRobotPIDControl.robotID));    
    else
        if ~isnan(obj.robotTargetPointNormal(1,id))
            NX=obj.robotTargetPointNormal(:,id)-obj.robotStateNormal([1 2],id);
            [Y,~]=obj.normalVectorToMotorControlVector(id,NX);
            fprintf(2,'[xy PID]NX(%f,%f)->Y(%f,%f)\n',NX(1),NX(2),Y(1),Y(2));
            obj.hRobotPIDControl.doSampleProcess(-norm(Y),Y);
        end
    end
end
drawnow;
lostT=toc(timer);
pause(max(0,1/obj.camera.fps-lostT));
% deltaT=toc(timer); fprintf(2,'[FPS]=%f\n',1/deltaT);
end
end

end

methods(Static)
    
function main(width,height,fps,calib,level,range)
javaaddpath({'muran_matlab.jar','muran_matlab_stroke.jar'});
javaclasspath('-dynamic');    
if ~exist('width','var'); width=Muran.DefaultWidth; end;
if ~exist('height','var'); height=Muran.DefaultHeight; end;
if ~exist('fps','var'); fps=Muran.DefaultFPS; end;
if ~exist('calib','var'); calib='RGB'; end;
if ~exist('level','var'); level=0; end;
if ~exist('range','var'); range=[Muran.MinContourLength Muran.MaxContourLength]; end;
    m=Muran(width,height,fps,calib,level,range);
    onCleanup(@() m.delete());
    if (strcmpi(calib,'skip'))
        fprintf(2,'Homography Matrix Load from %s\n',Muran.DefaultDataFile);
        load(Muran.DefaultDataFile);
        m.H=HomographyMatrix;
        m.invH=inv(m.H);
    else 
        [m.H,m.invH]=m.runCalibration();        
    end
    m.run();
end

function test_show(filename)
if ~exist('filename','var'); filename=Muran.SampleImg; end;
colorImg=imread([Muran.DefaultDataDir filename]);
calibPoints=Marker.calc_calibration_points(colorImg);
[H,invH]=Marker.calc_homography_matrix(calibPoints);
grayImg = rgb2gray(colorImg);
binRawImg = grayImg < Muran.DefaultBinarizeThreshold;
binImg = imopen(binRawImg,strel('square',floor(length(binRawImg)/Muran.DefaultCleaningElementFactor)));
[labelImg,numLabels] = bwlabel(Img.set_edge(binImg,0),8);
CsRaw=Img.contours(labelImg,numLabels);
CsRawPoints=cell2mat(CsRaw);
Cs=Img.contours_in_length(labelImg,numLabels,Muran.MinContourLength,Muran.MaxContourLength);
CsPoints=cell2mat(Cs);
Corners=cell(size(Cs));
for I=1:length(Corners)
    RowCol=Cs{I};
    % k を N/8 にすることで、4点に絞り込むことが容易になる
    Corners{I}=Img.corner_detect(RowCol,size(RowCol,1)/4,size(RowCol,1)/8,0.2); 
end
if numel(Corners) == 0 
    disp('No Corner Detected(1)');
    return;
end
Corners=Util.filter_cell(Corners,@(x) length(x)==4);
Corners=Img.filter_outers(Corners);
if numel(Corners) == 0 
    disp('No Corner Detected(2)');
    return;
end
numMarkers=length(Corners);
RectReal=zeros(2,4,numMarkers);
RectNormal=zeros(2,4,numMarkers);
RectNormalRotated=zeros(2,4,numMarkers);
TemplatePoints=zeros(2,16,numMarkers);
Template=zeros(numMarkers,16);
TemplateID=zeros(numMarkers);
TemplateRotation=zeros(numMarkers);
for I=1:numMarkers
%     R=flipud(Corners{I}');
      R=circshift(rot90(Corners{I}',2),[0 1]); % 本当？
    RectReal(:,:,I)=R;
    RN=H*[R;ones(1,4)];
    RectNormal(:,:,I)=[RN(1,:)./RN(3,:); RN(2,:)./RN(3,:)];
    TemplatePoints(:,:,I)=Marker.template_points(R);
    Template(I,:)=Marker.extract_template_near(binImg,TemplatePoints(:,:,I));
    [id,rot]=Marker.decode_template1_with_rotation(Template(I,:));
    TemplateID(I)=id;
    TemplateRotation(I)=rot;
    RectReal(:,:,I)=circshift(R,[0 rot]);
%     RectNormal(:,:,I)=circshift(RectNormal(:,:,I),[0 rot]);
    RectNormalRotated(:,:,I)=circshift(RectNormal(:,:,I),[0 rot]);
end
figure(1); clf(1);
subplot(3,2,1); imshow(grayImg);
subplot(3,2,2); imshow(binRawImg);
subplot(3,2,3); imshow(binImg);
subplot(3,2,4); imshow(labelImg,[]);
subplot(3,2,5); hold off; imshow(binImg); hold on; plot(CsRawPoints(:,2),CsRawPoints(:,1),'.r'); hold off
subplot(3,2,6); hold off; imshow(binImg); hold on; plot(CsPoints(:,2),CsPoints(:,1),'.r'); hold off;
figure(2); clf(2);
subplot(2,1,1); hold off; imshow(binImg); hold on;
for I=1:numMarkers
plot(RectReal(1,:,I),RectReal(2,:,I),'.r'); 
plot(TemplatePoints(1,:,I),TemplatePoints(2,:,I),'.c');     
text(RectReal(1,1,I)-10,RectReal(2,1,I)-10,int2str(I));
end
for I=1:numMarkers
subplot(2,2,3);
plot(RectNormal(1,:,I),RectNormal(2,:,I),'x-'); hold on;    
text(mean(RectNormal(1,:,I)),mean(RectNormal(2,:,I)),int2str(I));
subplot(2,2,4);
plot(RectNormalRotated(1,:,I),RectNormalRotated(2,:,I),'x-'); hold on;    
text(mean(RectNormalRotated(1,:,I)),mean(RectNormalRotated(2,:,I)),int2str(TemplateID(I)));
end
subplot(2,2,3);
axis equal; axis([-1 1 -1 1]); hold off;
title('Markers');
subplot(2,2,4);
axis equal; axis([-1 1 -1 1]); hold off;
title('Markers(rotated)');
figure(3); clf(3);
for I=1:numMarkers
subplot(5,4,I);
imshow(reshape(~Template(I,:),4,4));
title(sprintf('[%d]id=%d, rot=%d',I,TemplateID(I),90*TemplateRotation(I)));
end

end

function [H,Hinv]=calibration_by_color_markers()
colorImg=imread([Muran.DefaultDataDir filename]);
calibPoints=Marker.calc_calibration_points(colorImg);
[H,invH]=Marker.calc_homography_matrix(calibPoints);    
end

function test_camera_capture_finite()
cm=CameraManager();
% camera=cameraManager.setupCameraDevice(0,320,240,30);
cam=cm.setupCameraDevice(0,640,480,30);
% camera=cameraManager.setupCameraDevice(0,960,720,30);
figure(1);
img=cam.createBuffer;
hImg=imshow(img);
drawnow;
cam.setBrightness(0.8);
for I=1:100
img=cam.captureFrame;
set(hImg,'CData',img);
drawnow;
end
cam.delete;
cm.delete;
end

function test_camera_capture_infinite()
cm=CameraManager();
cam=cm.setupCameraDevice(0,640,480,5);
    function closeCamera(mgr,cam)
        cam.delete;
        mgr.delete;
        disp('camera delete done');
    end
c=onCleanup(@() closeCamera(cm,cam));
figure(1); clf(1);
img=cam.createBuffer;
hImg=imshow(img);
while(true)
img=cam.captureFrame;   
set(hImg,'CData',img);
drawnow;
end
end

function realtime_calibration()
cm=CameraManager();
cam=cm.setupCameraDevice(0,640,480,5);
    function closeCamera(mgr,cam)
        cam.delete;
        mgr.delete;
        disp('camera delete done');
    end
c=onCleanup(@() closeCamera(cm,cam));
figure(1); clf(1);
img=cam.createBuffer;
binImg=zeros(cam.height,cam.width);
subplot(1,2,1); hImg=imshow(img); hold on; hRect=plot(0,0,'or');
subplot(2,4,5); hRed=imshow(binImg);
subplot(2,4,7); hGreen=imshow(binImg);
subplot(2,4,8); hBlue=imshow(binImg);
subplot(2,4,6); hCyan=imshow(binImg);
cam.setBrightness(0.8);
while(true)
img=cam.captureFrame; set(hImg,'CData',img);
I=Img.get_marker_red_bin(img); set(hRed,'CData',I);
I=Img.get_marker_green_bin(img); set(hGreen,'CData',I);
I=Img.get_marker_blue_bin(img); set(hBlue,'CData',I);
I=Img.get_marker_cyan_bin(img); set(hCyan,'CData',I);
try 
    X=Marker.calc_calibration_points(img);
    set(hRect,'XData',X(1,:));
    set(hRect,'YData',X(2,:));
catch ex
    disp('calibration fail');
end
drawnow;
end
end

end
end
