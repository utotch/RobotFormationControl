classdef RobotController < handle
properties
hMuran;    
hOmnikit;    
numRobots;
angleFeedbackController;
xyFeedbackController;
end
    
methods

function obj=RobotController(hMuran,hOmnikit,numRobots,fps)
assert(isa(hOmnikit,'utotch.arduino.OmnikitRemote'));
obj.hMuran=hMuran;
obj.hOmnikit=hOmnikit;
obj.numRobots=numRobots;
% GainA=[0.058 0.002 0.006 0.055];
GainA=[0.058 0.002 0.006 0.060];
% GainXY=[0.76 0.006 0.113 0.065];
GainXY=[0.80 0.006 0.113 0.065];
obj.angleFeedbackController=[];
obj.xyFeedbackController=[];
for id=1:numRobots
%     obj.angleFeedbackController(id)=PIDController(GainA,fps,[-pi pi],[-1.0 1.0],true);
%     obj.angleFeedbackController(id).setTargetValue(NaN);
%     obj.xyFeedbackController(id)=PIDController(GainXY,fps,[-1 1],[0 1.0],false);
%     obj.xyFeedbackController(id).setTargetValue(0.0);
    obj.angleFeedbackController=[obj.angleFeedbackController PIDController(GainA,fps,[-pi pi],[-1.0 1.0],true)];
    obj.angleFeedbackController(id).setTargetValue(NaN);
    obj.xyFeedbackController=[obj.xyFeedbackController PIDController(GainXY,fps,[-1 1],[0 1.0],false)];
    obj.xyFeedbackController(id).setTargetValue(0.0);
end

end

function doFeedbackControl(obj)
for id=1:obj.numRobots
    if obj.angleFeedbackController(id).isEnable
      val=obj.angleFeedbackController(id).doSampleProcess(obj.hMuran.robotStateNormal(3,id));
      if obj.angleFeedbackController(id).isFinished
          obj.angleFeedbackController(id).doFinish;
          obj.angleFeedbackController(id).setTargetValue(NaN);
          obj.hOmnikit.motorStop(id);
          obj.hOmnikit.flush(id);
          set(obj.hMuran.hRobotsText(id),'BackgroundColor','c');  
      else
          obj.hOmnikit.rotate(id,val);
          obj.hOmnikit.flush(id);
          set(obj.hMuran.hRobotsText(id),'BackgroundColor','r');
      end
    else
          set(obj.hMuran.hRobotsText(id),'BackgroundColor','c');  
    end
    if obj.xyFeedbackController(id).isEnable
        if ~isnan(obj.hMuran.robotTargetPointNormal(1,id))
            NX=obj.hMuran.robotTargetPointNormal(:,id)-obj.hMuran.robotStateNormal([1 2],id);
            if obj.xyFeedbackController(id).isFinished
                obj.xyFeedbackController(id).doFinish;
                obj.hOmnikit.motorStop(id);
                obj.hOmnikit.flush(id);
                obj.hMuran.resetRobotTarget(id);
                set(obj.hMuran.hRobotsText(id),'BackgroundColor','c');  
            else
                [Y,~]=obj.hMuran.normalVectorToMotorControlVector(id,NX);
                vec=obj.xyFeedbackController(id).doSampleProcess(-norm(Y),Y);
                obj.hOmnikit.moveToDirection(id,vec(1),vec(2));
                obj.hOmnikit.flush(id);
                set(obj.hMuran.hRobotsText(id),'BackgroundColor','r');        
            end
        end        
    else
        set(obj.hMuran.hRobotsText(id),'BackgroundColor','c');              
    end    
end
end
        
end
    
end

