classdef CameraDevice
properties(Constant)
end
methods(Static)
end

properties
cameraManagerRef; 
deviceId;
width=640;
height=480;
fps=30;
% pixels;
end

methods

function obj=CameraDevice(cameraManagerRef,deviceId,width,height,fps)
obj.cameraManagerRef=cameraManagerRef;    
obj.deviceId=deviceId;
obj.width=width;
obj.height=height;
obj.fps=fps;
% obj.pixels=zeros(height,width,3,'uint8');
end
   
function img=createBuffer(obj)
img=zeros(obj.height,obj.width,3,'uint8');
end

function setBrightness(obj,val)
vi_set_video_setting(obj.cameraManagerRef.videoInputRef,obj.deviceId,1,val,false);
end

function setContrast(obj,val)
vi_set_video_setting(obj.cameraManagerRef.videoInputRef,obj.deviceId,2,val,false);
end

function img=captureFrame(obj)
vi_is_frame_new(obj.cameraManagerRef.videoInputRef,obj.deviceId);
img=vi_get_pixels(obj.cameraManagerRef.videoInputRef,obj.deviceId);
end

function delete(obj)
vi_stop_device(obj.cameraManagerRef.videoInputRef,obj.deviceId);    
end
    
end
end
