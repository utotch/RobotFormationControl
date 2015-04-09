classdef CameraManager
properties
videoInputRef;
numDevices=0;
end
    
methods 

function obj=CameraManager()
obj.videoInputRef=vi_create();
obj.numDevices=vi_list_devices(obj.videoInputRef);
fprintf(1, '%d cameras detected.\n', obj.numDevices);
end

function camera=setupCameraDevice(obj,id,width,height,fps)
assert(0<=id && id < obj.numDevices);
vi_setup_device(obj.videoInputRef,id,width,height,fps);
camera=CameraDevice(obj,id,width,height,fps);
end

function delete(obj)
vi_delete(obj.videoInputRef);
end

end

end

