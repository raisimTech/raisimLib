if ismac
    addpath('../../raisim/mac/lib')
elseif isunix
    addpath('../../raisim/linux/lib')
elseif ispc
    addpath('../../raisim/win32/mt_release/bin')
else
    disp('Platform not supported')
end

raisim('license', strcat(pwd, '/../../rsc/activation.raisim'));
raisim('init', strcat(pwd, '/../../rsc/laikago.xml'), 8080);

% the floating-base part of the gains are ignored
raisim('setPGains', 'laikago0', ones(18,1)*50);
raisim('setDGains', 'laikago0', ones(18,1));

raisim('setPTarget', 'laikago0', [0, 0, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -1]');
raisim('setDTarget', 'laikago0', ones(18,1));

for i=1:2000
    pause(1./500)
    raisim('integrate');
    [force, position, objects] = raisim('getContactForcePositionsObject', 'laikago0', 'FR_calf');
end

% articulated system properties
massMatrix = raisim('getMassMatrix', 'laikago0');
nonLinearForce = raisim('getNonlinearities', 'laikago0');
raisim('printOutFrameNamesInOrder', 'laikago0');
framePosition = raisim('getFramePosition', 'laikago0', 'FL_hip_joint');
frameVelocity = raisim('getFrameVelocity', 'laikago0', 'FR_hip_fixed');
frameOrientation = raisim('getFrameOrientation', 'laikago0', 'FR_hip_fixed');
frameAngularVelocity = raisim('getFrameAngularVelocity', 'laikago0', 'FR_hip_fixed');
frameJacobian = raisim('getDenseFrameJacobian', 'laikago0', 'FR_hip_fixed');
frameRotationalJacobian = raisim('getDenseFrameRotationalJacobian', 'laikago0', 'FR_hip_fixed');

% add 1kg 0.2m radius sphere
raisim('addSphere', 'sphere', 0.2, 1);
raisim('setPosition', 'sphere', -1, 0, 0.8);    
raisim('setVelocity', 'sphere', 3, 0, 0, 0, 0, 0);   

raisim('addCylinder', 'cylinder', 0.2, 0.5, 1);
raisim('setPosition', 'cylinder', rand()*2, rand()*2, rand()+2);
quat = rand(4,1); quat = quat ./ norm(quat);
% quaternion in [w,x,y,z] convention
raisim('setQuaternion', 'cylinder', quat(1), quat(2), quat(3), quat(4));

raisim('addCapsule', 'capsule', 0.2, 0.5, 1);
raisim('setPosition', 'capsule', rand()*2, rand()*2, rand()+2);
quat = rand(4,1); quat = quat ./ norm(quat);
raisim('setQuaternion', 'capsule', quat(1), quat(2), quat(3), quat(4));

raisim('addBox', 'box', 0.2, 0.4, 0.3, 2);
raisim('setPosition', 'box', rand()*2, rand()*2, rand()+5);
quat = rand(4,1); quat = quat ./ norm(quat);
raisim('setQuaternion', 'box', quat(1), quat(2), quat(3), quat(4));

raisim('addArticulatedSystem', 'anymal0', strcat(pwd,'../../rsc/anymal/urdf/anymal.urdf'));
raisim('setGeneralizedCoordinate', 'anymal0', [2, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8]');
raisim('setPGains', 'anymal0', ones(18,1) .* 200);
raisim('setDGains', 'anymal0', ones(18,1));
raisim('setPTarget', 'anymal0', [0, 0, 1.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8]');
raisim('setDTarget', 'anymal0', ones(18,1));
raisim('setExternalForce', 'anymal0', 'base_to_base_inertia', [10000.0, 0.0, 0.0]);

for i=1:5000
    pause(1./500)
    raisim('setGeneralizedForce', 'anymal0', zeros(18,1));
    raisim('integrate1');
    raisim('integrate2');
    [force, position, objects] = raisim('getContactForcePositionsObject', 'sphere');
end

box_position = raisim('getPosition', 'box');
box_orientation = raisim('getQuaternion', 'box');

raisim('quit');
