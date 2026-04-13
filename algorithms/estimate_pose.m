function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

if public_vars.counter > 20
    estimated_pose = median(public_vars.particles);
else
    estimated_pose = nan(1,3);
end

end

