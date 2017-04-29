function [confidence] = getConfidence(Dist,Normals)

theta3 = 0.5;
confidence = exp(-theta3*asin(((dot(Dist,Normals))./norm(Dist))^2));

