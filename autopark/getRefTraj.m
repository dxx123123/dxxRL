function Xref = getRefTraj(map,egoInitialPose,Ts,Tf)
% 将从地图获得的[x，y，θ]路径转换为基于时间的路径用于参考跟踪的轨迹。

    [xref0,yref0,tref0] = getReferencePath(map);
    dist = (xref0-egoInitialPose(1)).^2 + (yref0-egoInitialPose(2)).^2 + 10*(tref0-egoInitialPose(3)).^2;
    [~,startIdx] = min(dist);
    xrefReal = [xref0(startIdx:end),xref0(1:end),xref0(1:end)];
    yrefReal = [yref0(startIdx:end),yref0(1:end),yref0(1:end)];
    trefReal = [tref0(startIdx:end),2*pi+tref0(1:end),4*pi+tref0(1:end)];
    Tsteps = Tf/Ts;
    xRef = [xrefReal;yrefReal;trefReal]';
    p = size(xRef,1);
    Xref = [xRef(1:p,:);repmat(xRef(end,:),Tsteps-p,1)];
end