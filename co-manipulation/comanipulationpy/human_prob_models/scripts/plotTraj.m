function[] = plotTraj(trajDir)

traj = readmatrix(trajDir);
ts = 0;
for row = 1:size(traj,1)
    ts
    ts = ts + 1;
    shoulder = traj(row, 1:3);
    elbow = traj(row, 4:6);
    wrist = traj(row, 7:9);
    palm = traj(row, 10:12);
    
    pts = [shoulder; elbow; wrist; palm];
    plot3(pts(:,1), pts(:,2), pts(:,3))
    axis([1 2 0.5 1.5 0 1.5])
    pause(.1)
end

%P1 = [0,0,0];
%P2 = [13,-11,19];

% Their vertial concatenation is what you want
%pts = [P1; P2];

% Because that's what line() wants to see    
%line(pts(:,1), pts(:,2), pts(:,3))

% Alternatively, you could use plot3:
%plot3(pts(:,1), pts(:,2), pts(:,3))