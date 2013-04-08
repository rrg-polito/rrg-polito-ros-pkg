%clear all
close all
clc

data = csvread('loops.csv');
edges = csvread('edges.csv');



pose_edges = [0 0 0];
for i=1:length(edges(:,1))
    x_prev = pose_edges(i,1);
    y_prev = pose_edges(i,2);
    th_prev = pose_edges(i,3);
    deltax = edges(i,3 ) 
    deltay = edges(i,4);
    deltath = edges(i,5);
    x_next = cos(th_prev)*deltax - sin(th_prev) * deltay + x_prev;
    y_next = sin(th_prev)*deltax + cos(th_prev) * deltay + y_prev;  
    th_next = th_prev + deltath;
    pose_edges(end+1,:) = [x_next y_next th_next];
end

figure
plot(pose_edges(:,1),pose_edges(:,2),'.-m')
hold on


%figure
%plot(poses(:,1),poses(:,2),'.r')
%hold on


for i=1:length(data(:,1)) % for each loop closing
    a = data(i,1);
    b = data(i,2);
    xya = pose_edges(a+1,1:2);
    xyb = pose_edges(b+1,1:2);
        
    deltax = data(i,3);
    deltay = data(i,4);
    th = pose_edges(a+1,3);
    
%     if b~=a+1 % if it is a loop closing
        error_lc = (data(i,end)); % we show the matching error
        xe = cos(th)*deltax - sin(th) * deltay + xya(1);
        ye = sin(th)*deltax + cos(th) * deltay + xya(2);
%         if error_lc<0.3  && error_lc > 0.1
        plot([xya(1) xe],[xya(2) ye],'--g')
        plot([xya(1) xyb(1)],[xya(2) xyb(2)],':k')   
        %pause
        
%     end
end
