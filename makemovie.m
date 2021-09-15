%Tung_Eric_MiniProject1

close all;

framerate = 30;

straight_path = VideoWriter('straight_path'); %open video file
straight_path.FrameRate = framerate;
open(straight_path)

smooth_path = VideoWriter('smooth_path'); %open video file
smooth_path.FrameRate = framerate;
open(smooth_path)

M(length(l)) = struct('cdata',[],'colormap',[]);
Mm(smoothn) = struct('cdata',[],'colormap',[]);

room1=showroom(1,5,5);
for i=1:length(l)
    clf    
    room1=showroom(1,5,5);

    q = quiver(path12(1,i),path12(2,i),thetavectors_short(1,i),thetavectors_short(2,i),1);
    q.LineWidth = 2;
    robot.Pose(1:3,4)=[path12(:,i); -1];

    show(robot);
    drawnow
    
    pause(0.01) %Pause and grab frame

    M(i) = getframe;
    writeVideo(straight_path, M(i));
end

clf
room2=showroom(2,5,5);
for i=1:smoothn
    clf 
    room1=showroom(2,5,5);

    q = quiver(pathtotal(1,i),pathtotal(2,i),smooththetavectors_short(1,i),smooththetavectors_short(2,i),1);
    q.LineWidth = 2;
    robot.Pose(1:3,4)= [pathtotal(:,i); -1];
    
    show(robot);
    drawnow
    
    pause(0.01) %Pause and grab frame

    Mm(i) = getframe;
    writeVideo(smooth_path, Mm(i));
end

%movie(M,100,framerate)

%movie(Mm,100,framerate)




function h=showroom(fignum,xdim,ydim)
    h=figure(fignum);
    hold on
    
    % adjust the view so you can see from above
    view(0,90)
    % adjust the figure range so you can see the 5x5 room
    axis([0 xdim 0 ydim]);
    grid on 
    axis square
    grid minor
    set(gca,'xtick',[0:0.5:5])
    set(gca,'ytick',[0:0.5:5])
    
    zed=.1;
    smolthick = 0.025;

    w1 = collisionBox(0.05,5,zed);
    w1.Pose(1:3,4)=[0-smolthick;2.5;-1];
    w1.Pose(1:2,1:2)=rot2(0);

    w2 = collisionBox(0.05,5,zed);
    w2.Pose(1:3,4)=[2.5;5+smolthick;-1];
    w2.Pose(1:2,1:2)=rot2(pi/2);

    w3 = collisionBox(0.05,5,zed);
    w3.Pose(1:3,4)=[5+smolthick;2.5;-1];
    w3.Pose(1:2,1:2)=rot2(0);

    w4 = collisionBox(0.05,5,zed);
    w4.Pose(1:3,4)=[2.5;0-smolthick;-1];
    w4.Pose(1:2,1:2)=rot2(pi/2);

    human = collisionCylinder(0.2,zed);
    human.Pose(1:3,4)=[1;0.5;-1];
    human.Pose(1:2,1:2)=rot2(0);

    table = collisionBox(0.5,0.5,zed);
    table.Pose(1:3,4)=[2.5;2.5;-1];
    table.Pose(1:2,1:2)=rot2(pi/4);

    shelf = collisionBox(0.8,0.3,zed);
    shelf.Pose(1:3,4)=[1;4;-1];
    shelf.Pose(1:2,1:2)=rot2(-pi/2);

    % adjust the view so you can see from above

    % adjust the figure range so you can see the 5x5 room
    axis([-2*smolthick 5+2*smolthick -2*smolthick 5+2*smolthick]);

    show(w1)
    show(w2)
    show(w3)
    show(w4)
    show(human)
    show(table)
    show(shelf)
end

function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end



