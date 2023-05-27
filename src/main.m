clear
clearvars
close all

env1 = {collisionBox(170, 420, 50) collisionBox(420, 130, 50) collisionBox(420, 130, 50) collisionBox(70, 420, 50) collisionBox(90, 420, 50)};
env2 = {collisionBox(170, 420, 50) collisionBox(420, 130, 50) collisionBox(420, 130, 50) collisionBox(70, 420, 50) collisionBox(90, 420, 50)};
env3 = {collisionCylinder(72/2,600) collisionCylinder(140,25) collisionCylinder(120,25) collisionCylinder(110,25) collisionCylinder(80,25) collisionCylinder(100,25)};
env4 = {collisionCylinder(72/2,600) collisionCylinder(110,25) collisionCylinder(110,25) collisionCylinder(106,25) collisionCylinder(90,60) collisionCylinder(120,30) collisionCylinder(90,25) collisionCylinder(120,30)};

trnsf1 = trvec2tform([0 0 0])*axang2tform([0 1 0 pi/2]);
trnsf2 = trvec2tform([0 145 290])*axang2tform([0 1 0 pi/2]);
trnsf3 = trvec2tform([0 -145 290])*axang2tform([0 1 0 pi/2]);
trnsf4 = trvec2tform([0 0 160+170/2])*axang2tform([0 1 0 pi/2]);
trnsf5 = trvec2tform([0 0 160+170/2+160+45])*axang2tform([0 1 0 pi/2]);

env1{1}.Pose = trnsf1;
env1{2}.Pose = trnsf2;
env1{3}.Pose = trnsf3;
env1{4}.Pose = trnsf4;
env1{5}.Pose = trnsf5;

tf1 = trvec2tform([550 0 0])*axang2tform([0 1 0 pi/2]);
tf2 = trvec2tform([550 145 290])*axang2tform([0 1 0 pi/2]);
tf3 = trvec2tform([550 -145 290])*axang2tform([0 1 0 pi/2]);
tf4 = trvec2tform([550 0 160+170/2])*axang2tform([0 1 0 pi/2]);
tf5 = trvec2tform([550 0 160+170/2+160+45])*axang2tform([0 1 0 pi/2]);


env2{1}.Pose = tf1;
env2{2}.Pose = tf2;
env2{3}.Pose = tf3;
env2{4}.Pose = tf4;
env2{5}.Pose = tf5;


cy1tf1 = trvec2tform([300 0 155])*axang2tform([0 1 0 pi/2]);
cy1tf2 = trvec2tform([490 0 155])*axang2tform([0 1 0 pi/2]);
cy1tf3 = trvec2tform([268 0 155])*axang2tform([0 1 0 pi/2]);
cy1tf4 = trvec2tform([243 0 155])*axang2tform([0 1 0 pi/2]);
cy1tf5 = trvec2tform([200 0 155])*axang2tform([0 1 0 pi/2]);
cy1tf6 = trvec2tform([100 0 155])*axang2tform([0 1 0 pi/2]);
cy2tf1 = trvec2tform([100 0 150+200])*axang2tform([0 1 0 pi/2]);
cy2tf2 = trvec2tform([100 0 150+200])*axang2tform([0 1 0 pi/2]);
cy2tf3 = trvec2tform([200 0 150+200])*axang2tform([0 1 0 pi/2]);
cy2tf4 = trvec2tform([243 0 150+200])*axang2tform([0 1 0 pi/2]);
cy2tf5 = trvec2tform([268 0 150+200])*axang2tform([0 1 0 pi/2]);
cy2tf6 = trvec2tform([320 0 150+200])*axang2tform([0 1 0 pi/2]);
cy2tf7 = trvec2tform([355 0 150+200])*axang2tform([0 1 0 pi/2]);
cy2tf8 = trvec2tform([170 0 150+200])*axang2tform([0 1 0 pi/2]);


env3{1}.Pose = cy1tf1;
env3{2}.Pose = cy1tf2;
env3{3}.Pose = cy1tf3;
env3{4}.Pose = cy1tf4;
env3{5}.Pose = cy1tf5;
env3{6}.Pose = cy1tf6;

env4{1}.Pose = cy2tf1;
env4{2}.Pose = cy2tf2;
env4{3}.Pose = cy2tf3;
env4{4}.Pose = cy2tf4;
env4{5}.Pose = cy2tf5;
env4{6}.Pose = cy2tf6;
env4{7}.Pose = cy2tf7;
env4{8}.Pose = cy2tf8;

worldCollisionArray = {env1{1} env1{2} env1{3} env1{4} env1{5} env2{1} env2{2} env2{3} env2{4} env2{5} env3{1} env3{2} env3{3} env3{4} env3{5} env4{1} env4{2} env4{3} env4{4} env4{5} env4{6} env4{7} env4{8}};
CollisionArray = {env1{1} env1{2} env1{3} env1{4} env1{5} env2{1} env2{2} env2{3} env2{4} env2{5} env3{1} env3{2} env3{3} env3{4} env3{5}};
ax = collision_checker(CollisionArray);

for i=1:length(env4)
    show(env4{i},"Parent",ax)
    hold on
end

CollisionArray = {env1{1} env1{2} env1{3} env1{4} env1{5} env2{1} env2{2} env2{3} env2{4} env2{5} env3{1} env3{2} env3{3} env3{4} env3{5}};

EPS = 50;
numNodes = 5000;       

q_start.coord = [100 0 350];

q_start.cost = 0;
q_start.parent = 0;
q_start.rotation = 0;
q_goal.coord = [-400 50 200];
q_goal.cost = 0;


nodes = RRT_star_planner(ax, q_start, q_goal, numNodes, EPS, CollisionArray, env4);

n = nodes(end);
path = [];
ang_list = [];

while ~(n.coord==q_start.coord)
    path = [path ;[n.coord]];
    ang_list = [ang_list n.rotation];
    prev = n.coord;
    n = nodes(n.parent);
    plot3(ax,[n.coord(1), prev(1)], [n.coord(2), prev(2)], [n.coord(3), prev(3)], 'Color', 'b', 'LineWidth', 2);
    drawnow;
end

pause(10)

%For animation.
for p = length(path):-1:1
    for c = 1:length(CollisionArray)
        show(CollisionArray{c});
        hold on;
    end
    axis equal
    view(ax,[20,30]);
    env4{1}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 100-100]);
    env4{2}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 100-100]);
    env4{3}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 200-100]);
    env4{4}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 243-100]);
    env4{5}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 268-100]);
    env4{6}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 320-100]);
    env4{7}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 355-100]);
    env4{8}.Pose = trvec2tform(path(p,:))*axang2tform([0 1 0 pi/2])*axang2tform([0 1 0 -ang_list(p)])*trvec2tform([0 0 170-100]);
    show(env4{1});
    show(env4{2});
    show(env4{3});
    show(env4{4});
    show(env4{5});
    show(env4{6});
    show(env4{7});
    show(env4{8});
    hold off;
    drawnow;
end