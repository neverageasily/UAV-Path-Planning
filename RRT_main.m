clc;
clear;
close all;

obj_data
numofobs = length(obs(:,1));

cmap=[1 1 1;
    0 0 0;
    1 0 0;
    0 0 1;
    0 1 0;
    1 1 0];
colormap(cmap);
map=ones(25);

for k=1:numofobs
    row=obs(k,1);
    col=obs(k,2);
    map(row,col)=2;
end

start_row=1;
start_col=25;
goal_row=25;
goal_col=1;

map(start_row,start_col)=5;
map(goal_row,goal_col)=6;

image(1.5,1.5,map);
grid on;
axis image;

tree=[start_row,start_col,0];
step_size=1;
max_iter=5000;

nrows=size(map,1);
ncols=size(map,2);

found=false;

tic;

for iter=1:max_iter

    q_rand=[randi(nrows),randi(ncols)];
    
    if map(q_rand(1),q_rand(2))==2
        continue;
    end

    dist=zeros(size(tree,1),1);

    for j=1:size(tree,1)
        dist(j)=sqrt((tree(j,1)-q_rand(1))^2+(tree(j,2)-q_rand(2))^2);
    end

    [~,near_idx]=min(dist);
    q_near=tree(near_idx,1:2);

    direction=q_rand-q_near;
    step=sign(direction);
    q_new=q_near+step_size*step;

    if q_new(1)<1||q_new(1)>nrows||q_new(2)<1||q_new(2)>ncols
        continue;
    end

    if map(q_new(1),q_new(2))==2
        continue;
    end

    if any(tree(:,1)==q_new(1)&tree(:,2)==q_new(2))
        continue;
    end

    if step(1)~=0&&step(2)~=0
        side1=[q_near(1)+step(1),q_near(2)];
        side2=[q_near(1),q_near(2)+step(2)];
        if map(side1(1),side1(2))==2||map(side2(1),side2(2))==2
            continue;
        end
    end

    new_idx=size(tree,1)+1;
    tree(new_idx,:)=[q_new,near_idx];

    if q_new(1)==goal_row&&q_new(2)==goal_col
        found=true;
        goal_idx=new_idx;
        break;
    end
end

planning_time=toc;

if found
        route_idx=goal_idx;

        while tree(route_idx(1),3)~=0
            parent_idx=tree(route_idx(1),3);
            route_idx=[parent_idx,route_idx];
        end
else
        disp("no feasible path found");
end

if found
    route=tree(route_idx,1:2);
    
    for k=2:size(route,1)-1
        map(route(k,1),route(k,2))=6;
    end

    map(start_row,start_col)=5;
    map(goal_row,goal_col)=6;

    figure;
    colormap(cmap);
    image(1.5,1.5,map);
    grid on;
    axis image;

    steps=size(route,1)-1;
    distance=0;
    for k=1:size(route,1)-1
        distance=distance+sqrt((route(k+1,1)-route(k,1))^2+(route(k+1,2)-route(k,2))^2);
    end

    fprintf("plannning time:%.6f s\n",planning_time);
    fprintf("steps:%d\n",steps);
    fprintf("distance:%.2f m\n",distance);
end