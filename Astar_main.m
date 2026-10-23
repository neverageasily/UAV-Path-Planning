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

start_node=sub2ind(size(map),start_row,start_col);
goal_node=sub2ind(size(map),goal_row,goal_col);

image(1.5,1.5,map);
grid on;
axis image;

nrows=size(map,1);
ncols=size(map,2);

G=Inf(nrows,ncols);
G(start_node)=0;

[X,Y]=meshgrid(1:ncols,1:nrows);
H=abs(X-goal_col)+abs(Y-goal_row);

F=Inf(nrows,ncols);
F(start_node)=G(start_node)+H(start_node);

parent=zeros(nrows,ncols);

closed=false(nrows,ncols);

tic;

while true
    [min_F,current]=min(F(:));
    
    if isinf(min_F)
        disp("no feasible path found");
        break;
    end

    if current==goal_node
        break;
    end

    closed(current)=true;
    F(current)=Inf;

    if current~=start_node&&current~=goal_node
        map(current)=3;
    end

    [row,col]=ind2sub(size(map),current);
    neighbor=[row-1,col;
    row+1,col;
    row,col-1;
    row,col+1];

    outrangetest=(neighbor(:,1)<1)+(neighbor(:,1)>nrows)+(neighbor(:,2)<1)+(neighbor(:,2)>ncols);
    locate=find(outrangetest>0);
    neighbor(locate,:)=[];
    neighborIndex=sub2ind(size(map),neighbor(:,1),neighbor(:,2));
    
    for i=1:length(neighborIndex)
        next=neighborIndex(i);
        
        if map(next)==2||closed(next)
        continue;
        end

        tentative_G=G(current)+1;
        
        if tentative_G<G(next)
            G(next)=tentative_G;
            parent(next)=current;
            F(next)=G(next)+H(next);

            if next~=goal_node&&next~=start_node
                map(next)=4;
            end
        end
    end
    
    %image(1.5,1.5,map);
    %grid on;
    %axis image;
    %drawnow;

end

planning_time=toc;

if isinf(G(goal_node))
    route=[];
else
    route=goal_node;

    while parent(route(1))~=0
        route=[parent(route(1)),route];
    end
end

if ~isempty(route)

    for k=2:length(route)-1
        map(route(k))=6;
    end

    map(start_node)=5;
    map(goal_node)=6;

    figure;
    colormap(cmap);
    image(1.5,1.5,map);
    grid on;
    axis image;

    steps=length(route)-1;
    distance=steps;

    fprintf("planning time:%.6f s\n",planning_time);
    fprintf("steps:%d\n",steps);
    fprintf("distance:%.2f m\n",distance);
end