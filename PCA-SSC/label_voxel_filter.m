function [ptcloud2,label2]=label_voxel_filter(ptcloud1,label1)
cellsize = 0.5;
pc_point = ptcloud1.Location;
xlimit = ptcloud1.XLimits;
ylimit = ptcloud1.YLimits;
zlimit = ptcloud1.ZLimits;

W = floor((xlimit(2) - xlimit(1))/cellsize)+1;
H = floor((ylimit(2) - ylimit(1))/cellsize)+1;
D = floor((zlimit(2) - zlimit(1))/cellsize)+1;

voxel = cell(W,H,D);
pointre =[];
label2=[];
for i =1:length(pc_point)
    I = floor((pc_point(i,1)-xlimit(1))/cellsize)+1;
    J = floor((pc_point(i,2)-ylimit(1))/cellsize)+1;
    K = floor((pc_point(i,3)-zlimit(1))/cellsize)+1;
    if(isempty(voxel{I,J,K}))
        pointre=[pointre;pc_point(i,:)];
        label2=[label2,label1(i)];
        voxel{I,J,K} = [voxel{I,J,K};pc_point(i,:)];
    else
        continue;
    end
    
end
ptcloud2=pointCloud(pointre);
end