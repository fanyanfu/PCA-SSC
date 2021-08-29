function [scancontexts, ringkeys, xy_poses] = makeExperience(data_dir, shape, skip_data_frame)

%%
num_rings = shape(1);
num_sectors = shape(2);

%%
lidar_data_dir = strcat(data_dir, 'velodyne/');
label_data_dir=strcat(data_dir, 'labels/');
label_names=osdir(label_data_dir);
data_names = osdir(lidar_data_dir);

%% gps to xyz
if(1) %为后续验证
    load poses.mat
    gtpose_xy=poses;
end


%%
num_data = length(data_names);
num_data_save = floor(num_data/skip_data_frame) + 1;
save_counter = 1;

scancontexts = cell(1, num_data_save);
ringkeys = zeros(num_data_save, num_rings);
xy_poses = zeros(num_data_save, 2);

for data_idx = 1:num_data%num_data
    if(rem(data_idx, skip_data_frame) ~=0)
        continue;
    end
    
    file_name = data_names{data_idx};
    label_file_name=label_names{data_idx};
    data_time = str2double(file_name(1:end-4));
    data_path = strcat(lidar_data_dir, file_name);
    label_data_path=strcat(label_data_dir, label_file_name);
    
    % get
    ptcloud = readBin(data_path);
    label1=label2mat(label_data_path);
    [ptcloud1,label1]=pointlabelpca(ptcloud,label1);%%%进行PCA处理
    [sc,cell_img] = Ptcloud2ScanContext1(ptcloud1, shape(2), shape(1),80,label1);
%     [sc,cell_img] = Ptcloud2ScanContext_sc2four(ptcloud, shape(2), shape(1), 80); % up to 80 meter
    %sc=sc_dealwith(sc); %已经在上一个函数Ptcloud2ScanContext1中进行处理
    rk = ringkey(sc);
    
%     [nearest_time_gap, nearest_idx] = min(abs(repmat(data_time, length(gtpose_time), 1) - gtpose_time));
    xy_pose = gtpose_xy(data_idx, :);
    
    % save 
    scancontexts{save_counter} = cell_img;
    ringkeys(save_counter, :) = rk;
    xy_poses(save_counter, :) = xy_pose;
    save_counter = save_counter + 1;

    % log
    if(rem(data_idx, 100) == 0)
        message = strcat(num2str(data_idx), " / ", num2str(num_data), " processed (skip: ", num2str(skip_data_frame), ")");
        disp(message); 
    end
end

scancontexts = scancontexts(1:save_counter-1);
ringkeys = ringkeys(1:save_counter-1, :);
xy_poses = xy_poses(1:save_counter-1, :);

end
