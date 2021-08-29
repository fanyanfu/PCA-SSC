%%激光雷达数据读取
clc;
clear all;
close all;
data_path ='/home/vision/deep_learning/lidar-bonnetal-master/train/tasks/semantic/dataset/sequences/00/';
lidar_data_dir = strcat(data_path, 'velodyne/');
data_names = osdir(lidar_data_dir);
data_ptcloud=cell(1,4540);
tic
for i=1:4540
    file_name = data_names{i};
    data_time = str2double(file_name(1:end-4));
    data_path = strcat(lidar_data_dir, file_name);
    ptcloud = readBin(data_path);
    ptcloud_data=ptcloud.Location;
    data_ptcloud{i}=ptcloud_data;
end
toc