function [ img,cell_img] = Ptcloud2ScanContext1( ptcloud, num_sector, num_ring, max_range,label )

%% Preprocessing 

% Downsampling for fast search
if(0)
    gridStep = 0.5; % 0.5m cubic grid downsampling is applied in the paper.
    ptcloud = pcdownsample(ptcloud, 'gridAverage', gridStep);
end
[ptcloud,label]=label_voxel_filter(ptcloud,label);

cell_img=cell(1,4);
% point cloud information 
num_points = ptcloud.Count;
gap = max_range / num_ring; 
angle_one_sector = 360/num_sector;
%%%
cell_bin_counter = ones(num_ring, num_sector);
%%%
cell_bin_counter1=ones(num_ring, num_sector);
cell_bin_counter2=ones(num_ring, num_sector);
cell_bin_counter3=ones(num_ring, num_sector);
%%%
cell_bin_index=cell(num_ring, num_sector);
%%%
cell_bin_index1=cell(num_ring, num_sector);
cell_bin_index2=cell(num_ring, num_sector);
cell_bin_index3=cell(num_ring, num_sector);
%%%

%% Save a point to the corresponding bin 
for ith_point =1:num_points

    % Point information 
    ith_point_xyz = ptcloud.Location(ith_point,:);
    ith_point_r = sqrt(ith_point_xyz(1)^2 + ith_point_xyz(2)^2);
    ith_point_theta = XY2Theta(ith_point_xyz(1), ith_point_xyz(2)); % degree
    ith_point_theta1 = XY2Theta(-ith_point_xyz(1), ith_point_xyz(2)); % degree
    ith_point_theta2 = XY2Theta(-ith_point_xyz(1), -ith_point_xyz(2)); % degree
    ith_point_theta3 = XY2Theta(ith_point_xyz(1), -ith_point_xyz(2)); % degree
    % Find the corresponding ring index 
    tmp_ring_index = floor(ith_point_r/gap);
    if(tmp_ring_index >= num_ring)
        ring_index = num_ring;
    else
        ring_index = tmp_ring_index + 1;
    end
    
    % Find the corresponding sector index 
    tmp_sector_index = ceil(ith_point_theta/angle_one_sector);
    if(tmp_sector_index == 0)
        sector_index = 1;
    elseif(tmp_sector_index > num_sector || tmp_sector_index < 1)
        sector_index = num_sector;
    else
        sector_index = tmp_sector_index;
    end
    
    tmp_sector_index1 = ceil(ith_point_theta1/angle_one_sector);
    if(tmp_sector_index1 == 0)
        sector_index1 = 1;
    elseif(tmp_sector_index1 > num_sector || tmp_sector_index1 < 1)
        sector_index1 = num_sector;
    else
        sector_index1 = tmp_sector_index1;
    end
    
    tmp_sector_index2 = ceil(ith_point_theta2/angle_one_sector);
    if(tmp_sector_index2 == 0)
        sector_index2 = 1;
    elseif(tmp_sector_index2 > num_sector || tmp_sector_index2 < 1)
        sector_index2 = num_sector;
    else
        sector_index2 = tmp_sector_index2;
    end
    
    tmp_sector_index3 = ceil(ith_point_theta3/angle_one_sector);
    if(tmp_sector_index3 == 0)
        sector_index3 = 1;
    elseif(tmp_sector_index3 > num_sector || tmp_sector_index3 < 1)
        sector_index3 = num_sector;
    else
        sector_index3 = tmp_sector_index3;
    end
    
    
    % Assign point to the corresponding bin cell 
    try
        corresponding_counter = cell_bin_counter(ring_index, sector_index); % 1D real value.
    catch
        continue;
    end
    cell_bin_index{ring_index, sector_index}=[cell_bin_index{ring_index, sector_index},label(ith_point)];
    cell_bin_counter(ring_index, sector_index) = cell_bin_counter(ring_index, sector_index) + 1; % increase count 1
    
    
    try
        corresponding_counter1 = cell_bin_counter1(ring_index, sector_index1); % 1D real value.
    catch
        continue;
    end
    cell_bin_index1{ring_index, sector_index1}=[cell_bin_index1{ring_index, sector_index1},label(ith_point)];
    cell_bin_counter1(ring_index, sector_index1) = cell_bin_counter1(ring_index, sector_index1) + 1; % increase count 1
    
    
    
    try
        corresponding_counter2 = cell_bin_counter2(ring_index, sector_index2); % 1D real value.
    catch
        continue;
    end
    cell_bin_index2{ring_index, sector_index2}=[cell_bin_index2{ring_index, sector_index2},label(ith_point)];
    cell_bin_counter2(ring_index, sector_index2) = cell_bin_counter2(ring_index, sector_index2) + 1; % increase count 1
    
    
    try
        corresponding_counter3 = cell_bin_counter3(ring_index, sector_index3); % 1D real value.
    catch
        continue;
    end
    cell_bin_index3{ring_index, sector_index3}=[cell_bin_index3{ring_index, sector_index3},label(ith_point)];
    cell_bin_counter3(ring_index, sector_index3) = cell_bin_counter3(ring_index, sector_index3) + 1; % increase count 1
    
end


%%%�Լ���
img = zeros(num_ring, num_sector);
img1 = zeros(num_ring, num_sector);
img2 = zeros(num_ring, num_sector);
img3 = zeros(num_ring, num_sector);
for ith_ring = 1:num_ring
    for ith_sector = 1:num_sector
        if((~isempty(cell_bin_index{ith_ring,ith_sector}))&(cell_bin_index{ith_ring,ith_sector}>1)&(cell_bin_index{ith_ring,ith_sector}<99))
            %取代sc_dealwith这个函数 少加一个循环
            img(ith_ring,ith_sector)=mode(cell_bin_index{ith_ring,ith_sector});
        else
            img(ith_ring,ith_sector)=0;
        end
        
        if((~isempty(cell_bin_index1{ith_ring,ith_sector}))&(cell_bin_index1{ith_ring,ith_sector}>1)&(cell_bin_index1{ith_ring,ith_sector}<99))
            img1(ith_ring,ith_sector)=mode(cell_bin_index1{ith_ring,ith_sector});
        else
            img1(ith_ring,ith_sector)=0;
        end
        
        if((~isempty(cell_bin_index2{ith_ring,ith_sector}))&(cell_bin_index2{ith_ring,ith_sector}>1)&(cell_bin_index2{ith_ring,ith_sector}<99))
            img2(ith_ring,ith_sector)=mode(cell_bin_index2{ith_ring,ith_sector});
        else
            img2(ith_ring,ith_sector)=0;
        end
        
        if((~isempty(cell_bin_index3{ith_ring,ith_sector}))&(cell_bin_index3{ith_ring,ith_sector}>1)&(cell_bin_index3{ith_ring,ith_sector}<99))
            img3(ith_ring,ith_sector)=mode(cell_bin_index3{ith_ring,ith_sector});
        else
            img3(ith_ring,ith_sector)=0;
        end
    end
end
cell_img={img,img1,img2,img3};
%%%
end

%% bin to image format (2D matrix) 
