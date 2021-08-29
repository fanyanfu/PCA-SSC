clear; clc;
tic
addpath(genpath('src'));
addpath(genpath('data'));
count_detectloop=0;
count_true_loop=0;
log_result=[];
log_result_true=[];

if(1)
    %% data preparation
    global data_path;
    % your directory should contain files like this
    % - 00
    %   l- 00.csv (gt pose)
    %   l- velodyne
    %      l- <0xxxx.bin>
    %%data_path = '/media/gskim/Data/KITTI odo/data_odometry_velodyne/dataset/sequences/00/';
    data_path ='/home/vision/deep_learning/lidar-bonnetal-master/train/tasks/semantic/dataset/sequences/08/';
    
    
    down_shape = [40, 120];
    skip_data_frame = 1;
    [data_scancontexts, data_ringkeys, data_poses] = loadData(down_shape, skip_data_frame);
end

figure(101); clf;
% plot(data_poses(:,1), data_poses(:,2));


%% main - global recognizer
revisit_criteria = 5; % in meter (recommend test for 5, 10, 20 meters)
keyframe_gap = 1; % for_fast_eval (if 1, no skip)

global num_candidates; num_candidates = 50;
% global num_node_enough_apart; num_node_enough_apart = 50; 

% policy (top N)
num_top_n = 25;
top_n = linspace(1, num_top_n, num_top_n);

% Entropy thresholds 
middle_thres = 0.01;
thresholds1 = linspace(0, middle_thres, 50); 
thresholds2 = linspace(middle_thres, 1, 50);
thresholds = [thresholds1, thresholds2];
num_thresholds = length(thresholds);

% Main variables to store the result for drawing PR curve 
num_hits = zeros(num_top_n, num_thresholds); 
num_false_alarms = zeros(num_top_n, num_thresholds); 
num_correct_rejections = zeros(num_top_n, num_thresholds); 
num_misses = zeros(num_top_n, num_thresholds);

% main 
loop_log = [];

exp_poses = [];
exp_ringkeys = [];
exp_scancontexts = {};

num_queries = length(data_poses);
for query_idx = 1:num_queries - 1
        
    % save to (online) DB
    query_sc = data_scancontexts{query_idx};
    query_rk = data_ringkeys(query_idx, :);
    query_pose = data_poses(query_idx,:);

    exp_scancontexts{end+1} = query_sc;
    exp_poses = [exp_poses; query_pose];
    exp_ringkeys = [exp_ringkeys; query_rk];
    
    if(rem(query_idx, keyframe_gap) ~= 0)
       continue;
    end

    if( length(exp_scancontexts) < num_candidates )
       continue;
    end

    tree = createns(exp_ringkeys(1:end-(num_candidates-1), :), 'NSMethod', 'kdtree'); % Create object to use in k-nearest neighbor search

    % revisitness 
    [revisitness, how_far_apart] = isRevisitGlobalLoc(query_pose, exp_poses(1:end-(num_candidates-1), :), revisit_criteria);
% % %     disp([revisitness, how_far_apart])
    
    % find candidates 
    candidates = knnsearch(tree, query_rk, 'K', num_candidates); 
    
    % find the nearest (top 1) via pairwise comparison
    nearest_idx = 0;
    min_dist = inf; % initialization 
    for ith_candidate = 1:length(candidates)
        candidate_node_idx = candidates(ith_candidate);
        candidate_img = exp_scancontexts{candidate_node_idx};
        candidate_img=candidate_img{1};
        
        %%gai
        query_sc1=query_sc{1};
        query_sc2=query_sc{2};
        query_sc3=query_sc{3};
        query_sc4=query_sc{4};
        dist21 = DistanceBtnScanContexts2(candidate_img, query_sc1);
        dist22 = DistanceBtnScanContexts2(candidate_img, query_sc2);
        dist23 = DistanceBtnScanContexts2(candidate_img, query_sc3);
        dist24 = DistanceBtnScanContexts2(candidate_img, query_sc4);
        distance_to_query=min([dist21,dist22,dist23,dist24]);
        %%end
        
        %distance_to_query = sc_dist(query_sc, candidate_img);
        
        if( distance_to_query < min_dist)
            nearest_idx = candidate_node_idx;
            min_dist = distance_to_query;
            %%%second加上
            %%%second加上
        end
    end
    hold on;
    plot(data_poses(query_idx,1),data_poses(query_idx,2),'g.');
    
    if min_dist<=0.3%%修改为0.4看一下效果
        count_detectloop=count_detectloop+1;
        log_result=[log_result;nearest_idx,query_idx];
        
        hold on;
        plot(data_poses(query_idx,1),data_poses(query_idx,2),'r.');
        
        if(norm(data_poses(nearest_idx)-data_poses(query_idx),2)<5)
            count_true_loop=count_true_loop+1;
            log_result_true=[log_result_true;nearest_idx,query_idx];
        end
    end
    
    % prcurve analysis
    
% % %     if query_idx==500
% % %         display('debug');
% % %     end
    for topk = 1:num_top_n
        for thres_idx = 1:num_thresholds
            threshold = thresholds(thres_idx);
            
            reject = 0;
            if( min_dist > threshold)
                reject = 1; 
            end

            if(reject == 1) 
                if(revisitness == 0)
                    % TN: Correct Rejection
                    num_correct_rejections(topk, thres_idx) = num_correct_rejections(topk, thres_idx) + 1;
                else            
                    % FN: MISS
                    num_misses(topk, thres_idx) = num_misses(topk, thres_idx) + 1; 
                end
            else
                % if under the theshold, it is considered seen.
                % and then check the correctness
                if( dist_btn_pose(query_pose, exp_poses(nearest_idx, :)) < revisit_criteria)
                    % TP: Hit
                    num_hits(topk, thres_idx) = num_hits(topk, thres_idx) + 1;
                else
                    % FP: False Alarm 
                    num_false_alarms(topk, thres_idx) = num_false_alarms(topk, thres_idx) + 1;            
                end
            end
            
        end
    end

% % %     if( rem(query_idx, 100) == 0)
% % %         disp( strcat(num2str(query_idx/num_queries * 100), ' % processed') );
% % %     end
    
end
toc


%% save the log 
savePath = strcat("pr_result/within ", num2str(revisit_criteria), "m/");
if((~7==exist(savePath,'dir')))
    mkdir(savePath);
end
save(strcat(savePath, 'nCorrectRejections.mat'), 'num_correct_rejections');
save(strcat(savePath, 'nMisses.mat'), 'num_misses');
save(strcat(savePath, 'nHits.mat'), 'num_hits');
save(strcat(savePath, 'nFalseAlarms.mat'), 'num_false_alarms');







