function [dist] = DistanceBtnScanContexts2(sc1,sc2)

%用的是每个scan1和scan进行逐列比较
%每个scan1的得分是instance比较匹配上的/比较的次数

num_sectors = size(sc1, 2);

% repeate to move 1 columns 
shift_area=-5;
shift_count=2*abs(shift_area)+1;
sim_for_each_cols = zeros(1, shift_count);
sc1=circshift(sc1,shift_area, 2);

for i = 1:shift_count
    %% Shift 
    one_step = 1; % const 
    sc1 = circshift(sc1, one_step, 2); % 2 means columne shift
    
    %% compare
    score_sum=0;
    count=0;
    correct=0;
    for j = 1:num_sectors 
        col_j_1 = sc1(:,j);
        col_j_2 = sc2(:,j);
        score=0;
        for k=1:size(col_j_1,1)
            if((col_j_1(k)<=1|col_j_1(k)>=99)&&(col_j_2(k)<=1|col_j_2(k)>=99))
                continue
            else
                count=count+1;
                if((col_j_1(k)==col_j_2(k)))
                    correct=correct+1;
                end            
            end
        end
        
        % calc sim
    end
    if(count==0)
        score=0;
    else
        score=correct/count;
    end
    
    % devided by num_col_engaged: So, even if there are many columns that are excluded from the calculation, we can get high scores if other columns are well fit.
    sim_for_each_cols(i) = score;
    
end

dist = 1-max(sim_for_each_cols);

end

