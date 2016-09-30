function [] = final_results_after_tracking()

    % name of new video to create
    new_video = '092016_0214_final.avi';
    new_hdf5 = '092016_0214.h5';

    % threshold for active points
% for scenario 2
    table_thr = 30;
    my_left_thr = 25;
    my_right_thr = 15;
    other_left_thr = Inf;
    %other_left_thr = 100;
    other_right_thr = 20;

    pan_thr = 15;
    trivet_thr = 15;
    beer_box_thr = Inf;
    oatmeal_thr = Inf;
    butter_thr = Inf;
    coffee_thr = Inf;

    % add RGB package to display color
    addpath(genpath('rgb'));

    % window_file
    window_file = 'all_windows.txt';
    window_data = parse_input_window_file(window_file);
    assignin('base', 'window_data', window_data);

% for scenario 2
    % read csv files
    pan_fname = 'pan.csv';
    trivet_fname = 'trivet.csv';
    table_fname = 'table.csv';
    my_left_fname = 'my_left.csv';
    my_right_fname = 'my_right.csv';
    other_right_fname = 'other_right.csv';
    bk_my_left_fname = 'bk_my_left.csv';
    bk_my_right_fname = 'bk_my_right.csv';
    bk_other_right_fname = 'bk_other_right.csv';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % reference
    table_m = csvread(table_fname);
    % variable 1
    my_left_m = csvread(my_left_fname);
    %bk_my_left_m = csvread(bk_my_left_fname);
    bk_my_left_m = zeros(size(table_m));
    % variable 2
    my_right_m = csvread(my_right_fname);
    bk_my_right_m = zeros(size(table_m));
    %bk_my_right_m = csvread(bk_my_right_fname);
    % variable 3
    other_left_m = zeros(size(table_m));
    bk_other_left_m = zeros(size(table_m));
    % variable 4
    other_right_m = csvread(other_right_fname);
    bk_other_right_m = csvread(bk_other_right_fname);
    % variable 5
    pan_m = csvread(pan_fname);
    % variable 6
    trivet_m = csvread(trivet_fname);
    % variable 7
    beer_box_m = zeros(size(table_m));
    % variable 8
    oatmeal_m = zeros(size(table_m));
    % variable 9
    butter_m = zeros(size(table_m));
    % variable 10
    coffee_m = zeros(size(table_m));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
    beer_box_fname = 'beer_box.csv';
    oatmeal_fname = 'oatmeal.csv';
    butter_fname = 'butter.csv';
    coffee_fname = 'coffee.csv';
    my_left_fname = 'my_left.csv';
    my_right_fname = 'my_right.csv';
    other_left_fname = 'other_left.csv';
    other_right_fname = 'other_right.csv';
    bk_my_right_fname = 'bk_my_right.csv';
    bk_other_left_fname = 'bk_other_left.csv';
    bk_other_right_fname = 'bk_other_right.csv';

    beer_box_m = csvread(beer_box_fname);
    oatmeal_m = csvread(oatmeal_fname);
    butter_m = csvread(butter_fname);
    coffee_m = csvread(coffee_fname);
    my_left_m = csvread(my_left_fname);
    my_right_m = csvread(my_right_fname);
    other_left_m = csvread(other_left_fname);
    other_right_m = csvread(other_right_fname);
    bk_my_right_m = csvread(bk_my_right_fname);
    bk_other_left_m = csvread(bk_other_left_fname);
    bk_other_right_m = csvread(bk_other_right_fname);
%}

    hand_types = {'my_left','my_right','your_left','your_right'};
    hand_colors = {'blue','yellow','red','green'};

    objects = {'pan', 'trivet', 'table', 'pan', 'beer_box', 'butter', 'oatmeal', 'coffee'};
    object_colors = {'magenta', 'cyan', 'black', 'white', rgb('Orange')*255, rgb('Pink')*255,  ...
                     rgb('Olive')*255, rgb('Tan')*255};

    % get number of frames
    [num_of_frames, bbx] = size(table_m);

    outputVideo = VideoWriter(new_video);
    outputVideo.FrameRate = 30;
    open(outputVideo)

    % first objects/hands detected frame
    table_start = table_m(1,1);
    my_left_start = my_left_m(1,1);  
    my_right_start = my_right_m(1,1);
    other_left_start = other_left_m(1,1);
    other_right_start = other_right_m(1,1);
    pan_start = pan_m(1,1);
    trivet_start = trivet_m(1,1);
    beer_box_start = beer_box_m(1,1);
    oatmeal_start = oatmeal_m(1,1);
    butter_start = butter_m(1,1);
    coffee_start = coffee_m(1,1);
    %disp(other_right_start);

    img_size = window_data(1).img_size;

    prev_table_pos = [NaN NaN];
    prev_my_left_pos = [NaN NaN];
    prev_my_right_pos = [NaN NaN];
    prev_ot_left_pos = [NaN NaN];
    prev_ot_right_pos = [NaN NaN];
    prev_pan_pos = [NaN NaN];
    prev_trivet_pos = [NaN NaN];
    prev_beer_box_pos = [NaN NaN];
    prev_butter_pos = [NaN NaN];
    prev_oatmeal_pos = [NaN NaN];
    prev_coffee_pos = [NaN NaN];

    %loop over each frame
    %num_of_frames = 125;

    for f = 1:num_of_frames 
        % show processing frame
        X = sprintf('processing... frame: %d',f);
        disp(X);
        % get each image frame
        img_path = window_data(f).img_path;
        img = imread(img_path);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % to be used as reference points
        % object: table
        %disp('Active points:');
        if f > table_start
            if table_m(f - table_start, 3) > table_thr
                window = [table_m(f - table_start, 11) table_m(f - table_start, 12) ...
                          table_m(f - table_start, 6) table_m(f - table_start, 7)]; % turn into [x y width height] for rectint function
                %img = insertObjectAnnotation(img,'rectangle', window, objects{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{3}, 'LineWidth', 7);
                table_pos = [table_m(f - table_start, 4) table_m(f - table_start, 5)];
                ref_pos = table_pos;
                prev_table_pos = table_pos;
            else
                table_pos = prev_table_pos;
                ref_pos = table_pos;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 1
        % my left
        %disp('Active points:');
        if f > my_left_start 
            if my_left_m(f - my_left_start, 3) > my_left_thr
                window = [my_left_m(f - my_left_start, 11) my_left_m(f - my_left_start, 12) ...
                          my_left_m(f - my_left_start, 6) my_left_m(f - my_left_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{1}, 'LineWidth', 7);
                raw_pos = [my_left_m(f - my_left_start, 4) my_left_m(f - my_left_start, 5)] - ref_pos;
                prev_my_left_pos = raw_pos;
            else
                raw_pos = prev_my_left_pos;
            end
        end

        % bk my left
        %disp('Active points:');
        if f <= my_left_start 
            if bk_my_left_m(my_left_start - f + 2, 3) > my_left_thr
                window = [bk_my_left_m(my_left_start - f + 2, 11) bk_my_left_m(my_left_start - f + 2, 12) ...
                          bk_my_left_m(my_left_start - f + 2, 6) bk_my_left_m(my_left_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{1}, 'LineWidth', 7);
                raw_pos = [bk_my_left_m(my_left_start - f + 2, 4) bk_my_left_m(my_left_start - f + 2, 5)] - ref_pos;
                prev_my_left_pos = raw_pos;
            else
                raw_pos = prev_my_left_pos;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 2
        % my right
        %disp('Active points:');
        if f > my_right_start 
            if my_right_m(f - my_right_start, 3) > my_right_thr
                window = [my_right_m(f - my_right_start, 11) my_right_m(f - my_right_start, 12) ...
                          my_right_m(f - my_right_start, 6) my_right_m(f - my_right_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{2}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{2}, 'LineWidth', 7);
                raw_pos = [raw_pos; (my_right_m(f - my_right_start, 4) - ref_pos(1)) (my_right_m(f - my_right_start, 5) - ref_pos(2))];
                prev_my_right_pos = [(my_right_m(f - my_right_start, 4) - ref_pos(1)) (my_right_m(f - my_right_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_my_right_pos];
            end
        end

        % bk my right
        %disp('Active points:');
        if f <= my_right_start 
            if bk_my_right_m(my_right_start - f + 2, 3) > my_right_thr
                window = [bk_my_right_m(my_right_start - f + 2, 11) bk_my_right_m(my_right_start - f + 2, 12) ...
                          bk_my_right_m(my_right_start - f + 2, 6) bk_my_right_m(my_right_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{2}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{2}, 'LineWidth', 7);
                raw_pos = [raw_pos; (bk_my_right_m(my_right_start - f + 2, 4) - ref_pos(1)) (bk_my_right_m(my_right_start - f + 2, 5) - ref_pos(2))];
                prev_my_right_pos = [(bk_my_right_m(my_right_start - f + 2, 4) - ref_pos(1)) (bk_my_right_m(my_right_start - f + 2, 5) - ref_pos(2))]; 
            else
                raw_pos = [raw_pos; prev_my_right_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 3
        % other left
        %disp('Active points:');
        if f > other_left_start 
            if other_left_m(f - other_left_start, 3) > other_left_thr
                window = [other_left_m(f - other_left_start, 11) other_left_m(f - other_left_start, 12) ...
                          other_left_m(f - other_left_start, 6) other_left_m(f - other_left_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{3}, 'LineWidth', 7);
                raw_pos = [raw_pos; (other_left_m(f - other_left_start, 4) - ref_pos(1)) (other_left_m(f - other_left_start, 5) - ref_pos(2))];
                prev_ot_left_pos = [(other_left_m(f - other_left_start, 4) - ref_pos(1)) (other_left_m(f - other_left_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_ot_left_pos];
            end
        end

        % bk other left
        %disp('Active points:');
        if f <= other_left_start 
            if bk_other_left_m(other_left_start - f + 2, 3) > other_left_thr
                window = [bk_other_left_m(other_left_start - f + 2, 11) bk_other_left_m(other_left_start - f + 2, 12) ...
                          bk_other_left_m(other_left_start - f + 2, 6) bk_other_left_m(other_left_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{3}, 'LineWidth', 7);
                raw_pos = [raw_pos; (bk_other_left_m(other_left_start - f + 2, 4) - ref_pos(1)) (bk_other_left_m(other_left_start - f + 2, 5) - ref_pos(2))];
                prev_ot_left_pos = [(bk_other_left_m(other_left_start - f + 2, 4) - ref_pos(1)) (bk_other_left_m(other_left_start - f + 2, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_ot_left_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 4
        % other right
        %disp('Active points:');
        if f > other_right_start 
            if other_right_m(f - other_right_start, 3) > other_right_thr
                window = [other_right_m(f - other_right_start, 11) other_right_m(f - other_right_start, 12) ...
                          other_right_m(f - other_right_start, 6) other_right_m(f - other_right_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{4}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{4}, 'LineWidth', 7);
                raw_pos = [raw_pos; (other_right_m(f - other_right_start, 4) - ref_pos(1)) (other_right_m(f - other_right_start, 5) - ref_pos(2))];
                prev_ot_right_pos = [(other_right_m(f - other_right_start, 4) - ref_pos(1)) (other_right_m(f - other_right_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_ot_right_pos];
            end
        end

        % bk other right
        %disp('Active points:');
        if f <= other_right_start 
            if bk_other_right_m(other_right_start - f + 2, 3) > other_right_thr
                window = [bk_other_right_m(other_right_start - f + 2, 11) bk_other_right_m(other_right_start - f + 2, 12) ...
                          bk_other_right_m(other_right_start - f + 2, 6) bk_other_right_m(other_right_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{4}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{4}, 'LineWidth', 7);
                raw_pos = [raw_pos; (bk_other_right_m(other_right_start - f + 2, 4) - ref_pos(1)) (bk_other_right_m(other_right_start - f + 2, 5) - ref_pos(2))];
                prev_ot_right_pos = [(bk_other_right_m(other_right_start - f + 2, 4) - ref_pos(1)) (bk_other_right_m(other_right_start - f + 2, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_ot_right_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 5
        % object: pan
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > pan_start
            if pan_m(f - pan_start, 3) > pan_thr
                window = [pan_m(f - pan_start, 11) pan_m(f - pan_start, 12) ...
                          pan_m(f - pan_start, 6) pan_m(f - pan_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{1}, 'LineWidth', 7);
                raw_pos = [raw_pos; (pan_m(f - pan_start, 4) - ref_pos(1)) (pan_m(f - pan_start, 5) - ref_pos(2))];
                prev_pan_pos = [(pan_m(f - pan_start, 4) - ref_pos(1)) (pan_m(f - pan_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_pan_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 6
        % object: trivet
        %disp('Active points:');
        if f > trivet_start
            if trivet_m(f - trivet_start, 3) > trivet_thr
                window = [trivet_m(f - trivet_start, 11) trivet_m(f - trivet_start, 12) ...
                          trivet_m(f - trivet_start, 6) trivet_m(f - trivet_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{2}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{2}, 'LineWidth', 7);
                raw_pos = [raw_pos; (trivet_m(f - trivet_start, 4) - ref_pos(1)) (trivet_m(f - trivet_start, 5) - ref_pos(2))];
                prev_trivet_pos = [(trivet_m(f - trivet_start, 4) - ref_pos(1)) (trivet_m(f - trivet_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_trivet_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 7
        % object: beer_box
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > beer_box_start
            if beer_box_m(f - beer_box_start, 3) > beer_box_thr
                window = [beer_box_m(f - beer_box_start, 11) beer_box_m(f - beer_box_start, 12) ...
                          beer_box_m(f - beer_box_start, 6) beer_box_m(f - beer_box_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{6}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{6}, 'LineWidth', 7);
                raw_pos = [raw_pos; (beer_box_m(f - beer_box_start, 4) - ref_pos(1)) (beer_box_m(f - beer_box_start, 5) - ref_pos(2))]; 
                prev_beer_box_pos = [(beer_box_m(f - beer_box_start, 4) - ref_pos(1)) (beer_box_m(f - beer_box_start, 5) - ref_pos(2))]; 
            else
                raw_pos = [raw_pos; prev_beer_box_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 8
        % object: oatmeal
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > oatmeal_start
            if oatmeal_m(f - oatmeal_start, 3) > oatmeal_thr
                window = [oatmeal_m(f - oatmeal_start, 11) oatmeal_m(f - oatmeal_start, 12) ...
                          oatmeal_m(f - oatmeal_start, 6) oatmeal_m(f - oatmeal_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{3}, 'LineWidth', 7);
                raw_pos = [raw_pos; (oatmeal_m(f - oatmeal_start, 4) - ref_pos(1)) (oatmeal_m(f - oatmeal_start, 5) - ref_pos(2))];
                prev_oatmeal_pos = [(oatmeal_m(f - oatmeal_start, 4) - ref_pos(1)) (oatmeal_m(f - oatmeal_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_oatmeal_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 9
        % object: butter
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > butter_start
            if butter_m(f - butter_start, 3) > butter_thr
                window = [butter_m(f - butter_start, 11) butter_m(f - butter_start, 12) ...
                          butter_m(f - butter_start, 6) butter_m(f - butter_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{1}, 'LineWidth', 7);
                raw_pos = [raw_pos; (butter_m(f - butter_start, 4) - ref_pos(1)) (butter_m(f - butter_start, 5) - ref_pos(2))];
                prev_butter_pos = [(butter_m(f - butter_start, 4) - ref_pos(1)) (butter_m(f - butter_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_butter_pos];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 10
        % object: coffee
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > coffee_start
            if coffee_m(f - coffee_start, 3) > coffee_thr
                window = [coffee_m(f - coffee_start, 11) coffee_m(f - coffee_start, 12) ...
                          coffee_m(f - coffee_start, 6) coffee_m(f - coffee_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{2}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{2}, 'LineWidth', 7);
                raw_pos = [raw_pos; (coffee_m(f - coffee_start, 4) - ref_pos(1)) (coffee_m(f - coffee_start, 5) - ref_pos(2))];
                prev_coffee_pos = [(coffee_m(f - coffee_start, 4) - ref_pos(1)) (coffee_m(f - coffee_start, 5) - ref_pos(2))];
            else
                raw_pos = [raw_pos; prev_coffee_pos];
            end
        end

        data_disk(:,:,1,f) = raw_pos;
        label_disk(1:10,f) = raw_pos(:,1);
        label_disk(11:20,f) = raw_pos(:,2);

        %disp(ref_pos);
        % write frame with annotation
        writeVideo(outputVideo, img);
    end

    close(outputVideo);

    %disp(data_disk);
    %disp(img_size);

    % to save hdf5 file
    chunksz = (num_of_frames - 35);
    created_flag = false;
    totalct = 0;
    for batchno = 1:(num_of_frames - 35)/chunksz
        fprintf('batch no. %d\n', batchno);
        last_read = (batchno-1)*chunksz;

        % to simulate maximum data to be held in memory before dumping to hdf5 file
        %batchdata = data_disk(:,:,1,last_read+1:last_read+chunksz);
        %batchlabs = label_disk(:,:,1,last_read+1:last_read+chunksz);
        %batchdata = data_disk(:,:,1,last_read+1:last_read+chunksz);

        % concatenate 10 frames
        for frame = 1:(num_of_frames - 35)
            for idx = 0:9
                batchdata(1+idx*10:10+idx*10,:,1,frame) = data_disk(:,:,1,frame + idx);
            end
        end

        batchlabs = label_disk(:,last_read+36:last_read+chunksz+35);
    
        % normalize
        batchdata(:,1,1,:) = (batchdata(:,1,1,:) + img_size(1)) / (img_size(1) + img_size(1)); % 1280 * 2 (x)
        batchdata(:,2,1,:) = (batchdata(:,2,1,:) + img_size(2)) / (img_size(2) + img_size(2)); % 720 * 2 (y)
        batchlabs(1:10,:) = (batchlabs(1:10,:) + img_size(1)) / (img_size(1) + img_size(1)); % 1280 * 2 (x)
        batchlabs(11:20,:) = (batchlabs(11:20,:) + img_size(2)) / (img_size(2) + img_size(2)); % 720 * 2 (y)

        % to set all NaN to zero
        batchdata(isnan(batchdata)) = 0.0;        
        batchlabs(isnan(batchlabs)) = 0.0;        

        %disp(batchdata);

        % store to hdf5
        startloc = struct('dat',[1,1,1,totalct + 1], 'lab', [1,totalct + 1]);
        curr_dat_sz = store2hdf5(new_hdf5, batchdata, batchlabs, ~created_flag, startloc, chunksz); 
        created_flag = true;% flag set so that file is created only once
        totalct = curr_dat_sz(end);% updated dataset size (#samples)
    end

    % display structure of the stored HDF5 file
    h5disp(new_hdf5);

    disp('finally done!!');
end

function window_data = parse_input_window_file(input_file)

    window_data = struct();
    fi = 1;

    fid = fopen(input_file);
    tline = fgetl(fid);

    total_num_windows = 0;
    while ischar(tline)
        id = sscanf(tline, '%s %f');
        window_data(fi).frame_id = id(2);
        img_path = sscanf(fgetl(fid), '%s');
        window_data(fi).img_path = img_path;
        channels = sscanf(fgetl(fid), '%d');
        img_height = sscanf(fgetl(fid), '%d');
        img_width = sscanf(fgetl(fid), '%d');
        window_data(fi).img_size = [img_width img_height];
        num_windows = sscanf(fgetl(fid), '%d');
        total_num_windows = total_num_windows + num_windows;

        % testscan: reads file data using the formant N times
        [win, pos] = textscan(fid, '%f %f %f %f %f %f', num_windows);
        fseek(fid, pos+1, 'bof');
        window_data(fi).windows = [win{2} win{3}, win{4}, win{5}, win{6}];
            
        fi = fi + 1;
        tline = fgetl(fid);
    end

    fclose(fid);
end
