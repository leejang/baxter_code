function [] = final_results_after_tracking()

    % name of new video to create
    new_video = '092016_0201_final.avi';
    new_hdf5 = '092016_0201.h5';

    % threshold for active points
    thr = 15;
    my_left_thr = 85;
    my_right_thr = 25;
    other_left_thr = 100;
    other_right_thr = 70;

    % add RGB package to display color
    addpath(genpath('rgb'));

    % window_file
    window_file = 'all_windows.txt';
    window_data = parse_input_window_file(window_file);
    assignin('base', 'window_data', window_data);

    % read csv files

% for scenario 2
    pan_fname = 'pan.csv';
    trivet_fname = 'trivet.csv';
    table_fname = 'table.csv';
    my_left_fname = 'my_left.csv';
    my_right_fname = 'my_right.csv';
    other_right_fname = 'other_right.csv';
    bk_my_left_fname = 'bk_my_left.csv';
    bk_my_right_fname = 'bk_my_right.csv';
    bk_other_right_fname = 'bk_other_right.csv';

    pan_m = csvread(pan_fname);
    trivet_m = csvread(trivet_fname);
    table_m = csvread(table_fname);
    my_left_m = csvread(my_left_fname);
    my_right_m = csvread(my_right_fname);
    other_right_m = csvread(other_right_fname);
    bk_my_left_m = csvread(bk_my_left_fname);
    bk_my_right_m = csvread(bk_my_right_fname);
    bk_other_right_m = csvread(bk_other_right_fname);

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
    % for scenario 2
    pan_start = pan_m(1,1);
    trivet_start = trivet_m(1,1);
    table_start = table_m(1,1);
    my_left_start = my_left_m(1,1);  
    my_right_start = my_right_m(1,1);  
    other_right_start = other_right_m(1,1);

    %disp(pan_start);
    %disp(table_mat_start);
    %disp(other_right_start);
%{
    beer_box_start = beer_box_m(1,1);
    oatmeal_start = oatmeal_m(1,1);
    butter_start = butter_m(1,1);
    coffee_start = coffee_m(1,1);
    my_left_start = my_left_m(1,1);  
    my_right_start = my_right_m(1,1);  
    other_left_start = other_left_m(1,1);
    other_right_start = other_right_m(1,1);
%}
    % loop over each frame
    for f = 1:num_of_frames 
        % show processing frame
        X = sprintf('processing... frame: %d',f);
        disp(X);
        % get each image frame
        img_path = window_data(f).img_path;
        img = imread(img_path);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % variable 1
        % my left
        %disp('Active points:');
        if f > my_left_start 
            if my_left_m(f - my_left_start, 3) > my_left_thr
                window = [my_left_m(f - my_left_start, 11) my_left_m(f - my_left_start, 12) ...
                          my_left_m(f - my_left_start, 6) my_left_m(f - my_left_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{1}, 'LineWidth', 7);
                raw_pos = [my_left_m(f - my_left_start, 4) my_left_m(f - my_left_start, 5)];
            end
        end

        % bk my left
        %disp('Active points:');
        if f <= my_left_start 
            if bk_my_left_m(my_left_start - f + 2, 3) > thr
                window = [bk_my_left_m(my_left_start - f + 2, 11) bk_my_left_m(my_left_start - f + 2, 12) ...
                          bk_my_left_m(my_left_start - f + 2, 6) bk_my_left_m(my_left_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{1}, 'LineWidth', 7);
                raw_pos = [bk_my_left_m(my_left_start - f + 2, 4) bk_my_left_m(my_left_start - f + 2, 5)];
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
                raw_pos = [raw_pos; my_right_m(f - my_right_start, 4) my_right_m(f - my_right_start, 5)];
            end
        end

        % bk my right
        %disp('Active points:');
        if f <= my_right_start 
            if bk_my_right_m(my_right_start - f + 2, 3) > my_right_thr
                window = [bk_my_right_m(my_right_start - f + 2, 11) bk_my_right_m(my_right_start - f + 2, 12) ...
                          bk_my_right_m(my_right_start - f + 2, 6) bk_my_right_m(my_right_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{2}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{2}, 'LineWidth', 7);
                raw_pos = [raw_pos; bk_my_right_m(my_right_start - f + 2, 11) bk_my_right_m(my_right_start - f + 2, 12)];
            end
        end

        % object: pan
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > pan_start
            if pan_m(f - pan_start, 3) > thr
                window = [pan_m(f - pan_start, 11) pan_m(f - pan_start, 12) ...
                          pan_m(f - pan_start, 6) pan_m(f - pan_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{1}, 'LineWidth', 7);
            end
        end

        % object: trivet
        %disp('Active points:');
        if f > trivet_start
            if trivet_m(f - trivet_start, 3) > thr
                window = [trivet_m(f - trivet_start, 11) trivet_m(f - trivet_start, 12) ...
                          trivet_m(f - trivet_start, 6) trivet_m(f - trivet_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{2}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{2}, 'LineWidth', 7);
            end
        end

        % object: table
        %disp('Active points:');
        if f > table_start
            if table_m(f - table_start, 3) > thr
                window = [table_m(f - table_start, 11) table_m(f - table_start, 12) ...
                          table_m(f - table_start, 6) table_m(f - table_start, 7)]; % turn into [x y width height] for rectint function
                %img = insertObjectAnnotation(img,'rectangle', window, objects{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{3}, 'LineWidth', 7);
            end
        end


        % other right
        %disp('Active points:');
        if f > other_right_start 
            if other_right_m(f - other_right_start, 3) > other_right_thr
                window = [other_right_m(f - other_right_start, 11) other_right_m(f - other_right_start, 12) ...
                          other_right_m(f - other_right_start, 6) other_right_m(f - other_right_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{4}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{4}, 'LineWidth', 7);
            end
        end


        % bk other right
        %disp('Active points:');
        if f <= other_right_start 
            if bk_other_right_m(other_right_start - f + 2, 3) > other_right_thr
                window = [bk_other_right_m(other_right_start - f + 2, 11) bk_other_right_m(other_right_start - f + 2, 12) ...
                          bk_other_right_m(other_right_start - f + 2, 6) bk_other_right_m(other_right_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{4}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{4}, 'LineWidth', 7);
            end
        end
%{
        % object: beer_box
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > beer_box_start
            if beer_box_m(f - beer_box_start, 3) > thr
                window = [beer_box_m(f - beer_box_start, 11) beer_box_m(f - beer_box_start, 12) ...
                          beer_box_m(f - beer_box_start, 6) beer_box_m(f - beer_box_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{6}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{6}, 'LineWidth', 7);
            end
        end
%}

%{
        % object: oatmeal
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > oatmeal_start
            if oatmeal_m(f - oatmeal_start, 3) > thr
                window = [oatmeal_m(f - oatmeal_start, 11) oatmeal_m(f - oatmeal_start, 12) ...
                          oatmeal_m(f - oatmeal_start, 6) oatmeal_m(f - oatmeal_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{3}, 'LineWidth', 7);
            end
        end
%}

%{
        % object: butter
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > butter_start
            if butter_m(f - butter_start, 3) > thr
                window = [butter_m(f - butter_start, 11) butter_m(f - butter_start, 12) ...
                          butter_m(f - butter_start, 6) butter_m(f - butter_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{1}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{1}, 'LineWidth', 7);
            end
        end
%}

%{
        % object: coffee
        %disp('Active points:');
        %disp(pan_m(f,3));
        if f > coffee_start
            if coffee_m(f - coffee_start, 3) > thr
                window = [coffee_m(f - coffee_start, 11) coffee_m(f - coffee_start, 12) ...
                          coffee_m(f - coffee_start, 6) coffee_m(f - coffee_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, objects{2}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', object_colors{2}, 'LineWidth', 7);
            end
        end

%}

%{
        % other left
        %disp('Active points:');
        if f > other_left_start 
            if other_left_m(f - other_left_start, 3) > other_left_thr
                window = [other_left_m(f - other_left_start, 11) other_left_m(f - other_left_start, 12) ...
                          other_left_m(f - other_left_start, 6) other_left_m(f - other_left_start, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{3}, 'LineWidth', 7);
            end
        end
%}

%{
        % bk other left
        %disp('Active points:');
        if f <= other_left_start 
            if bk_other_left_m(other_left_start - f + 2, 3) > other_left_thr
                window = [bk_other_left_m(other_left_start - f + 2, 11) bk_other_left_m(other_left_start - f + 2, 12) ...
                          bk_other_left_m(other_left_start - f + 2, 6) bk_other_left_m(other_left_start - f + 2, 7)]; % turn into [x y width height] for rectint function
                img = insertObjectAnnotation(img,'rectangle', window, hand_types{3}, 'TextBoxOpacity',0.4,'FontSize',25, 'Color', hand_colors{3}, 'LineWidth', 7);
            end
        end
%}

        data_disk(:,:,1,f) = raw_pos;
 
        % write frame with annotation
        writeVideo(outputVideo, img);
    end

    close(outputVideo);

    disp(data_disk);
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
        img_width = sscanf(fgetl(fid), '%d');
        img_height = sscanf(fgetl(fid), '%d');
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
