function [] = visualize_prediction_results()

    % name of new video to create
    cur_video = '092016_0112_current.avi';
    %pre_video = '092016_0112_new_prediction.avi';
    pre_video = '092016_0112_current_with_prediction.avi';
    
    % read hdf5 file (ground truth)
    gt_data = hdf5read('092016_0112.h5','data');
    gt_label = hdf5read('092016_0112.h5','label');

    % add RGB package
    addpath(genpath('rgb'));

    % prediction_file
    prediction_file = 'prediction.txt';
    prediction_data = parse_prediction_file(prediction_file);
    assignin('base', 'prediction_data', prediction_data);

    % window_file
    window_file = 'all_windows.txt';
    window_data = parse_input_window_file(window_file);
    assignin('base', 'window_data', window_data);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % reference
    table_fname = 'table.csv';
    table_m = csvread(table_fname);

    % get number of frames
    [num_of_frames, bbx] = size(table_m);

    curVideo = VideoWriter(cur_video);
    curVideo.FrameRate = 30;
    open(curVideo)

    preVideo = VideoWriter(pre_video);
    preVideo.FrameRate = 30;
    open(preVideo)

    hand_types = {'my_left','my_right','your_left','your_right'};
    hand_colors = {rgb('Blue')*255, rgb('Yellow')*255, rgb('Red')*255, rgb('Green')*255};

    objects = {'pan', 'trivet', 'table', 'beer_box', 'oatmeal', 'butter', 'coffee'};
    object_colors = {rgb('Magenta')*255, rgb('Cyan')*255, rgb('Black')*255, rgb('Olive')*255, rgb('Orange')*255, rgb('Pink')*255, rgb('Tan')*255};

    % first objects/hands detected frame
    table_thr = 30;
    table_start = table_m(1,1);
    img_size = window_data(1).img_size;
    prev_table_pos = [NaN NaN];

    for f = 1:num_of_frames-35
        % show processing frame
        X = sprintf('processing... frame: %d',f);
        disp(X);

        % get each image frame
        img_path = window_data(f).img_path;
        img = imread(img_path);

        % get each image frame
        f_img_path = window_data(f+35).img_path;
        f_img = imread(f_img_path);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % to be used as reference points
        % object: table
        %disp('Active points:');
        if (f+35) > table_start
            if table_m((f+35) - table_start, 3) > table_thr
                table_pos = [table_m((f+35) - table_start, 4) table_m((f+35) - table_start, 5)];
                ref_pos = table_pos;
                prev_table_pos = table_pos;
            else
                table_pos = prev_table_pos;
                ref_pos = table_pos;
            end
        end

        % my_left
        my_left = [round((prediction_data(f,1) - 0.5)*2560) round((prediction_data(f,11) - 0.5)*1440)] + round(ref_pos);
        
        % my_right
        my_right = [round((prediction_data(f,2) - 0.5)*2560) round((prediction_data(f,12) - 0.5)*1440)] + round(ref_pos);

        % your_left
        your_left = [round((prediction_data(f,3) - 0.5)*2560) round((prediction_data(f,13) - 0.5)*1440)] + round(ref_pos);

        % your_right
        your_right = [round((prediction_data(f,4) - 0.5)*2560) round((prediction_data(f,14) - 0.5)*1440)] + round(ref_pos);
%{
        % pan
        pan = [round((prediction_data(f,5) - 0.5)*2560) round((prediction_data(f,15) - 0.5)*1440)] + round(ref_pos);
        %f_img = insertObjectAnnotation(img,'circle', [pan 50], objects{1}, 'LineWidth', 7, 'Color',object_colors{1},'TextColor','black');

        % trivet
        trivet = [round((prediction_data(f,6) - 0.5)*2560) round((prediction_data(f,16) - 0.5)*1440)] + round(ref_pos);
        %f_img = insertObjectAnnotation(img,'circle', [trivet 50], objects{2}, 'LineWidth', 7, 'Color',object_colors{2},'TextColor','black');
%}
        % beer box
        beer = [round((prediction_data(f,7) - 0.5)*2560) round((prediction_data(f,17) - 0.5)*1440)] + round(ref_pos);
        % oatmeal
        oatmeal = [round((prediction_data(f,8) - 0.5)*2560) round((prediction_data(f,18) - 0.5)*1440)] + round(ref_pos);
        % butter
        butter = [round((prediction_data(f,9) - 0.5)*2560) round((prediction_data(f,19) - 0.5)*1440)] + round(ref_pos);
        % coffee
        coffee = [round((prediction_data(f,10) - 0.5)*2560) round((prediction_data(f,20) - 0.5)*1440)] + round(ref_pos);

%         predic_pos = [my_left 50; my_right 50; your_right 50; pan 50; trivet 50];
%         predic_labels = {hand_types{1} hand_types{2} hand_types{4} objects{1} objects{2}};
        predic_pos = [my_left 50; my_right 50; your_left 50; your_right 50; beer 50; oatmeal 50; butter 50; coffee 50];
        predic_labels = {hand_types{1} hand_types{2} hand_types{3} hand_types{4} objects{4} objects{5} objects{6} objects{7}};

        % ground truth
        % gt_my_left
        gt_my_left = [round((gt_label(1,f) - 0.5)*2560) round((gt_label(11,f) - 0.5)*1440)] + round(ref_pos);
 
        % gt_my_right
        gt_my_right = [round((gt_label(2,f) - 0.5)*2560) round((gt_label(12,f) - 0.5)*1440)] + round(ref_pos);

        % gt_your_left
        gt_your_left = [round((gt_label(3,f) - 0.5)*2560) round((gt_label(13,f) - 0.5)*1440)] + round(ref_pos);

        % gt_your_right
        gt_your_right = [round((gt_label(4,f) - 0.5)*2560) round((gt_label(14,f) - 0.5)*1440)] + round(ref_pos);
%{
        % gt_pan
        gt_pan = [round((gt_label(5,f) - 0.5)*2560) round((gt_label(15,f) - 0.5)*1440)] + round(ref_pos);

        % gt_trivet
        gt_trivet = [round((gt_label(6,f) - 0.5)*2560) round((gt_label(16,f) - 0.5)*1440)] + round(ref_pos);
%}
        % beer box
        gt_beer = [round((gt_label(7,f) - 0.5)*2560) round((gt_label(17,f) - 0.5)*1440)] + round(ref_pos);
        % oatmeal
        gt_oatmeal = [round((gt_label(8,f) - 0.5)*2560) round((gt_label(18,f) - 0.5)*1440)] + round(ref_pos);
        % butter
        gt_butter = [round((gt_label(9,f) - 0.5)*2560) round((gt_label(19,f) - 0.5)*1440)] + round(ref_pos);
        % coffee
        gt_coffee = [round((gt_label(10,f) - 0.5)*2560) round((gt_label(20,f) - 0.5)*1440)] + round(ref_pos);

%        gt_pos = [gt_my_left 50; gt_my_right 50; gt_your_right 50; gt_pan 50; gt_trivet 50];
%        gt_labels = {hand_types{1} hand_types{2} hand_types{4} objects{1} objects{2}};

        gt_pos = [gt_my_left 50; gt_my_right 50; gt_your_left 50; gt_your_right 50; gt_beer 50; gt_oatmeal 50; gt_butter 50; gt_coffee 50];
        gt_labels = {hand_types{1} hand_types{2} hand_types{3} hand_types{4} objects{4} objects{5} objects{6} objects{7}};

        predic_pos(isnan(predic_pos)) = 0;
        gt_pos(isnan(gt_pos)) = 0;

%{
        f_img = insertObjectAnnotation(f_img,'circle', [predic_pos; gt_pos], [predic_labels gt_labels], 'LineWidth', 7, ...
                                       'Color', [hand_colors{1}; hand_colors{2}; hand_colors{3}; hand_colors{4}; ...
                                                 object_colors{4}; object_colors{5}; object_colors{6}; object_colors{7}; ...
                                                 rgb('White')*255; rgb('White')*255; rgb('White')*255; rgb('White')*255; ...
                                                 rgb('White')*255; rgb('White')*255; rgb('White')*255; rgb('White')*255],'TextColor',rgb('Black')*255);
%}
%{
        % for scenario 2
        f_img = insertObjectAnnotation(f_img,'circle', [predic_pos; gt_pos], [predic_labels gt_labels], 'LineWidth', 7, ...
                                       'Color', [hand_colors{1}; hand_colors{2}; hand_colors{4}; ...
                                                 object_colors{1}; object_colors{2}; ...
                                                 rgb('White')*255; rgb('White')*255; ...
                                                 rgb('White')*255; rgb('White')*255; rgb('White')*255;],'TextColor',rgb('Black')*255);
%}
        % current frame with prediction (no ground truth) - scenario 1
        f_img = insertObjectAnnotation(img,'circle', [predic_pos], [predic_labels], 'LineWidth', 7, ...
                                       'Color', [hand_colors{1}; hand_colors{2}; hand_colors{3}; hand_colors{4}; ...
                                                 object_colors{4}; object_colors{5}; object_colors{6}; object_colors{7}], ...
                                                 'TextColor',rgb('Black')*255);

	writeVideo(curVideo, img);
	writeVideo(preVideo, f_img);
    end

    close(curVideo);
    close(preVideo);

    disp('visualization done!!');

end


function prediction_data = parse_prediction_file(prediction_file)
    % read in CNN hand probs for each 
    disp(['Parsing ' prediction_file]);
    fileID = fopen(prediction_file);
    data = textscan(fileID,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    fclose(fileID);
    prediction_data = [data{2} data{3} data{4} data{5} data{6} data{7} data{8} data{9} data{10} data{11} ...
                       data{12} data{13} data{14} data{15} data{16} data{17} data{18} data{19} data{20} data{21}];
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
