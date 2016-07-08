function [] = print_bounding_boxes(cnn_output_file, input_window_file)
    
    % parse CNN output file (hands probabilities for each window)
    all_hand_probs = parse_prob_file(cnn_output_file);
    window_data = parse_input_window_file(input_window_file);

    assignin('base', 'window_data', window_data);
    assignin('base', 'all_hand_probs', all_hand_probs);

    faceDetector = vision.CascadeObjectDetector;
    
    % bounding box data
    text = {'my left','my right','your left','your right'};
    colors = {'blue','yellow','red','green'};
    label_str = {'my left', 'my right', 'other left', 'other right'};
    
    % for a recall of 0.7
    threshold = [0.90 0.90 0.90 0.90]; % my left, my right, your left, your right
    
    nms_t = [0.2500 0.3000 0.1500 0.2000];
  
    pi2 = 1; % index to get the right probability for each window

    outputVideo = VideoWriter('video_5_hands.avi');
    outputVideo.FrameRate = 30;
    open(outputVideo)
    
    % loop over each frame
    for f = 1:611

        img_path = window_data(f).img_path;
        img = imread(img_path);

        face_bboxes = step(faceDetector, img);
        %evaluate each hand seperately
        %1 = ml, 2 = mr, 3 =  yl, 4 = yr
        for h = 1:4
                        
            disp(['Hand: ' num2str(h) ' | Frame: ' num2str(f)]);

            % get windows for frame
            windows = window_data(f).windows;
            probs = all_hand_probs(pi2:pi2 + size(windows, 1) - 1, h);
            windows = [windows(:, 2:5) probs];

            % subsample windows
            windows = windows(1:2000, :);


            % get all windows with prob > threshold and do non-maxium
            % surpression
            windows_nms = windows;
            windows_nms(windows_nms(:,5) < threshold(h), :) = [];
            windows_nms = nmsIoU(windows_nms, nms_t(h)); % windows = nx5 with each row = x1 y1 x2 y2 prob

            % block windows that are on faces
            if ~isempty(face_bboxes) && ~isempty(windows_nms)
                face_areas = repmat([face_bboxes(:,3) .* face_bboxes(:,4)]', [size(windows_nms, 1), 1]);
                window_nms_tmp = [windows_nms(:,1) windows_nms(:,2) windows_nms(:,3)-windows_nms(:,1)+1 windows_nms(:,4)-windows_nms(:,2)+1];
                overlaps = rectint(window_nms_tmp, face_bboxes) ./ face_areas;
                kill = overlaps > 0.1;
                if size(overlaps, 2) == 1
                    kill = find(overlaps);
                else
                    kill = find(any(overlaps')');
                end
                windows_nms(kill, :) = [];
            end

            if size(windows_nms, 1) > 0

                % format window and get area of the window
                for s = 1%:size(windows_nms, 1)
                    window = windows_nms(s, 1:4);
                    window = [window(1) window(2) window(3)-window(1)+1 window(4)-window(2)+1]; % turn into [x y width height] for rectint function

                    % img = insertShape(img, 'Rectangle', window, 'Color', colors{h}, 'LineWidth', 7);
                    img = insertObjectAnnotation(img,'rectangle', window, [label_str{h} ' | ' sprintf('%.3f', windows_nms(s,5))], 'TextBoxOpacity',0.4,'FontSize',25, 'Color', colors{h}, 'LineWidth', 7);

                    if ~isempty(face_bboxes)
                        img = insertObjectAnnotation(img, 'rectangle', face_bboxes(1,:), 'Face', 'Color', [0 0 0]);
                    end
                end
            end

        end % for h = 1:4
 
        pi2 = pi2 + size(windows, 1);
        writeVideo(outputVideo, img);

    end % for loop over each frame

    close(outputVideo);

    disp('done!')
end


function hand_probs = parse_prob_file(prob_file)
    % read in CNN hand probs for each window
    disp(['Parsing ' prob_file]);
    fileID = fopen(prob_file);
    data = textscan(fileID,'%f %f %f %f %f %f');
    fclose(fileID);
    hand_probs = [data{3} data{4} data{5} data{6}];
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

function [top] = nmsIoU(boxes, overlap)

    x1 = boxes(:,1);
    y1 = boxes(:,2);
    x2 = boxes(:,3);
    y2 = boxes(:,4);
    s = boxes(:,end);

    area = (x2-x1+1) .* (y2-y1+1);
    [~, I] = sort(s);

    pick = s*0;
    counter = 1;
    while ~isempty(I)

      last = length(I);
      i = I(last);  
      pick(counter) = i;
      counter = counter + 1;

      xx1 = max(x1(i), x1(I(1:last-1)));
      yy1 = max(y1(i), y1(I(1:last-1)));
      xx2 = min(x2(i), x2(I(1:last-1)));
      yy2 = min(y2(i), y2(I(1:last-1)));

      w = max(0.0, xx2-xx1+1);
      h = max(0.0, yy2-yy1+1);
      int = w.*h;
      o = int ./ (area(I(1:last-1)) + area(i) - int);

      I([last; find(o>overlap)]) = [];
    end

    pick = pick(1:(counter-1));
    top = boxes(pick,:);
end
