% This function is used for finding
% the first frame of detected objects for tracking
function [] = do_tracking()

    % for mapping object id to its name
    % do not need now..
    %obj_map = road_obj_map();

    % raw detection file
    raw_detection_results = 'raw_detection_results.txt';

    % read raw detection results file to get initial tracking position of bounding boxes
    init_bbx = parse_raw_detection_file(raw_detection_results);

    all_objs = keys(init_bbx);

    for obj = 1:init_bbx.Count
        % detected object
        disp(all_objs(obj));
        % position of bounding box
        disp(init_bbx(char(all_objs(obj))));
    end
    %disp(init_bbx('butter')); 
end

% for mapping object id to its name
function obj_map = road_obj_map()

    obj_map = containers.Map('KeyType', 'char', 'ValueType', 'any');
    obj_map('butter') = [1 10];
    obj_map('coffee') = [2 11 14];
    obj_map('oatmeal') = [3 9 13];
    obj_map('pan') = [4 12];
    obj_map('table_mat') = [5];
    obj_map('beer_box') = [6 7 8];
    obj_map('cup') = [15];
    obj_map('my_left') = [96];
    obj_map('my_right') = [97];
    obj_map('other_left') = [98];
    obj_map('my_right') = [99];
end

% read raw hand/object detection file
function init_bbx = parse_raw_detection_file(detection_file)

    init_bbx = containers.Map('KeyType', 'char', 'ValueType', 'any');

    fid = fopen(detection_file);

    tline = fgetl(fid);
    while ischar(tline)
        %disp(tline);
        new = sscanf(tline, '%s', 1);

        if strcmp(new,'#')
            % get frame id
            id = sscanf(tline, '%s %f');
            frame_id = id(2); 
        else
            %results = sscanf(tline, '%c, %f, %f, %f, %f, %f');
            results = regexp(tline, '\w*', 'match');
            obj_id = char(results(1));
            if ~isKey(init_bbx, obj_id)
                init_bbx(obj_id) = [frame_id str2num(char(results(3))) str2num(char(results(4))) ...
                                    str2num(char(results(5))) str2num(char(results(6)))];
            end
        end
        tline = fgetl(fid);
    end

    fclose(fid);
end
