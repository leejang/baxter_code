%function proposals = genWindowProposals(img_path, output_file_path, num_windows)
function proposals = genWindowProposals(num_windows, phase)

img_path = '/home/leejang/data/hands/current_frame.jpg';
output_file_path = '/home/leejang/data/hands/current_windows.txt';

%proposals = num_windows + 1;

% add KDE package
addpath(genpath('kde'));

if strcmp(phase, 'test')
  % load skin and KDE models
  load('box_proposal_model_test.mat', 'model');
elseif strcmp(phase, 'train')
  % load skin and KDE models
  load('box_proposal_model_train.mat', 'model');
else
  disp('please choose correct (test or train) phase');
  return
end

% OPEN FILE
output_file = fopen(output_file_path, 'W');

image_index = 0;
% get a target image
img = imread(img_path);

% size of image
[img_height, img_width, img_dim] = size(img);

% generate window proposals
boxes = sampleProposals(img, model.kde_whxy_hands, model.skin, num_windows, phase);

%turn [r c r c] into [x y x y]
boxes = [boxes(:,2) boxes(:,1) boxes(:,4) boxes(:,3)];
n_boxes = size(boxes, 1);

fprintf(output_file, '# %d\n%s\n%d\n%d\n%d\n', image_index, img_path, img_dim, img_height, img_width);

% print number of windows
fprintf(output_file, '%d\n', n_boxes);

% print each window
for window_i = 1:n_boxes
    window = boxes(window_i, :);
    fprintf(output_file, '0 0.0 %d %d %d %d\n', window(1), window(2), window(3), window(4));
end

% CLOSE FILE
fclose(output_file);

end
