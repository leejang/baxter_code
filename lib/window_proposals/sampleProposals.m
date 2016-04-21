function boxes = sampleProposals(img, kde, skinmodel, numSamples)
    
    skin_probs = get_skin_probs(img, skinmodel);

    minBoxWidth = 25;
    maxArea = 400*400;
    minArea = 1600;

    pnts = floor(getPoints(kde.ml))';            
    winds = sub2ind(size(skin_probs), pnts(:,4), pnts(:,3)); % get row column as linear indicies
    weights = skin_probs(winds)';
    weights(pnts(:,1) < minBoxWidth) = 0;
    weights(pnts(:,1).*pnts(:,2) < minArea) = 0;
    weights(pnts(:,1).*pnts(:,2) > maxArea) = 0;
    weights = weights ./ sum(weights);
    adjustWeights(kde.ml, weights); % each input point corresponds to one kernel, weight kernel by skin probability at point

    pnts = floor(getPoints(kde.mr))';
    winds = sub2ind(size(skin_probs), pnts(:,4), pnts(:,3));
    weights = skin_probs(winds)';
    weights(pnts(:,1) < minBoxWidth) = 0;
    weights(pnts(:,1).*pnts(:,2) < minArea) = 0;
    weights(pnts(:,1).*pnts(:,2) > maxArea) = 0;
    weights = weights ./ sum(weights);
    adjustWeights(kde.mr, weights);

    pnts = floor(getPoints(kde.yl))';
    winds = sub2ind(size(skin_probs), pnts(:,4), pnts(:,3));
    weights = skin_probs(winds)';
    weights(pnts(:,1) < minBoxWidth) = 0;
    weights(pnts(:,1).*pnts(:,2) < minArea) = 0;
    weights(pnts(:,1).*pnts(:,2) > maxArea) = 0;
    weights = weights ./ sum(weights);
    adjustWeights(kde.yl, weights);

    pnts = floor(getPoints(kde.yr))';
    winds = sub2ind(size(skin_probs), pnts(:,4), pnts(:,3));
    weights = skin_probs(winds)';
    weights(pnts(:,1) < minBoxWidth) = 0;
    weights(pnts(:,1).*pnts(:,2) < minArea) = 0;
    weights(pnts(:,1).*pnts(:,2) > maxArea) = 0;
    weights = weights ./ sum(weights);
    adjustWeights(kde.yr, weights);

    sampleSize = numSamples*1.2;
    ml_size = floor(sampleSize*kde.prior(1));
    mr_size = floor(sampleSize*kde.prior(2));
    yl_size = floor(sampleSize*kde.prior(3));
    yr_size = sampleSize - ml_size - mr_size - yl_size;
    boxes = vertcat(sample(kde.ml, ml_size)', ...
                    sample(kde.mr, mr_size)', ...
                    sample(kde.yl, yl_size)', ...
                    sample(kde.yr, yr_size)');
         
    % turn [width height xc yc] into [r c r c]
    boxes = [boxes(:,4)-boxes(:,2)/2, boxes(:,3)-boxes(:,1)/2, ...
            boxes(:,4)+boxes(:,2)/2, boxes(:,3)+boxes(:,1)/2];
    boxes = round(boxes);

    % crop windows that are (partly) out of the image
    boxes(boxes(:,1) < 1, 1) = 1;
    boxes(boxes(:,1) > 720, 1) = 720;
    boxes(boxes(:,2) < 1, 2) = 1;
    boxes(boxes(:,2) > 1280, 2) = 1280;
    boxes(boxes(:,3) < 1, 3) = 1;
    boxes(boxes(:,3) > 720, 3) = 720;
    boxes(boxes(:,4) < 1, 4) = 1;
    boxes(boxes(:,4) > 1280, 4) = 1280;

    % get rid of windows with an area of 0
    zero_area = any([boxes(:,1) >= boxes(:,3) boxes(:,2) >= boxes(:,4)], 2);
    boxes(zero_area, :) = [];

    % get rid of windows too big or too small
    too_small = any([(boxes(:,4)-boxes(:,2)) .* (boxes(:,3)-boxes(:,1)) < minArea], 2);
    boxes(too_small, :) = [];
    too_big = any([(boxes(:,4)-boxes(:,2)) .* (boxes(:,3)-boxes(:,1)) > maxArea], 2);
    boxes(too_big, :) = [];

    %downsample to exact numSamples
    boxes = boxes(randperm(size(boxes,1)),:);
    boxes = boxes(1:min(size(boxes,1), numSamples),:);
end

function skin_probs = get_skin_probs(img, model)
    img = rgb2ycbcr(img);
    [rows, cols, ~] = size(img);
    u = img(:,:,2);
    v = img(:,:,3);
    u = double(u(:));
    v = double(v(:));
    skin_prob = sub2ind(size(model), u, v);
    skin_probs = reshape(model(skin_prob), rows, cols); 
end
