function [] = simple_making_video()

    video_name = 'Hand_SSD_500_prediction_baseline_interaction_set_only.avi';
    outputVideo = VideoWriter(video_name);
    outputVideo.FrameRate = 30;
    open(outputVideo);

    % Input
    input_path = '/home/leejang/data/test/detection_results_future_baseline_interaction_set_only/';
    num_of_frames = 233;

    for f = 0:(num_of_frames - 1)
        frame_name = [input_path 'frame_' num2str(f) '.jpg.result.jpg'];
        disp(frame_name)
        img = imread(frame_name);
        writeVideo(outputVideo, img);
    end

    close(outputVideo);

    disp('done!')
end
