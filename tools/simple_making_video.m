function [] = simple_making_video()

    video_name = 'robot_view_0101';
    outputVideo = VideoWriter(video_name);
    outputVideo.FrameRate = 4;
    open(outputVideo);

    % Input
    input_path = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/video/0101/future_pred_result/';
    num_of_frames = 42;

    for f = 6:num_of_frames
        %frame_name = [input_path 'f_result_' num2str(f) '.jpg'];
        frame_name = [input_path 'result_' num2str(f) '.jpg'];
        disp(frame_name)
        img = imread(frame_name);
        writeVideo(outputVideo, img);
    end

    close(outputVideo);

    disp('done!')
end
