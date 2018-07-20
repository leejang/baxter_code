function [] = simple_making_video()

    video_name = 'scenario1_robot_view';
    outputVideo = VideoWriter(video_name);
    outputVideo.FrameRate = 5;
    open(outputVideo);

    % Input
    input_path = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/new_robot_video/0103/future_pred_result/';
    num_of_frames = 149;

    for f = 0:num_of_frames
        %frame_name = [input_path 'f_result_' num2str(f) '.jpg'];
        frame_name = [input_path 'result_' num2str(f) '.jpg'];
        disp(frame_name)
        img = imread(frame_name);
        writeVideo(outputVideo, img);
    end

    close(outputVideo);

    disp('done!')
end
