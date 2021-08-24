// gcc body_tracking_test.c -lk4a -lk4abt
// https://docs.microsoft.com/ja-jp/azure/kinect-dk/build-first-body-app

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <unistd.h>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <k4abttypes.h>

#include <opencv2/opencv.hpp>

using namespace std;

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

int sensorType = 71;
int last_json_deci_second = -1;
int cameraNo = 0;
int captureMode = 0;
bool isSaveCapture = false;
int frameNo = 0;
bool nextOutCaptureJson = false;

std::string prg_dir;

k4a_calibration_t sensor_calibration;
k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

bool existFile(const char* path)
{
    FILE* fp = fopen(path, "r");
    if (fp == NULL) {
        return false;
    }

    fclose(fp);
    return true;
}

bool isCaptureNow() {
    char capture_txt_path[255];
    sprintf(capture_txt_path, "%s/capture.txt", prg_dir.c_str());

    bool ret = existFile(capture_txt_path);
    if(ret) {
        int err = remove(capture_txt_path);
    }
    return ret;
}

void outputJson(int sensorNo, size_t num_bodies, k4abt_frame_t body_frame) {
	// ディレクトリ作成
	char save_dir[] = "/ramdisk/upload";
	char mkdir_cmd[255];
	sprintf(mkdir_cmd, "mkdir -p %s", save_dir);
	system(mkdir_cmd);

	// 時間取得
	struct tm* tm_info;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	// 100msec周期判定
	const int json_deci_second = tv.tv_usec / 100000;
	if(last_json_deci_second == json_deci_second) return;

	last_json_deci_second = json_deci_second;

	tm_info = gmtime(&tv.tv_sec);

	char sensing_time[64];
	strftime(sensing_time, 36, "%Y-%m-%d %H:%M:%S", tm_info);
	sprintf(sensing_time, "%s.%06ld", sensing_time, tv.tv_usec);

	printf("%s\n", sensing_time);

	// json 出力
	char save_json_file[255];
	char save_json_file_tmp[255];

	sprintf(save_json_file_tmp, "%s/%s_%d_%d", save_dir, sensing_time, sensorType, sensorNo);
	sprintf(save_json_file, "%s.json", save_json_file_tmp);

	ofstream json_file(save_json_file_tmp);

	json_file << "{" << endl;
	json_file << "	\"sensing_time\" : \"" << sensing_time << "\"," << endl;
	json_file << "	\"type\" : " << sensorType << "," << endl;
	json_file << "	\"no\" : " << sensorNo << "," << endl;
	json_file << "	\"data\" : {" << endl;
	json_file << "		\"skeletons\" : [" << endl;

    for (size_t i = 0; i < num_bodies; i++) {        
        k4abt_skeleton_t skeleton;
        k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        uint32_t id = k4abt_frame_get_body_id(body_frame, i);

        json_file << "{" << endl;

        json_file << "  \"id\" : " << id << "," << endl;

        // 定義はこちら
        // https://microsoft.github.io/Azure-Kinect-Body-Tracking/release/0.9.x/k4abttypes_8h_source.html

        json_file << "  \"joints\" : [" << endl;
        for(int jIdx = 0; jIdx < K4ABT_JOINT_COUNT; jIdx++) {
            k4abt_joint_t joint = skeleton.joints[jIdx];

            json_file << "  {" << endl;

            // 3D
            json_file << "  \"position\" : [" << joint.position.v[0] << "," << joint.position.v[1] << "," << joint.position.v[2] << "]," << endl;

            // 2D
            if(deviceConfig.color_resolution != K4A_COLOR_RESOLUTION_OFF) {
                k4a_float2_t target_point2d;
                int valid;
                k4a_calibration_3d_to_2d(&sensor_calibration, &joint.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &target_point2d, &valid);

                json_file << "  \"position2d\" : [" << target_point2d.v[0] << "," << target_point2d.v[1] << "]," << endl;

                // printf("3D %f, %f, %f\n", joint.position.v[0], joint.position.v[1], joint.position.v[2]);
                // printf("2D %f, %f\n", target_point2d.v[0], target_point2d.v[1]);
            }

            // orient
            json_file << "  \"orientation\" : [" << joint.orientation.v[0] << "," << joint.orientation.v[1] << "," << joint.orientation.v[2] << "," << joint.orientation.v[3] << "]" << endl;

            json_file << "  }" << endl;

            if((jIdx + 1) != K4ABT_JOINT_COUNT) {
                json_file << "," << endl;
            }
        }
        json_file << "  ]" << endl;

	    json_file << "}" << endl;

        if((i + 1) != num_bodies) {
            json_file << "," << endl;
        }
    }

	json_file << "		]" << endl;
	
	json_file << "	}" << endl;
	json_file << "}" << endl;

	json_file.close();

    if(nextOutCaptureJson) {
        char captureFilename[255];
        char captureTmpFilename[255];

        sprintf(captureFilename, "%s/capture.json", prg_dir.c_str());
        sprintf(captureTmpFilename, "%s/capture_tmp.json", prg_dir.c_str());

        char copy_cmd[255];
        sprintf(copy_cmd, "cp -f \"%s\" \"%s\"", save_json_file_tmp, captureTmpFilename);
        system(copy_cmd);

        rename(captureTmpFilename, captureFilename);

        //printf("aa : %s  %s  %s\n", copy_cmd, captureTmpFilename, captureFilename);

        nextOutCaptureJson = false;
    }

	rename(save_json_file_tmp, save_json_file);
}

volatile sig_atomic_t e_flag = 0;

void abrt_handler(int sig) {
    e_flag = 1;
}

bool capture(k4a_device_t & device, k4abt_tracker_t & tracker) {
    k4a_capture_t sensor_capture;
    k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
    if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        // capture color image
        if(captureMode == 1 || (captureMode == 2 && isCaptureNow())) {
            k4a_image_t colorImage = k4a_capture_get_depth_image(sensor_capture);
            //k4a_image_t colorImage = k4a_capture_get_depth_image(sensor_capture);
            //k4a_image_t colorImage = k4a_capture_get_ir_image(sensor_capture);
            if (colorImage != NULL)
            {
                // you can check the format with this function
                k4a_image_format_t format = k4a_image_get_format(colorImage); // K4A_IMAGE_FORMAT_COLOR_BGRA32 
                
                // get raw buffer
                uint8_t* buffer = k4a_image_get_buffer(colorImage);
                
                // convert the raw buffer to cv::Mat
                int rows = k4a_image_get_height_pixels(colorImage);
                int cols = k4a_image_get_width_pixels(colorImage);
                cv::Mat depth_mat = cv::Mat( rows, cols, CV_16UC1, reinterpret_cast<uint16_t*>( buffer ) );
                depth_mat.convertTo( depth_mat, CV_8U, 255.0 / 5000.0, 0.0 );

                cv::Mat colorMat(rows , cols, CV_8UC4, (void*)buffer, cv::Mat::AUTO_STEP);
                
                cv::namedWindow( "Display window", cv::WINDOW_NORMAL); // Create a window for display.
                cv::imshow( "Display window", depth_mat); // Show our image inside it.
                cv::waitKey(1); // Wait for a keystroke in the window
                
                printf(" | image %d  %4dx%4d stride:%5d\n",
                        k4a_image_get_format(colorImage),
                        k4a_image_get_height_pixels(colorImage),
                        k4a_image_get_width_pixels(colorImage),
                        k4a_image_get_stride_bytes(colorImage));

                if(isSaveCapture) {
                    // vector<int> compression_params = vector<int>(2);
                    // compression_params[0] = cv::CV_IMWRITE_JPEG_QUALITY;
                    // compression_params[1] = 100;

                    // cv::imwrite("/ramdisk/filename.jpg", colorMat, compression_params);

                    frameNo++;
                    char filename[100];
                    if (captureMode == 1) {
                        sprintf(filename, "/ramdisk/capture_%d_%010d.png", cameraNo, frameNo);
                    } else if(captureMode == 2) {
                        sprintf(filename, "%s/capture.png", prg_dir.c_str());
                        nextOutCaptureJson = true;
                    }

                    cv::imwrite(filename, depth_mat);
                }

                k4a_image_release(colorImage);
            }
        }

        k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
        k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
        
        if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit timeout when K4A_WAIT_INFINITE is set.
            printf("Error! Add capture to tracker process queue timeout!\n");
            return false;
        }
        else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
        {
            printf("Error! Add capture to tracker process queue failed!\n");
            return false;
        }

        k4abt_frame_t body_frame = NULL;
        k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
        if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // Successfully popped the body tracking result. Start your processing

            size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
            printf("%zu bodies are detected!\n", num_bodies);

            outputJson(cameraNo, num_bodies, body_frame);

            k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
        }
        else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            //  It should never hit timeout when K4A_WAIT_INFINITE is set.
            printf("Error! Pop body frame result timeout!\n");
            return false;
        }
        else
        {
            printf("Pop body frame result failed!\n");
            return false;
        }
    }
    else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        // It should never hit time out when K4A_WAIT_INFINITE is set.
        printf("Error! Get depth frame time out!\n");
        return false;
    }
    else
    {
        printf("Get depth capture returned error: %d\n", get_capture_result);
        return false;
    }

    return true;
}

void showSerialNo(k4a_device_t & device) {
    // Get the size of the serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char *serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    printf("Opened device: %s\n", serial);
    free(serial);
}

int main(int argc, char *argv[])
{
    if (signal(SIGINT, abrt_handler) == SIG_ERR) {
        exit(1);
    }

    cameraNo = 0;


    if(argc > 1) {
        cameraNo = atoi(argv[1]);
    }
    if(argc > 2) {
        captureMode = atoi(argv[2]);
    }
    if(argc > 3) {
        isSaveCapture = atoi(argv[3]) == 1;
    }

    isSaveCapture |= captureMode == 2;

    char cprg_path[256];
    readlink("/proc/self/exe", cprg_path, sizeof(cprg_path) );
    std::string prg_path(cprg_path); 
    prg_dir = prg_path.substr(0, prg_path.find_last_of('/'));
    printf("prg_dir %s\n", prg_dir.c_str());

    printf("capture mode = %d\n", captureMode);

    uint32_t count = k4a_device_get_installed_count();

    printf("installed camera count %d \n", count);

    printf("camera no %d\n", cameraNo);

    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(cameraNo, &device), "Open 0 K4A Device failed!");

    showSerialNo(device);

    // Start camera. Make sure depth camera is enabled.
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // K4A_DEPTH_MODE_WFOV_2X2BINNED

    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.color_resolution = captureMode > 0 ? K4A_COLOR_RESOLUTION_1536P : K4A_COLOR_RESOLUTION_OFF;

    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    while (!e_flag) {
        bool ret = capture(device, tracker);
        if(!ret) break;
    };

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}