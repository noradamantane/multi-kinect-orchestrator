#include <stdio.h>
#include <stdlib.h>

#include <k4a/k4a.h>
#include <k4abt.h>

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

static const char* joint_names[] = {
    "pelvis", "spine_naval", "spine_chest", "neck", "clavicle_left",
    "shoulder_left", "elbow_left", "wrist_left", "hand_left", "handtip_left",
    "thumb_left", "clavicle_right", "shoulder_right", "elbow_right",
    "wrist_right", "hand_right", "handtip_right", "thumb_right", "hip_left",
    "knee_left", "ankle_left", "foot_left", "hip_right", "knee_right",
    "ankle_right", "foot_right", "head", "nose", "eye_left", "ear_left",
    "eye_right", "ear_right"
};

//thanks chatgpt
/**
 * Write all skeletons from a k4abt_frame_t into an open JSON file.
 *
 * @param fp    An open FILE* (already positioned inside "frames": [])
 * @param body_frame A valid body tracking frame
 * @param frame_index Your running frame counter
 * @param first_frame Pass 1 if this is the first JSON frame, 0 otherwise
 */
void export_skeletons_json(FILE *fp, k4abt_frame_t body_frame, int frame_index, int first_frame)
{
    uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

    if (!first_frame) {
        fprintf(fp, ",\n");
    }
    fprintf(fp, "    {\n      \"frame\": %d,\n      \"skeletons\": [\n", frame_index);

    for (uint32_t i = 0; i < num_bodies; i++) {
        k4abt_skeleton_t skeleton;
        k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        uint32_t id = k4abt_frame_get_body_id(body_frame, i);

        if (i > 0) fprintf(fp, ",\n");
        fprintf(fp, "        {\n          \"id\": %u,\n          \"joints\": {\n", id);

        for (int j = 0; j < (int)K4ABT_JOINT_COUNT; j++) {
            k4a_float3_t pos = skeleton.joints[j].position;
            k4abt_joint_confidence_level_t conf = skeleton.joints[j].confidence_level;

            fprintf(fp, "            \"%s\": {\"pos\": [%.3f, %.3f, %.3f], \"conf\": %d}",
                    joint_names[j], pos.v[0], pos.v[1], pos.v[2], conf);

            if (j < (int)K4ABT_JOINT_COUNT - 1) fprintf(fp, ",\n");
            else fprintf(fp, "\n");
        }

        fprintf(fp, "          }\n        }");
    }

    fprintf(fp, "\n      ]\n    },"); // added , to allow it to be correct json separation between frames
}


int main()
{
    
    FILE *fp = fopen("skeleton_output.json", "w");
    fprintf(fp, "{\n  \"frames\": [\n");

    //microsoft code
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    int frame_count = 0;
    int first_frame = (frame_count == 0);
    do
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing

                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                printf("%zu bodies are detected!\n", num_bodies);

                //write to json file?
               export_skeletons_json(fp, body_frame, frame_count, first_frame);


                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

    } while (frame_count < 100);
    //close out JSON file
    fprintf(fp, "\n  ]\n}\n");
    fclose(fp);
    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}