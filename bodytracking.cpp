#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#define VERIFY(result, error) \
 if(result != K4A_RESULT_SUCCEEDED) \
 { \
 printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
 exit(1); \
 } \
int main()
{
	float totalchesty = 0;
	float totalnavaly = 0;
	float totalrhandtipy = 0;
	float totalrhandy = 0;
	float totalrwristy = 0;
	float totalrshouldery = 0;
	float totalclavicley = 0;
	float totalheady = 0;

	k4a_device_t device = NULL;
	VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");
	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");
	k4a_calibration_t sensor_calibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution,
		&sensor_calibration),
		"Get depth camera calibration failed!");
	k4abt_tracker_t tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization
		failed!");
		int frame_count = 0;
	float trigger = 0;
	int trigcount = 0;
	//not sure if this needs to change based on arm movement
	float rhand[900];
	do
	{
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture,
			K4A_WAIT_INFINITE);
		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			frame_count++;
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker,
				sensor_capture, K4A_WAIT_INFINITE);
			k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you
			finish using it
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
			k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame,
				K4A_WAIT_INFINITE);
			if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Successfully popped the body tracking result. Start your processing
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				printf("%zu bodies are detected!\n", num_bodies);
				for (size_t i = 0; i < num_bodies; i++)
				{
					k4abt_skeleton_t skeleton;
					k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
					uint32_t id = k4abt_frame_get_body_id(body_frame, i);
					float headxinitial = skeleton.joints[15].position.xyz.x; //change this index to be about bowler arm
					float headyinitial = skeleton.joints[15].position.xyz.y; //this is about the whole hand
					float headzinitial = skeleton.joints[15].position.xyz.z;
					printf("Head Initial: \n");
					printf("X: %f\n", headxinitial);
					printf("Y: %f\n", headyinitial);
					printf("Z: %f\n\n", headzinitial);
					if (headyinitial < 0) // look only at hand
					{
						float rhandx = skeleton.joints[15].position.xyz.x; //dont know if i will have to change this at all or even include it?
						float rhandy = skeleton.joints[15].position.xyz.y; //change to be about right arm
						//track hand
						rhand[frame_count] = rhandy; //change to be about
						//activate trigger to collect other values if hand moves
						if (rhand[frame_count] > (abs(rhand[frame_count - 5] + 200)))//if they are diff
						{
							trigger++;
						}
						//hand
						float rhandz = skeleton.joints[15].position.xyz.z;
						printf("Right Hand: \n");
						printf("X: %f\n", rhandx);
						printf("Y: %f\n", rhandy);
						printf("Y: array value %f\n", rhand[frame_count]);
						printf("Y: array value -5 %f\n", rhand[frame_count - 5]);
						printf("Z: %f\n", rhandz);
						printf("Trigger = %f\n", trigger);
					}
					if (trigger > 1)
					{ 
						//change these to be body parts we care about
						float rclaciclex = skeleton.joints[12].position.xyz.x;
						float rclavicley = skeleton.joints[12].position.xyz.y;
						float rclaviclez = skeleton.joints[12].position.xyz.z;
						printf("R Shoulder-Clavicle: \n");
						printf("X: %f\n", rclaviclex);
						printf("Y: %f\n", rclavicley);
						printf("Z: %f\n", rclaviclez);
						
						float rshoulderx = skeleton.joints[13].position.xyz.x;
						float rshouldery = skeleton.joints[13].position.xyz.y;
						float rshoulderz = skeleton.joints[13].position.xyz.z;
						printf("RElbow-Shoulder: \n");
						printf("X: %f\n", rshoulderx);
						printf("Y: %f\n", rshouldery);
						printf("Z: %f\n", rshoulderz);
						
						float relbowx = skeleton.joints[14].position.xyz.x;
						float relbowy = skeleton.joints[14].position.xyz.y;
						float relbowz = skeleton.joints[14].position.xyz.z;
						printf("RWrist-Elbow: \n");
						printf("X: %f\n", relbowx);
						printf("Y: %f\n", relbowy);
						printf("Z: %f\n", relbowz);

						float rhandx = skeleton.joints[15].position.xyz.x;
						float rhandy = skeleton.joints[15].position.xyz.y;
						float rhandz = skeleton.joints[15].position.xyz.z;
						printf("RHand-Wrist: \n");
						printf("X: %f\n", rhandx);
						printf("Y: %f\n", rhandy);
						printf("Z: %f\n", rhandz);

						float rhandtipx = skeleton.joints[16].position.xyz.x;
						float rhandtipy = skeleton.joints[16].position.xyz.y;
						float rhandtipz = skeleton.joints[16].position.xyz.z;
						printf("R Hand Tip-Hand: \n");
						printf("X: %f\n", rhandtipx);
						printf("Y: %f\n", rhandtipy);
						printf("Z: %f\n", rhandtipz);

						float navalx = skeleton.joints[1].position.xyz.x;
						float navaly = skeleton.joints[1].position.xyz.y;
						float navalz = skeleton.joints[1].position.xyz.z;
						printf("Spine - Naval: \n");
						printf("X: %f\n", navalx);
						printf("Y: %f\n", navaly);
						printf("Z: %f\n", navalz);
						float chestx = skeleton.joints[2].position.xyz.x;
						float chesty = skeleton.joints[2].position.xyz.y;
						float chestz = skeleton.joints[2].position.xyz.z;
						printf("Spine - Chest: \n");
						printf("X: %f\n", chestx);
						printf("Y: %f\n", chesty);
						printf("Z: %f\n", chestz);
						//able to track catcher
						float headx = skeleton.joints[26].position.xyz.x;
						float heady = skeleton.joints[26].position.xyz.y;
						float headz = skeleton.joints[26].position.xyz.z;
						printf("Head: \n");
						printf("X: %f\n", headx);
						printf("Y: %f\n", heady);
						printf("Z: %f\n\n", headz);
						
						trigcount++;
					}
					//can take catcher part out
				}
				k4a_image_t body_index_map =
					k4abt_frame_get_body_index_map(body_frame);
				k4a_image_release(body_index_map);
				k4abt_frame_release(body_frame); // Remember release frame once done with it
			}
			else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
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
	} while (trigcount < 60); //collect data for about 1 second
	//able to track catcher
	float chestaverage;
	float navalaverage;
	float rhandtipaverage;
	float rhandaverage;
	float rwristaverage;
	float rshoulderaverage;
	float rclavicleaverage;
	float headaverage;
	
	rhandtipaverage = totalrhandtipy / 30;
	rhandaverage = totalrhandy / 30;
	rwristaverage = totalrwristy / 30;
	rshoulderaverage = totalrshouldery / 30;
	rclavicleaverage = totalclavicley / 30;
	navalaverage = totalnavaly / 30;
	chestaverage = totalchesty / 30;
	headaverage = totalheady / 30;
	
	
	FILE* fp;
	errno_t err;
	err = fopen_s(&fp,
		//change file name
		"C:\\CEI\\CEI_Matlab_files_07_19_2019\\CEI_Matlab_files_07_19_2019\\Camera\\Matlab_Capstone\\camera_test
		.txt", "w");
		if (err != NULL)
		{
			printf("Error Opening File!\n");
			return 0;
		}
	fprintf(fp, "Spine - Chest: %f \n", chestaverage);
	fprintf(fp, "Spine - Naval: %f \n", navalaverage);
	fprintf(fp, "Right Hand Tip: %f \n", rhandtipaverage);
	fprintf(fp, "Right Hand: %f \n", rhandaverage);
	fprintf(fp, "Right Elbow: %f \n", rwristaverage);
	fprintf(fp, "Right Shoulder: %f \n", rshoulderaverage);
	fprintf(fp, "Right Clavicle: %f \n", rclavicleaverage);
	fprintf(fp, "Head: %f \n", headaverage);
	fclose(fp);
	
	FILE* fmat;
	errno_t errmat;
	errmat = fopen_s(&fmat,
		//change file name
		"C:\\Users\\jorr5\\Documents\\MATLAB\\CEI_Matlab_files_07_19_2019\\cameramat_test.txt", "w");
	
		if (errmat != NULL)
		{
			printf("Error Opening File!\n");
			return 0;
		}
	fprintf(fmat, "Spine - Chest: %f \n", chestaverage);
	fprintf(fmat, "Spine - Naval: %f \n", navalaverage);
	fprintf(fmat, "Right Hand Tip: %f \n", rhandtipaverage);
	fprintf(fmat, "Right Hand: %f \n", rhandaverage);
	fprintf(fmat, "Right Wrist: %f \n", rwristaverage);
	fprintf(fmat, "Right Shoulder: %f \n", rshoulderaverage);
	fprintf(fmat, "Right Clavicle: %f \n", rclavicleaverage);
	fprintf(fmat, "Head: %f \n", headaverage);
	fclose(fmat);
	
	FILE* ff;
	errno_t err3;
	err3 = fopen_s(&ff,
		//change file name //also change this one to calculate the velocity by using equations release velocity upon ball leaving hand, calculated with distance from point a to point b and time
		"C:\\CEI\\CEI_Matlab_files_07_19_2019\\CEI_Matlab_files_07_19_2019\\Camera\\Matlab_Capstone\\height_off_
		ground.txt", "w");
		if (err3 != NULL)
		{
			printf("Error Opening File!\n");
			return 0;
		}
	//put print statement here that has velocity calculations
	fclose(ff);
	
	printf("Finished body tracking processing!\n");
	k4abt_tracker_shutdown(tracker);
	k4abt_tracker_destroy(tracker);
	k4a_device_stop_cameras(device);
	k4a_device_close(device);
	return 0;
}
