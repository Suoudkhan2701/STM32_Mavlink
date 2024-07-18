/*
 * Autopilot.h
 *
 *  Created on: Jul 10, 2024
 *      Author: suoud
 */

#ifndef INC_AUTOPILOT_H_
#define INC_AUTOPILOT_H_
#include <time.h>
#include <sys/time.h>
struct Time_Stamps
{
	Time_Stamps()
	{reset_timestamps();}
	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
	uint64_t control;
	uint64_t custom;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
		control=0;
		custom=0;
	}
};

struct Mavlink_Messages {

	int sysid;
	int compid;

	mavlink_heartbeat_t heartbeat;
	mavlink_sys_status_t sys_status;
	mavlink_battery_status_t battery_status;
	mavlink_radio_status_t radio_status;
	mavlink_local_position_ned_t local_position_ned;
	mavlink_global_position_int_t global_position_int;
	mavlink_position_target_local_ned_t position_target_local_ned;
	mavlink_position_target_global_int_t position_target_global_int;
	mavlink_highres_imu_t highres_imu;
	mavlink_attitude_t attitude;
	mavlink_manual_control_t control;
	mavlink_get_attitude_battery_t custom;

	Time_Stamps time_stamps; //Periodically Checked

	void reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}
};
class Autopilot_Interface{
    public:
	Autopilot_Interface(){};
	//autopilot_interface();
	~Autopilot_Interface(){};
    //char reading_status;
	char writing_status;
	bool control_status=false;
    uint64_t write_count;
    mavlink_status_t lastStatus;

    int system_id;
	int autopilot_id;
	int companion_id;
	int buttons;
	int x;
	int y;
	int z;
	int r;
	Mavlink_Messages current_messages;
	mavlink_set_position_target_local_ned_t initial_position;

	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void read_messages();
	int read_message(mavlink_message_t &message);
    int write_message(mavlink_message_t &message);

	int	 arm_disarm(bool flag);
	void enable_offboard_control();
	void disable_offboard_control();
	int move_control(uint16_t x, uint16_t y, uint16_t z, uint16_t r, uint16_t buttons, bool flag);

	void start();
	void stop();
	void read_thread();
	void write_thread(void);
	void handle_quit( int sig );
	void write_command(mavlink_message_t &message);
	void Commands(Autopilot_Interface &api, bool autotakeoff);
	void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
	void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);

private:
	int toggle_offboard_control( bool flag );
	void write_setpoint();
	bool time_to_exit;
	bool reading_status;
	struct {
		//std::mutex mutex;
		mavlink_set_position_target_local_ned_t data;
	} current_setpoint;
};

//void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
//void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void Commands(Autopilot_Interface &api, bool autotakeoff);
uint64_t get_time_usec();


#endif /* INC_AUTOPILOT_H_ */
