#include "UM7.h"

//////////////////////////////////////////
//	READ FUNCTIONS FOR THE UM7	//
//////////////////////////////////////////

/*
	Default constructor. Initial state and serial port, pass as reference
*/
UM7::UM7(HardwareSerial &serial) { 
	state = STATE_ZERO;
	serial_port = &serial;
}

/*
	The function that decodes and parses incoming data from the UM7. 
	The function will determine which register is being read, the batch length, and call checksum once finished.
	current_byte is current byte being read by serial.
	decode() returns true if a packet was read succesfully (if checksum returns true).
*/
bool UM7::decode(byte current_byte) {

	switch(state) {
	case STATE_ZERO:
		if (current_byte == 's') {
			state = STATE_S;		// Entering state S from state Zero
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_S:
		if (current_byte == 'n') {
			state = STATE_SN;		// Entering state SN from state S
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_SN:
		if (current_byte == 'p') {
			state = STATE_SNP;		// Entering state SNP from state SN.  Packet header detected.
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_SNP:
		state = STATE_PT;			// Entering state PT from state SNP.  Decode packet type.
		packet_type = current_byte;
		packet_has_data = (packet_type >> 7) & 0x01;
		packet_is_batch = (packet_type >> 6) & 0x01;
		batch_length    = (packet_type >> 2) & 0x0F;
		if (packet_has_data) {
			if (packet_is_batch) {
				data_length = 4 * batch_length;	// Each data packet is 4 bytes long
			} else {
				data_length = 4;
			}
		} else {
			data_length = 0;
		}
		return false;
	case STATE_PT:
		state = STATE_DATA;		// Next state will be READ_DATA.  Save address to memory. (eg 0x70 for a DREG_EULER_PHI_THETA packet)
		address = current_byte;
		data_index = 0;
		return false;
	case STATE_DATA:			//  Entering state READ_DATA.  Stay in state until all data is read.
		data[data_index] = current_byte;
		data_index++;
		if (data_index >= data_length){
			state = STATE_CHK1;	//  Data read completed.  Next state will be CHK1
		}
		return false;
	case STATE_CHK1:			// Entering state CHK1, save the byte as checksum1.  Next state will be CHK0
		state = STATE_CHK0;
		checksum1 = current_byte;
		return false;
	case STATE_CHK0:
		state = STATE_ZERO;		// Entering state CHK0, save the byte as checksum0.  Next state will be state Zero.
		checksum0 = current_byte;
		return checksum();
	default:
		return false;
	}
}

/*
	Checksum is used as a backcheck by the UM7 to ensure the correct packet was read.
	This function ties directly with decode() and calls save() if the checksum matches the parsed data.
	checksum() returns true if a packet is read succesfully.
*/
bool UM7::checksum() {
	checksum10 = ((checksum1 << 8) | checksum0);	// Combine checksum1 and checksum0
	computed_checksum = 's' + 'n' + 'p' + packet_type + address;
	for (int i = 0; i < data_length; i++) { // computed_checksum can only be 16bits long (2B)
		computed_checksum += data[i];
	}
	if (checksum10 == computed_checksum) {
		save();
		return true;
	} else {
		return false;
	}
}

/*
	The list of 'readable' registers by the UM7. Assigns the data[] variable to whatever dataset is beign read.
	The function switches for each register case.
*/
void UM7::save() {
	switch (address) {

	case DREG_HEALTH:
		error = read_register_as_float(0);
	break;


	case DREG_GYRO_RAW_XY : // In 2's complement!
		if (packet_is_batch) {
			gyro_raw_x = (uint16_t)((data[0] << 8) | data[1]);
			gyro_raw_y = (uint16_t)((data[2] << 8) | data[3]);
			gyro_raw_z = (uint16_t)((data[4] << 8) | data[5]);
			gyro_raw_time = read_register_as_float(6);

			accel_raw_x = (uint16_t)((data[10] << 8) | data[11]);
			accel_raw_y = (uint16_t)((data[12] << 8) | data[13]);
			accel_raw_z = (uint16_t)((data[14] << 8) | data[15]);
			accel_raw_time = read_register_as_float(16);

			mag_raw_x = (uint16_t)((data[20] << 8) | data[21]);
			mag_raw_y = (uint16_t)((data[22] << 8) | data[23]);
			mag_raw_z = (uint16_t)((data[24] << 8) | data[25]);
			mag_raw_time = read_register_as_float(26);

			temp = read_register_as_float(30);
			temp_time = read_register_as_float(34);
		} else {
			gyro_x = (int16_t)((data[0] << 8) | data[1]);
		}
    break;


	case DREG_GYRO_PROC_X : // CALIBRATION has to be applied first!! Typicaly done in-house @CHR
		if (packet_is_batch) {
			gyro_x = read_register_as_float(0);
			gyro_y = read_register_as_float(4);
			gyro_z = read_register_as_float(8);
			gyro_time = read_register_as_float(12);

			accel_x = read_register_as_float(16);
			accel_y = read_register_as_float(20);
			accel_z = read_register_as_float(24);
			accel_time = read_register_as_float(28);

			mag_x = read_register_as_float(32);
			mag_y = read_register_as_float(36);
			mag_z = read_register_as_float(40);
			mag_time = read_register_as_float(44);
		} else {
			gyro_x = read_register_as_float(0);
		}
    break;

	
	case DREG_QUAT_AB : // NOT a priority!!
		if (packet_is_batch) {
		  quat_a = (int16_t)((data[0]<<8) | data[1]) / 29789.09091;
		  quat_b = (int16_t)((data[2]<<8) | data[3]) / 29789.09091;
		  quat_c = (int16_t)((data[4]<<8) | data[5]) / 29789.09091;
		  quat_d = (int16_t)((data[6]<<8) | data[7]) / 29789.09091;
		  quat_time = read_register_as_float(8);
		} else {
		  quat_a = (int16_t)((data[0]<<8) | data[1]) / 29789.09091;
		  quat_b = (int16_t)((data[2]<<8) | data[3]) / 29789.09091;
		}
    break;
	

	case DREG_EULER_PHI_THETA : // data[6] and data[7] are unused. WORKS!!
  		if (packet_is_batch) {
			roll  = (int16_t)((data[0]<<8) | data[1]) / 91.02222;
    		pitch = (int16_t)((data[2]<<8) | data[3]) / 91.02222;
  			yaw   = (int16_t)((data[4]<<8) | data[5]) / 91.02222;
			roll_rate  = (int16_t)((data[8] << 8) | data[9]) / 16.0;
			pitch_rate = (int16_t)((data[10] << 8) | data[11]) / 16.0;
			yaw_rate   = (int16_t)((data[12] << 8) | data[13]) / 16.0;
			euler_time = read_register_as_float(14);
  		} else {
			roll  = (int16_t)((data[0]<<8) | data[1]) / 91.02222;
			pitch = (int16_t)((data[2]<<8) | data[3]) / 91.02222;
  		}
  	break;


	case DREG_POSITION_N : 
		if (packet_is_batch) {
			north_pos = read_register_as_float(0);
			east_pos = read_register_as_float(4);
			up_pos = read_register_as_float(8);
			pos_time = read_register_as_float(12);
		}
		else {
			north_pos = read_register_as_float(0);
		}
	break;


	case DREG_VELOCITY_N :
		if (packet_is_batch) {
			north_vel = read_register_as_float(0);
			east_vel = read_register_as_float(4);
			up_vel = read_register_as_float(8);
			vel_time = read_register_as_float(12);
		}
		else {
			north_vel = read_register_as_float(0);
		}
	break;


	case DREG_GPS_LATITUDE : // Only available if GPS is installed with coms set on TX2/RX2
		if (packet_is_batch) {
			lattitude = read_register_as_float(0);
			longitude = read_register_as_float(4);
			altitude = read_register_as_float(8);
			course = read_register_as_float(12);
			speed = read_register_as_float(16);
			gps_time = read_register_as_float(20);
		}
		else {
			lattitude = read_register_as_float(0);
		}
	break;


	case DREG_GPS_SAT_1_2 : // Only available if GPS is installed with coms set on TX2/RX2
		// Satellite 1 is satellite_id[0]
		if (packet_is_batch) {
			satellite_id[0] = (uint8_t)(data[0]);
			satellite_SNR[0] = (uint8_t)(data[1]);
			satellite_id[1] = (uint8_t)(data[2]);
			satellite_SNR[1] = (uint8_t)(data[3]);
			satellite_id[2] = (uint8_t)(data[4]);
			satellite_SNR[2] = (uint8_t)(data[5]);
			satellite_id[3] = (uint8_t)(data[6]);
			satellite_SNR[3] = (uint8_t)(data[7]);
			satellite_id[4] = (uint8_t)(data[8]);
			satellite_SNR[4] = (uint8_t)(data[9]);
			satellite_id[5] = (uint8_t)(data[10]);
			satellite_SNR[5] = (uint8_t)(data[11]);
			satellite_id[6] = (uint8_t)(data[12]);
			satellite_SNR[6] = (uint8_t)(data[13]);
			satellite_id[7] = (uint8_t)(data[14]);
			satellite_SNR[7] = (uint8_t)(data[15]);
			satellite_id[8] = (uint8_t)(data[16]);
			satellite_SNR[8] = (uint8_t)(data[17]);
			satellite_id[9] = (uint8_t)(data[18]);
			satellite_SNR[9] = (uint8_t)(data[19]);
			satellite_id[10] = (uint8_t)(data[20]);
			satellite_SNR[10] = (uint8_t)(data[21]);
			satellite_id[11] = (uint8_t)(data[22]);
			satellite_SNR[11] = (uint8_t)(data[23]);
		}
		else {
			satellite_id[0] = (uint8_t)(data[0]);
			satellite_SNR[0] = (uint8_t)(data[1]);
			satellite_id[1] = (uint8_t)(data[2]);
			satellite_SNR[1] = (uint8_t)(data[3]);
		}
	break;


	case DREG_GYRO_BIAS_X :
		if (packet_is_batch) {
			gyro_bias_x = read_register_as_float(0);
			gyro_bias_y = read_register_as_float(4);
			gyro_bias_z = read_register_as_float(8);
		}
		else {
			gyro_bias_x = read_register_as_float(0);
		}
	break;
	}
}

/*
	Union function which links a float to 4 bytes. Used to combine 4 bytes to one register.
*/
typedef union {
	float val;
	uint8_t bytes[4];
} floatval;

/*
	
*/
union combine {
	float f;
	uint8_t b[4];
};

/*
	Makes reading save() easier, this function reads a 4 byte register into a float (int32_t)
*/
float UM7::read_register_as_float(int firstByte) { // For one register as an IEEE floatpoint
	floatval temp;
	temp.bytes[3] = data[(firstByte)];
	temp.bytes[2] = data[(firstByte + 1)];
	temp.bytes[1] = data[(firstByte + 2)];
	temp.bytes[0] = data[(firstByte + 3)];
	return temp.val;
}

/*
	Parses error into string outputs
*/
void UM7::error_status() {
	
}

//////////////////////////////////////////////
//	COMMAND FUNCTIONS FOR THE UM7       //
//////////////////////////////////////////////

/*
	Sets the UM7 baud rate. If not called, default is 115200bps.
*/
void UM7::set_sensor_baud_rate(float baud) {
	byte rate = 0;

	if (baud == 9600) rate = 0x00;
	else if (baud == 14400) rate = 0x10;
	else if (baud == 19200) rate = 0x20;
	else if (baud == 38400) rate = 0x30;
	else if (baud == 57600) rate = 0x40;
	else if (baud == 115200) rate = 0x50;
	else if (baud == 128000) rate = 0x60;
	else if (baud == 153000) rate = 0x70;
	else if (baud == 230400) rate = 0x80;
	else if (baud == 256000) rate = 0x90;
	else if (baud == 460800) rate = 0xA0;
	else if (baud == 921600) rate = 0xB0;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_SETTINGS; // address

	config_buffer[5] = rate; // B3 (UM7 baud rate, 4b) | (GPS baud rate, 4b)
	config_buffer[6] = 0; // B2 Reserved
	config_buffer[7] = 0; // B1 (Reserved, 7b) | (Auto GPS transmission, 1b)
	config_buffer[8] = 0; // B0 (Reserved, 3b) | (Auto send satellite details, 1b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_SETTINGS + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Other form of set_sensor_baud_rate. Parameters are the UM7 baud rate as well as the gps baud rate.
	Combines into one register (1B) for command.
	GPS bit = Causes GPS data to be transmitted automatically once received externally, stored as a batch from DREG_GPS_LATITUDE
	SAT bit = Causes SAT data to be transmitted automatically once received externally, stored as batch DREG_GPS_SAT_1_2
*/
void UM7::set_sensor_baud_rate(float baud, float gps_baud, bool gps, bool sat) {
	byte rate = 0, gps_rate = 0, b1 = 0, b0 = 0;

	if (baud == 9600) rate = 0b0000;
	else if (baud == 14400) rate = 0b0001;
	else if (baud == 19200) rate = 0b0010;
	else if (baud == 38400) rate = 0b0011;
	else if (baud == 57600) rate = 0b0100;
	else if (baud == 115200) rate = 0b0101;
	else if (baud == 128000) rate = 0b0110;
	else if (baud == 153000) rate = 0b0111;
	else if (baud == 230400) rate = 0b1000;
	else if (baud == 256000) rate = 0b1001;
	else if (baud == 460800) rate = 0b1010;
	else if (baud == 921600) rate = 0b1011;

	if (gps_baud == 9600) gps_rate = 0b0000;
	else if (gps_baud == 14400) gps_rate = 0b0001;
	else if (gps_baud == 19200) gps_rate = 0b0010;
	else if (gps_baud == 38400) gps_rate = 0b0011;
	else if (gps_baud == 57600) gps_rate = 0b0100;
	else if (gps_baud == 115200) gps_rate = 0b0101;
	
	rate = ((rate << 4) | gps_rate);

	if (gps) b1 = 0b00000001;
	if (sat) b0 = 0b00010000;
	
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_SETTINGS; // address

	config_buffer[5] = rate; // B3 (UM7 baud rate, 4b) | (GPS baud rate, 4b)
	config_buffer[6] = 0; // B2 Reserved
	config_buffer[7] = b1; // B1 (Reserved, 7b) | (Auto GPS transmission, 1b)
	config_buffer[8] = b0; // B0 (Reserved, 3b) | (Auto send satellite details, 1b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_SETTINGS + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the individual rates for the raw datasets. 
	From 0 to 255 Hz, sent through their respective in their signed bytes.
*/
void UM7::set_raw_rate(uint8_t accel_rate, uint8_t gyro_rate, uint8_t mag_rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES1; // address

	config_buffer[5] = accel_rate; // B3 Raw accel rate
	config_buffer[6] = gyro_rate; // B2 Raw gyro rate
	config_buffer[7] = mag_rate; // B1 Raw mag rate
	config_buffer[8] = 0; // B0 Reserved

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES1 + accel_rate + gyro_rate + mag_rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all raw datasets to the same desired rate
*/
void UM7::set_all_raw_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES2; // address

	config_buffer[5] = 0; // B3 Reserved
	config_buffer[6] = 0; // B2 Reserved
	config_buffer[7] = 0; // B1 Reserved
	config_buffer[8] = rate; // B0 Raw Processed rate

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES2 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the temperature datasets as well as raw datasets.
*/
void UM7::set_all_raw_rate(uint8_t temp_rate, uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES2; // address

	config_buffer[5] = temp_rate; // B3 Reserved
	config_buffer[6] = 0; // B2 Reserved
	config_buffer[7] = 0; // B1 Reserved
	config_buffer[8] = rate; // B0 Raw Processed rate

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES2 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the individual rates for the processed datasets. 
	From 0 to 255 Hz, sent through their respective in their signed bytes.
*/
void UM7::set_processed_rate(uint8_t accel_rate, uint8_t gyro_rate, uint8_t mag_rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES3; // address

	config_buffer[5] = accel_rate; // B3 Raw accel rate
	config_buffer[6] = gyro_rate; // B2 Raw gyro rate
	config_buffer[7] = mag_rate; // B1 Raw mag rate
	config_buffer[8] = 0; // B0 Reserved

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES3 + accel_rate + gyro_rate + mag_rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all processed datasets to the same desired rate
*/
void UM7::set_all_processed_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES4; // address

	config_buffer[5] = 0; // B3 Reserved
	config_buffer[6] = 0; // B2 Reserved
	config_buffer[7] = 0; // B1 Reserved
	config_buffer[8] = rate; // B0 All Processed rate

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES4 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all quaternion datasets
*/
void UM7::set_quaternion_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES5; // address

	config_buffer[5] = rate; // B3 Quaternion rate
	config_buffer[6] = 0; // B2 Euler rate
	config_buffer[7] = 0; // B1 Position rate
	config_buffer[8] = 0; // B0 Velocity rate

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES5 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all euler angle datasets
*/
void UM7::set_euler_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES5; // address

	config_buffer[5] = 0; // B3 Quaternion rate
	config_buffer[6] = rate; // B2 Euler rate
	config_buffer[7] = 0; // B1 Position rate
	config_buffer[8] = 0; // B0 Velocity rate

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES5 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all position datasets
*/
void UM7::set_position_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES5; // address

	config_buffer[5] = 0; // B3 Quaternion rate
	config_buffer[6] = 0; // B2 Euler rate
	config_buffer[7] = rate; // B1 Position rate
	config_buffer[8] = 0; // B0 Velocity rate

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES5 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all velocity datasets
*/
void UM7::set_velocity_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES5; // address

	config_buffer[5] = 0; // B3 Quaternion rate
	config_buffer[6] = 0; // B2 Euler rate
	config_buffer[7] = 0; // B1 Position rate
	config_buffer[8] = rate; // B0 Velocity rate

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES5 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all position and euler angle datasets to the same desired rate
*/
void UM7::set_pose_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES6; // address

	config_buffer[5] = rate; // B3 Pose rate
	config_buffer[6] = 0; // B2 (Reserved, 4b) | (Health rate, 4b)
	config_buffer[7] = 0; // B1 Gyro bias rate
	config_buffer[8] = 0; // B0 Reserved

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES6 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the health packet. Default is 1 Hz.
*/
void UM7::set_health_rate(float baud) {
	byte rate = 0;

	if (baud == 0.125) rate = 0x01;
	else if (baud == 0.25) rate = 0x02;
	else if (baud == 0.5) rate = 0x03;
	else if (baud == 1) rate = 0x04;
	else if (baud == 2) rate = 0x05;
	else if (baud == 4) rate = 0x06;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES6; // address

	config_buffer[5] = 0; // B3 Pose rate
	config_buffer[6] = rate; // B2 (Reserved, 4b) | (Health rate, 4b)
	config_buffer[7] = 0; // B1 Gyro bias rate
	config_buffer[8] = 0; // B0 Reserved

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES6 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for all gyro bias datasets
*/
void UM7::set_gyro_bias_rate(uint8_t rate) {
	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES6; // address

	config_buffer[5] = 0; // B3 Pose rate
	config_buffer[6] = 0; // B2 (Reserved, 4b) | (Health rate, 4b)
	config_buffer[7] = rate; // B1 Gyro bias rate
	config_buffer[8] = 0; // B0 Reserved

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES6 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the NMEA health packet
*/
void UM7::set_NMEA_health_rate(int8_t baud) {
	byte rate = 0;

	if (baud == 1) rate = 0x10;
	else if (baud == 2) rate = 0x20;
	else if (baud == 4) rate = 0x30;
	else if (baud == 5) rate = 0x40;
	else if (baud == 10) rate = 0x50;
	else if (baud == 15) rate = 0x60;
	else if (baud == 20) rate = 0x70;
	else if (baud == 30) rate = 0x80;
	else if (baud == 40) rate = 0x90;
	else if (baud == 50) rate = 0xA0;
	else if (baud == 60) rate = 0xB0;
	else if (baud == 70) rate = 0xC0;
	else if (baud == 80) rate = 0xD0;
	else if (baud == 90) rate = 0xE0;
	else if (baud == 100) rate = 0xF0;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES7; // address

	config_buffer[5] = rate; // B3 (NMEA health rate, 4b) | (NMEA pose rate, 4b)
	config_buffer[6] = 0; // B2 (NMEA attitude rate, 4b) | (NMEA sensor rate, 4b)
	config_buffer[7] = 0; // B1 (NMEA data rates rate, 4b) | (NMEA GPS pose rate, 4b)
	config_buffer[8] = 0; // B0 (NMEA quaternion rate, 4b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES7 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the NMEA position and euler angle datasets
*/
void UM7::set_NMEA_pose_rate(int8_t baud) {
	byte rate = 0;

	if (baud == 1) rate = 0x01;
	else if (baud == 2) rate = 0x02;
	else if (baud == 4) rate = 0x03;
	else if (baud == 5) rate = 0x04;
	else if (baud == 10) rate = 0x05;
	else if (baud == 15) rate = 0x06;
	else if (baud == 20) rate = 0x07;
	else if (baud == 30) rate = 0x08;
	else if (baud == 40) rate = 0x09;
	else if (baud == 50) rate = 0x0A;
	else if (baud == 60) rate = 0x0B;
	else if (baud == 70) rate = 0x0C;
	else if (baud == 80) rate = 0x0D;
	else if (baud == 90) rate = 0x0E;
	else if (baud == 100) rate = 0x0F;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES7; // address

	config_buffer[5] = rate; // B3 (NMEA health rate, 4b) | (NMEA pose rate, 4b)
	config_buffer[6] = 0; // B2 (NMEA attitude rate, 4b) | (NMEA sensor rate, 4b)
	config_buffer[7] = 0; // B1 (NMEA data rates rate, 4b) | (NMEA GPS pose rate, 4b)
	config_buffer[8] = 0; // B0 (NMEA quaternion rate, 4b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES7 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the NMEA attitude datasets
*/
void UM7::set_NMEA_attitude_rate(int8_t baud) {
	byte rate = 0;

	if (baud == 1) rate = 0x10;
	else if (baud == 2) rate = 0x20;
	else if (baud == 4) rate = 0x30;
	else if (baud == 5) rate = 0x40;
	else if (baud == 10) rate = 0x50;
	else if (baud == 15) rate = 0x60;
	else if (baud == 20) rate = 0x70;
	else if (baud == 30) rate = 0x80;
	else if (baud == 40) rate = 0x90;
	else if (baud == 50) rate = 0xA0;
	else if (baud == 60) rate = 0xB0;
	else if (baud == 70) rate = 0xC0;
	else if (baud == 80) rate = 0xD0;
	else if (baud == 90) rate = 0xE0;
	else if (baud == 100) rate = 0xF0;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES7; // address

	config_buffer[5] = 0; // B3 (NMEA health rate, 4b) | (NMEA pose rate, 4b)
	config_buffer[6] = rate; // B2 (NMEA attitude rate, 4b) | (NMEA sensor rate, 4b)
	config_buffer[7] = 0; // B1 (NMEA data rates rate, 4b) | (NMEA GPS pose rate, 4b)
	config_buffer[8] = 0; // B0 (NMEA quaternion rate, 4b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES7 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the desired broadcast rate for NEMA sensor packets
*/
void UM7::set_NMEA_sensor_rate(int8_t baud) {
	byte rate = 0;

	if (baud == 1) rate = 0x01;
	else if (baud == 2) rate = 0x02;
	else if (baud == 4) rate = 0x03;
	else if (baud == 5) rate = 0x04;
	else if (baud == 10) rate = 0x05;
	else if (baud == 15) rate = 0x06;
	else if (baud == 20) rate = 0x07;
	else if (baud == 30) rate = 0x08;
	else if (baud == 40) rate = 0x09;
	else if (baud == 50) rate = 0x0A;
	else if (baud == 60) rate = 0x0B;
	else if (baud == 70) rate = 0x0C;
	else if (baud == 80) rate = 0x0D;
	else if (baud == 90) rate = 0x0E;
	else if (baud == 100) rate = 0x0F;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES7; // address

	config_buffer[5] = 0; // B3 (NMEA health rate, 4b) | (NMEA pose rate, 4b)
	config_buffer[6] = rate; // B2 (NMEA attitude rate, 4b) | (NMEA sensor rate, 4b)
	config_buffer[7] = 0; // B1 (NMEA data rates rate, 4b) | (NMEA GPS pose rate, 4b)
	config_buffer[8] = 0; // B0 (NMEA quaternion rate, 4b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES7 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the NMEA sensor datasets
*/
void UM7::set_NMEA_rates_rate(int8_t baud) {
	byte rate = 0;

	if (baud == 1) rate = 0x10;
	else if (baud == 2) rate = 0x20;
	else if (baud == 4) rate = 0x30;
	else if (baud == 5) rate = 0x40;
	else if (baud == 10) rate = 0x50;
	else if (baud == 15) rate = 0x60;
	else if (baud == 20) rate = 0x70;
	else if (baud == 30) rate = 0x80;
	else if (baud == 40) rate = 0x90;
	else if (baud == 50) rate = 0xA0;
	else if (baud == 60) rate = 0xB0;
	else if (baud == 70) rate = 0xC0;
	else if (baud == 80) rate = 0xD0;
	else if (baud == 90) rate = 0xE0;
	else if (baud == 100) rate = 0xF0;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES7; // address

	config_buffer[5] = 0; // B3 (NMEA health rate, 4b) | (NMEA pose rate, 4b)
	config_buffer[6] = 0; // B2 (NMEA attitude rate, 4b) | (NMEA sensor rate, 4b)
	config_buffer[7] = rate; // B1 (NMEA data rates rate, 4b) | (NMEA GPS pose rate, 4b)
	config_buffer[8] = 0; // B0 (NMEA quaternion rate, 4b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES7 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the NMEA GPS pose datasets
*/
void UM7::set_NMEA_GPS_pose_rate(int8_t baud) {
	byte rate = 0;

	if (baud == 1) rate = 0x01;
	else if (baud == 2) rate = 0x02;
	else if (baud == 4) rate = 0x03;
	else if (baud == 5) rate = 0x04;
	else if (baud == 10) rate = 0x05;
	else if (baud == 15) rate = 0x06;
	else if (baud == 20) rate = 0x07;
	else if (baud == 30) rate = 0x08;
	else if (baud == 40) rate = 0x09;
	else if (baud == 50) rate = 0x0A;
	else if (baud == 60) rate = 0x0B;
	else if (baud == 70) rate = 0x0C;
	else if (baud == 80) rate = 0x0D;
	else if (baud == 90) rate = 0x0E;
	else if (baud == 100) rate = 0x0F;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES7; // address

	config_buffer[5] = 0; // B3 (NMEA health rate, 4b) | (NMEA pose rate, 4b)
	config_buffer[6] = 0; // B2 (NMEA attitude rate, 4b) | (NMEA sensor rate, 4b)
	config_buffer[7] = rate; // B1 (NMEA data rates rate, 4b) | (NMEA GPS pose rate, 4b)
	config_buffer[8] = 0; // B0 (NMEA quaternion rate, 4b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES7 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Sets the rate for the NMEA quaternion datasets
*/
void UM7::set_NMEA_quaternion_rate(int8_t baud) {
	byte rate = 0;

	if (baud == 1) rate = 0x10;
	else if (baud == 2) rate = 0x20;
	else if (baud == 4) rate = 0x30;
	else if (baud == 5) rate = 0x40;
	else if (baud == 10) rate = 0x50;
	else if (baud == 15) rate = 0x60;
	else if (baud == 20) rate = 0x70;
	else if (baud == 30) rate = 0x80;
	else if (baud == 40) rate = 0x90;
	else if (baud == 50) rate = 0xA0;
	else if (baud == 60) rate = 0xB0;
	else if (baud == 70) rate = 0xC0;
	else if (baud == 80) rate = 0xD0;
	else if (baud == 90) rate = 0xE0;
	else if (baud == 100) rate = 0xF0;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_COM_RATES7; // address

	config_buffer[5] = 0; // B3 (NMEA health rate, 4b) | (NMEA pose rate, 4b)
	config_buffer[6] = 0; // B2 (NMEA attitude rate, 4b) | (NMEA sensor rate, 4b)
	config_buffer[7] = 0; // B1 (NMEA data rates rate, 4b) | (NMEA GPS pose rate, 4b)
	config_buffer[8] = rate; // B0 (NMEA quaternion rate, 4b) | (Reserved, 4b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_COM_RATES7 + rate;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Miscellaneous settings for filter and sensor control options. Send a 0 if you don't wish to configure a specific setting
	Ex. set_misc_settings(0, 1, 1, 0)

	PPS bit = Causes the TX2/RX2 pin to be used with an external GPS
	ZG bit = Causes UM7 to measure gyro bias at setup
	Q bit = Sensor will run in Quternion mode instead of Euler mode. Fixes pitch error in the Gimbal lock position
	MAG bit = Magnetometer will be used in state updates
*/
void UM7::set_misc_ssettings(bool pps, bool zg, bool q, bool mag) {
	uint8_t b1 = 0, b0 = 0;

	if (pps) b1 = 0b00000001;
	
	if (zg) { b0 = 00000100;
		if (q) { b0 = 00000110;
			if (mag) b0 = 00000111;
		}
		if (mag) b0 = 00000101;
	}
	if (q) { b0 = 00000010;
		if (mag) b0 = 00000011;
	}
	if (mag) b0 = 00000001;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_MISC_SETTINGS; // address

	config_buffer[5] = 0; // B3 Reserved
	config_buffer[6] = 0; // B2 Reserved
	config_buffer[7] = b1; // B1 (Reserved, 7b) | (PPS, 1b)
	config_buffer[8] = b0; // B0 (Reserved, 5b) | (ZG, 1b) | (Q, 1b) | (MAG, 1b)

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_MISC_SETTINGS + b1 + b0;

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Configuration for hard setting the north orientation vector
*/
void  UM7::set_home_north(float north) {
	combine n = { north };

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_HOME_NORTH; // address

	config_buffer[5] = n.b[0]; // B3
	config_buffer[6] = n.b[1]; // B2 
	config_buffer[7] = n.b[2]; // B1 
	config_buffer[8] = n.b[3]; // B0 

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_HOME_NORTH + n.b[0] + n.b[1] + n.b[2] + n.b[3];

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Configuration for hard setting the east orientation vector
*/
void  UM7::set_home_east(float east) {
	floatval e;
	e.val = east;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_HOME_EAST; // address

	config_buffer[5] = e.bytes[0]; // B3
	config_buffer[6] = e.bytes[1]; // B2 
	config_buffer[7] = e.bytes[2]; // B1 
	config_buffer[8] = e.bytes[3]; // B0 

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_HOME_EAST + e.bytes[0] + e.bytes[1] + e.bytes[2] + e.bytes[3];

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Configuration for hard setting the up orientation vector
*/
void  UM7::set_home_up(float up) {
	floatval u;
	u.val = up;

	config_buffer[0] = 's';
	config_buffer[1] = 'n';
	config_buffer[2] = 'p';
	config_buffer[3] = 0x80; // PT byte = 1000 0000.
	config_buffer[4] = CREG_HOME_UP; // address

	config_buffer[5] = u.bytes[0]; // B3
	config_buffer[6] = u.bytes[1]; // B2 
	config_buffer[7] = u.bytes[2]; // B1 
	config_buffer[8] = u.bytes[3]; // B0 

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_HOME_UP + u.bytes[0] + u.bytes[1] + u.bytes[2] + u.bytes[3];

	// Parsing checksumsum
	config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
	config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

	serial_port->write(config_buffer, 11);
}

/*
	Configuration for the gyro trim in XYZ. It's calculated additioanly to the intial bias compensation (done in 
	the ZERO_GYRO_BIAS command).
*/
void  UM7::set_gyro_trim(float trim_x, float trim_y, float trim_z) {
	for (int i = 0; i < 3; i++) {
		floatval n;
		if (i == 0) n.val = trim_x;
		if (i == 1) n.val = trim_y;
		if (i == 2) n.val = trim_z;

		config_buffer[0] = 's';
		config_buffer[1] = 'n';
		config_buffer[2] = 'p';
		config_buffer[3] = 0x80; // PT byte = 1000 0000.
		config_buffer[4] = CREG_GYRO_TRIM_X + i; // address

		config_buffer[5] = n.bytes[0]; // B3
		config_buffer[6] = n.bytes[1]; // B2 
		config_buffer[7] = n.bytes[2]; // B1 
		config_buffer[8] = n.bytes[3]; // B0 

		uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_GYRO_TRIM_X + i + n.bytes[0] + n.bytes[1] + n.bytes[2] + n.bytes[3];

		// Parsing checksumsum
		config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
		config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

		serial_port->write(config_buffer, 11);
	}
}

/*
	Uses 9 entries to perform a soft-iron calibration of the magnetometer. These terms are computed from the RedShiftLabs
	Serial Interface

	3D array requires testing...
*/
void  UM7::soft_iron_magnetometer_calibration(float (*array)[3][3]) {
	for (int i = 0; i < 3; i++) { // Can cycle through addresses with the index
		for (int j = 0; j < 3; j++) {
			floatval n;
			n.val = (*array)[i][j];

			config_buffer[0] = 's';
			config_buffer[1] = 'n';
			config_buffer[2] = 'p';
			config_buffer[3] = 0x80; // PT byte = 1000 0000.
			config_buffer[4] = CREG_MAG_CAL1_1 + i + j; // address

			config_buffer[5] = n.bytes[0]; // B3
			config_buffer[6] = n.bytes[1]; // B2 
			config_buffer[7] = n.bytes[2]; // B1 
			config_buffer[8] = n.bytes[3]; // B0 

			uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_MAG_CAL1_1 + 
				i + j + n.bytes[0] + n.bytes[1] + n.bytes[2] + n.bytes[3];

			// Parsing checksumsum
			config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
			config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

			serial_port->write(config_buffer, 11);
		}
	}
}

/*
	Performs a hard-iron calibration of the magnetometer. These terms are computed from the RedShiftLabs
	Serial Interface
*/
void  UM7::hard_iron_magnetometer_calibration(float bias_x, float bias_y, float bias_z) {
	for (int i = 0; i < 3; i++) {
		floatval n;
		if (i == 0) n.val = bias_x;
		if (i == 1) n.val = bias_y;
		if (i == 2) n.val = bias_z;

		config_buffer[0] = 's';
		config_buffer[1] = 'n';
		config_buffer[2] = 'p';
		config_buffer[3] = 0x80; // PT byte = 1000 0000.
		config_buffer[4] = CREG_MAG_BIAS_X + i; // address

		config_buffer[5] = n.bytes[0]; // B3
		config_buffer[6] = n.bytes[1]; // B2 
		config_buffer[7] = n.bytes[2]; // B1 
		config_buffer[8] = n.bytes[3]; // B0 

		uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_MAG_BIAS_X + i + n.bytes[0] + n.bytes[1] + n.bytes[2] + n.bytes[3];

		// Parsing checksumsum
		config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
		config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

		serial_port->write(config_buffer, 11);
	}
}

/*
	Uses 9 entries to the cross axis misalignment of the accelerometer. 

	3D array requires testing...
*/
void  UM7::accelerometer_misalignment_compensation(float (*array)[3][3]) {
	for (int i = 0; i < 3; i++) { // Can cycle through addresses with the index
		for (int j = 0; j < 3; j++) {
			floatval n;
			n.val = (*array)[i][j];

			config_buffer[0] = 's';
			config_buffer[1] = 'n';
			config_buffer[2] = 'p';
			config_buffer[3] = 0x80; // PT byte = 1000 0000.
			config_buffer[4] = CREG_ACCEL_CAL1_1 + i + j; // address

			config_buffer[5] = n.bytes[0]; // B3
			config_buffer[6] = n.bytes[1]; // B2 
			config_buffer[7] = n.bytes[2]; // B1 
			config_buffer[8] = n.bytes[3]; // B0 

			uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_ACCEL_CAL1_1 +
				i + j + n.bytes[0] + n.bytes[1] + n.bytes[2] + n.bytes[3];

			// Parsing checksumsum
			config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
			config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

			serial_port->write(config_buffer, 11);
		}
	}
}

/*
	Performs a calibration for the accelerometer bias in XYZ directions. These terms are computed from the RedShiftLabs
	Serial Interface
*/
void  UM7::accelerometer_calibration(float bias_x, float bias_y, float bias_z) {
	for (int i = 0; i < 3; i++) {
		floatval n;
		if (i == 0) n.val = bias_x;
		if (i == 1) n.val = bias_y;
		if (i == 2) n.val = bias_z;

		config_buffer[0] = 's';
		config_buffer[1] = 'n';
		config_buffer[2] = 'p';
		config_buffer[3] = 0x80; // PT byte = 1000 0000.
		config_buffer[4] = CREG_ACCEL_BIAS_X + i; // address

		config_buffer[5] = n.bytes[0]; // B3
		config_buffer[6] = n.bytes[1]; // B2 
		config_buffer[7] = n.bytes[2]; // B1 
		config_buffer[8] = n.bytes[3]; // B0 

		uint16_t checksumsum = 's' + 'n' + 'p' + 0x80 + CREG_ACCEL_BIAS_X + i + n.bytes[0] + n.bytes[1] + n.bytes[2] + n.bytes[3];

		// Parsing checksumsum
		config_buffer[10] = checksumsum & 0xFF; // Checksum LOW byte
		config_buffer[9] = (checksumsum >> 8); // Checksum HIGH byte

		serial_port->write(config_buffer, 11);
	}
}

/*
	Causes UM7 to transmit a packet containing the firmware revision string (a 4B char sequence)

	Does it work???
*/
char* UM7::get_firmware_revision() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = GET_FW_REVISION; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + GET_FW_REVISION;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);

	serial_port->write(cmd_buffer, 7);

	
	serial_port->readBytes(firmware, 4);
	
	
	return firmware;
}

/*
	Causes the UM7 to write all configuration settings to FLASH so that they will remain when the power is cycled.
*/
void UM7::save_configs_to_flash() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = GET_FW_REVISION; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + GET_FW_REVISION;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);

	serial_port->write(cmd_buffer, 7);
}

/*
	Causes the UM7 to load default factory settings.
*/
void UM7::factory_reset() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = RESET_TO_FACTORY; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + RESET_TO_FACTORY;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);

	serial_port->write(cmd_buffer, 7);
	Serial.println("FACTORY RESET UM7"); // default serial_port 0
}

/*
	Causes the UM7 to measure the gyro outputs and set the output trim registers to compensate for any non-zero bias. 
	The UM7 should be kept stationary while the zero operation is underway.
*/
void UM7::zero_gyros() { // Doesn't check for COMMAND_COMPLETE byte, only sends cmd
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = ZERO_GYROS; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + ZERO_GYROS;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);
	
	serial_port->write(cmd_buffer, 7);
}

/*
	Sets the current GPS latitude, longitude, and altitude as the home position. 
	All future positions will be referenced to the current GPS position.
*/
void  UM7::set_home_position() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = SET_HOME_POSITION; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + SET_HOME_POSITION;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);

	serial_port->write(cmd_buffer, 7);
}

/*
	Sets the current yaw heading position as north.
*/
void  UM7::set_mag_reference() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = SET_MAG_REFERENCE; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + SET_MAG_REFERENCE;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);

	serial_port->write(cmd_buffer, 7);
}

/*
	Reboots the UM7 and performs a crude calibration on the accelerometers. Best performed on a flat surface.
*/
void UM7::calibrate_accelerometers() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = CALIBRATE_ACCELEROMETERS; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + CALIBRATE_ACCELEROMETERS;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);

	serial_port->write(cmd_buffer, 7);
}

/*
	Resets the EKF. Extended Kalman Filter (EKF)
*/
void UM7::reset_kalman_filter() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0x00; // PT byte = 0000 0000 for command register
	cmd_buffer[4] = RESET_EKF; // address

	uint16_t checksumsum = 's' + 'n' + 'p' + 0x00 + RESET_EKF;

	cmd_buffer[6] = checksumsum & 0xFF;
	cmd_buffer[5] = (checksumsum >> 8);

	serial_port->write(cmd_buffer, 7);
}
